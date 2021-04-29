/* Copyright 2014 Gareth Cross

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License. */
#include "aerialmap_display.hpp"

#include <utility>
#include <string>
#include <iomanip>
#include <geodesy/wgs84.h>
#include <geodesy/utm.h>

#include <OgreManualObject.h>
#include <OgreMaterialManager.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreTextureManager.h>
#include <OgreTechnique.h>

#include "rviz_common/validate_floats.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/logging.hpp"

#include "rviz_default_plugins/transformation/tf_wrapper.hpp"

namespace rviz_satellite
{

/**
 * @file
 * The sequence of events is rather complex due to the asynchronous nature of the tile texture updates, and the
 * different coordinate systems and frame transforms involved:
 *
 * The navSatFixCallback calls the updateCenterTile function, which then queries a texture update and calls
 * transformTileToMapFrame. The latter finds and stores the transform from the NavSatFix frame to the map-frame, to
 * which the tiles are rigidly attached by ENU convention and Mercator projection. On each frame, update() is called,
 * which calls transformMapTileToFixedFrame, which then transforms the tile-map from the map-frame to the fixed-frame.
 * Splitting this transform lookup is necessary to mitigate frame jitter.
 */

using rviz_common::properties::Property;
using rviz_common::properties::FloatProperty;
using rviz_common::properties::IntProperty;
using rviz_common::properties::RosTopicProperty;
using rviz_common::properties::StringProperty;
using rviz_common::properties::StatusProperty;

using sensor_msgs::msg::NavSatFix;
using geographic_msgs::msg::GeoPoint;

char const AerialMapDisplay::UTM_FRAME[] = "utm";

AerialMapDisplay::AerialMapDisplay()
: RosTopicDisplay()
{
  alpha_property_ =
    new FloatProperty(
    "Alpha", 0.7, "Amount of transparency to apply to the map.", this,
    SLOT(updateAlpha()));
  alpha_property_->setMin(0);
  alpha_property_->setMax(1);
  alpha_property_->setShouldBeSaved(true);

  draw_under_property_ = new Property(
    "Draw Behind", false,
    "Rendering option, controls whether or not the map is always"
    " drawn behind everything else.",
    this, SLOT(updateDrawUnder()));
  draw_under_property_->setShouldBeSaved(true);

  // properties for map
  tile_url_property_ =
    new StringProperty(
    "Object URI", "", "URL from which to retrieve map tiles.", this,
    SLOT(updateTileUrl()));
  tile_url_property_->setShouldBeSaved(true);

  QString const zoom_desc = QString::fromStdString(
    "Zoom level (0 - " + std::to_string(
      MAX_ZOOM) + ")");
  zoom_property_ = new IntProperty("Zoom", 16, zoom_desc, this, SLOT(updateZoom()));
  zoom_property_->setMin(0);
  zoom_property_->setMax(MAX_ZOOM);
  zoom_property_->setShouldBeSaved(true);

  QString const blocks_desc =
    QString::fromStdString("Adjacent blocks (0 - " + std::to_string(MAX_BLOCKS) + ")");
  blocks_property_ = new IntProperty("Blocks", 3, blocks_desc, this, SLOT(updateBlocks()));
  blocks_property_->setMin(0);
  blocks_property_->setMax(MAX_BLOCKS);
  blocks_property_->setShouldBeSaved(true);
}

AerialMapDisplay::~AerialMapDisplay()
{
}

void AerialMapDisplay::onInitialize()
{
  RTDClass::onInitialize();
}

void AerialMapDisplay::onEnable()
{
  scene_node_->setVisible(true);
}

void AerialMapDisplay::onDisable()
{
  scene_node_->setVisible(false);
}

bool AerialMapDisplay::validateMessage(const NavSatFix::ConstSharedPtr message)
{
  bool message_is_valid = true;
  if (!rviz_common::validateFloats(message->latitude) ||
    !rviz_common::validateFloats(message->longitude))
  {
    setStatus(
      rviz_common::properties::StatusProperty::Error, "Message",
      "Message contained invalid floating point values (nans or infs)");
    message_is_valid = false;
  }
  return message_is_valid;
}

void AerialMapDisplay::updateAlpha()
{
  auto alpha = alpha_property_->getFloat();
  for (auto & object : tiles_) {
    object.second.updateAlpha(alpha);
  }
}

void AerialMapDisplay::updateDrawUnder()
{
  for (auto & tile : tiles_) {
    auto & object = tile.second;
    if (draw_under_property_->getValue().toBool()) {
      object.setRenderQueueGroup(Ogre::RENDER_QUEUE_3);
    } else {
      object.setRenderQueueGroup(Ogre::RENDER_QUEUE_MAIN);
    }
  }
}

void AerialMapDisplay::updateTileUrl()
{
  // rebuild on next received message
  resetMap();
}

void AerialMapDisplay::updateZoom()
{
  // rebuild on next received message
  resetMap();
}

void AerialMapDisplay::updateBlocks()
{
  // rebuild on next received message
  resetMap();
}

void AerialMapDisplay::processMessage(const NavSatFix::ConstSharedPtr msg)
{
  if (!isEnabled()) {
    // if not enabled, don't incur network traffic
    return;
  }
  if (!validateMessage(msg)) {
    return;
  }
  if (tiles_.empty() && !tile_url_property_->getStdString().empty()) {
    // TODO react to changing center tile
    buildObjects(geodesy::toMsg(*msg));
  }
}

void AerialMapDisplay::buildObjects(const GeoPoint & center)
{
  auto tile_server = tile_url_property_->getStdString();
  auto zoom = zoom_property_->getInt();
  auto tile_url = tile_url_property_->getStdString();
  auto center_tile = fromWGS(center, zoom);
  auto blocks = blocks_property_->getInt();

  geodesy::UTMPoint utm_center(center);

  for (int x = -blocks; x <= blocks; ++x) {
    for (int y = -blocks; y <= blocks; ++y) {

      const TileId tile_id{tile_server, {center_tile.x + x, center_tile.y + y, zoom}};
      pending_tiles_.emplace(tile_id, tile_client_.request(tile_id));

      auto size = zoomSize(center.latitude, zoom);
      double tx = utm_center.easting + x * size;
      double ty = utm_center.northing + y * size;
      std::stringstream ss;
      ss << tile_id;
      tiles_.emplace(
        std::piecewise_construct,
        std::forward_as_tuple(tile_id),
        std::forward_as_tuple(
          scene_manager_, scene_node_,
          ss.str(), size, tx, ty, false));
    }
  }
  // set material properties
  updateAlpha();
  updateDrawUnder();
}

void AerialMapDisplay::update(float, float)
{
  for (auto it = pending_tiles_.begin(); it != pending_tiles_.end(); ) {
    if (it->second.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
      try {
        auto image = it->second.get();
        tiles_.find(it->first)->second.updateData(image);
      } catch (const tile_request_error & e) {
        RVIZ_COMMON_LOG_ERROR(e.what());
        setStatus(rviz_common::properties::StatusProperty::Error, "TileRequest", e.what());
      }
      // remove from pending requests
      it = pending_tiles_.erase(it);
    } else {
      // check next request
      ++it;
    }
  }

  Ogre::Vector3 translation;
  Ogre::Quaternion orientation;
  // transform the entire satellite map from the UTM frame to the fixed frame
  if (context_->getFrameManager()->getTransform(std::string(UTM_FRAME), translation, orientation)) {
    setStatus(rviz_common::properties::StatusProperty::Ok, "Transform", "Transform OK");
    scene_node_->setPosition(translation);
    scene_node_->setOrientation(orientation);
    scene_node_->setVisible(true);
  } else {
    std::string error;
    if (context_->getFrameManager()->transformHasProblems(
        std::string(UTM_FRAME), context_->getClock()->now(), error))
    {
      setStatus(
        rviz_common::properties::StatusProperty::Error, "Transform", QString::fromStdString(error));
    } else {
      setMissingTransformToFixedFrame(std::string(UTM_FRAME));
    }
    scene_node_->setVisible(false);
  }
}

void AerialMapDisplay::resetMap()
{
  tiles_.clear();
  pending_tiles_.clear();
}

void AerialMapDisplay::reset()
{
  RTDClass::reset();
  resetMap();
}

}  // namespace rviz_satellite

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(rviz_satellite::AerialMapDisplay, rviz_common::Display)
