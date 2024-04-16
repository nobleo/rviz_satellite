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
#include "field.hpp"

#include <limits>
#include <algorithm>
#include <utility>
#include <string>

#include <OgreManualObject.h>
#include <OgreMaterialManager.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreTextureManager.h>
#include <OgreTechnique.h>

#include <rcpputils/asserts.hpp>
#include "rviz_common/validate_floats.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/logging.hpp"
#include "rviz_common/msg_conversions.hpp"

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

// disable cpplint: not using string as const char*
// declaring as std::string and QString to avoid copies
const std::string AerialMapDisplay::MAP_FRAME = "map"; // NOLINT
const QString AerialMapDisplay::MESSAGE_STATUS = "Message"; // NOLINT
const QString AerialMapDisplay::TILE_REQUEST_STATUS = "TileRequest"; // NOLINT
const QString AerialMapDisplay::PROPERTIES_STATUS = "Properties"; // NOLINT
const QString AerialMapDisplay::ORIENTATION_STATUS = "Orientation"; // NOLINT
const QString AerialMapDisplay::TRANSFORM_STATUS = "Transform"; // NOLINT

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

  timeout_property_ =
    new FloatProperty(
    "Timeout", 3.0,
    "Message header timestamp timeout in seconds. Will start to fade out at half time, ignored if 0.",
    this);
  timeout_property_->setMin(0.0);
  timeout_property_->setShouldBeSaved(true);

  tf_tolerance_property_ =
    new FloatProperty(
    "TF tolerance", 0.1,
    "Maximum allowed age of latest transformation looked up from TF.",
    this);
  tf_tolerance_property_->setMin(0.0);
  tf_tolerance_property_->setShouldBeSaved(true);
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
  resetTileServerError();
  resetMap();
}

bool AerialMapDisplay::validateMessage(const NavSatFix::ConstSharedPtr message)
{
  bool message_is_valid = true;
  if (!rviz_common::validateFloats(message->latitude) ||
    !rviz_common::validateFloats(message->longitude))
  {
    setStatus(
      rviz_common::properties::StatusProperty::Error, MESSAGE_STATUS,
      "Message contains invalid floating point values (nans or infs)");
    message_is_valid = false;
  }
  if (message->status.status == sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX) {
    setStatus(
      rviz_common::properties::StatusProperty::Error, MESSAGE_STATUS,
      "NavSatFix status NO_FIX");
    message_is_valid = false;
  }
  return message_is_valid;
}

bool AerialMapDisplay::validateProperties()
{
  if (tile_url_property_->getStdString().empty()) {
    setStatus(
      rviz_common::properties::StatusProperty::Warn, PROPERTIES_STATUS,
      "Object URI is required to fetch map tiles");
    return false;
  }
  return true;
}

void AerialMapDisplay::updateAlpha()
{
  auto t = rviz_ros_node_.lock()->get_raw_node()->get_clock()->now();
  updateAlpha(t);
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
  // updated tile url may work
  resetTileServerError();
  // rebuild on next received message
  resetMap();
}

void AerialMapDisplay::updateZoom()
{
  // updated zoom may be supported by this tile server
  resetTileServerError();
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
  } else {
    setStatus(
      rviz_common::properties::StatusProperty::Ok, MESSAGE_STATUS,
      "Message OK");
  }
  if (tile_server_had_errors_) {
    return;
  }
  last_fix_ = msg;
  if (!validateProperties()) {
    return;
  } else {
    deleteStatus(PROPERTIES_STATUS);
  }
  auto zoom = zoom_property_->getInt();
  auto tile_at_fix = fromWGS(*msg, zoom);
  {
    const std::lock_guard<std::mutex> lock(tiles_mutex_);
    try {
      double tile_size_m = zoomSize(msg->latitude, zoom);
      if (tiles_.empty()) {
        // create whole map initially
        buildMap(tile_at_fix, tile_size_m);
      } else {
        auto center = centerTile();
        auto offset = Ogre::Vector2i(tile_at_fix.x - center.x, tile_at_fix.y - center.y);
        if (!offset.isZeroLength()) {
          auto blocks = blocks_property_->getInt();
          if (blocks > 0 && std::abs(offset.data[0]) <= blocks &&
            std::abs(offset.data[1]) <= blocks)
          {
            // if center tile changed to some index direction, within the bounds of the surrounding blocks,
            // create only the missing tiles
            shiftMap(center, offset, tile_size_m);
          } else {
            // if more tiles than blocks are skipped, recreate the entire map
            pending_tiles_.clear();
            tiles_.clear();
            buildMap(tile_at_fix, tile_size_m);
          }
        }
      }
    } catch (const tile_request_error & e) {
      tile_server_had_errors_ = true;
      pending_tiles_.clear();
      tiles_.clear();
      setStatus(rviz_common::properties::StatusProperty::Error, TILE_REQUEST_STATUS, e.what());
    }
  }
  // set material properties on created tiles
  updateAlpha(last_fix_->header.stamp);
  updateDrawUnder();
}

void AerialMapDisplay::shiftMap(TileCoordinate center, Ogre::Vector2i offset, double tile_size_m)
{
  int delta_x = offset.data[0];
  int delta_y = offset.data[1];

  // TODO(ZeilingerM) validate map borders
  int blocks = blocks_property_->getInt();
  auto tile_url = tile_url_property_->getStdString();

  // remove tiles on the far end
  for (auto far_offset : farEndOffsets(blocks, offset)) {
    int x = center.x + far_offset.data[0];
    int y = center.y + far_offset.data[1];
    const TileCoordinate coordinate_to_delete{x, y, center.z};
    const TileId tile_to_delete{tile_url, coordinate_to_delete};
    auto erased = tiles_.erase(tile_to_delete);
    // TODO(ZeilingerM) assertion is not correct on border of map
    rcpputils::assert_true(erased == 1, "failed to erase tile at far end");
  }

  // shift existing tiles to new center
  Ogre::Vector3 translation(-delta_x * tile_size_m, delta_y * tile_size_m, 0.0);
  for (auto & tile : tiles_) {
    tile.second.translate(translation);
  }

  // create tiles on the near end
  for (auto near_offset : nearEndOffsets(blocks, offset)) {
    int x = center.x + near_offset.data[0];
    int y = center.y + near_offset.data[1];
    TileCoordinate new_coordinate{x, y, center.z};
    // set tile offset with the assumption of a new center
    buildTile(new_coordinate, near_offset - offset, tile_size_m);
  }
}

void AerialMapDisplay::buildMap(TileCoordinate center_tile, double size)
{
  int zoom = center_tile.z;
  int number_of_tiles_per_dim = 1 << zoom;
  auto blocks = blocks_property_->getInt();
  for (int x = -blocks; x <= blocks; ++x) {
    for (int y = -blocks; y <= blocks; ++y) {
      int tile_x = center_tile.x + x;
      int tile_y = center_tile.y + y;
      if (tile_x < 0 || tile_x >= number_of_tiles_per_dim) {
        continue;
      }
      if (tile_y < 0 || tile_y >= number_of_tiles_per_dim) {
        continue;
      }
      buildTile({tile_x, tile_y, zoom}, Ogre::Vector2i(x, y), size);
    }
  }
}

void AerialMapDisplay::buildTile(TileCoordinate coordinate, Ogre::Vector2i offset, double size)
{
  auto tile_url = tile_url_property_->getStdString();
  const TileId tile_id{tile_url, coordinate};
  auto pending_emplace_result = pending_tiles_.emplace(tile_id, tile_client_.request(tile_id));
  rcpputils::assert_true(pending_emplace_result.second, "failed to store tile request");

  // position of each tile is set so the origin of the aerial map is the center of the middle tile
  double tx = offset.data[0] * size - size / 2;
  // while tiles follow a east-south coordinate system, the aerial map is displayed in ENU
  // thus, we invert the y translation
  double ty = -offset.data[1] * size - size / 2;
  std::stringstream ss;
  ss << tile_id;
  auto tile_emplace_result =
    tiles_.emplace(
    std::piecewise_construct,
    std::forward_as_tuple(tile_id),
    std::forward_as_tuple(
      scene_manager_, scene_node_,
      ss.str(), size, tx, ty, false));
  // hide until the tile request was completed
  tile_emplace_result.first->second.setVisible(false);
  rcpputils::assert_true(tile_emplace_result.second, "failed to store tile object");
}

// Try to get transform to fixed frame at given time, fallback to latest time within tolerance otherwise
static void get_fixed_frame_transform_fallback_to_latest(
  rviz_common::FrameManagerIface * frame_manager,
  const std::string & frame_id,
  const rclcpp::Time & t,
  const rclcpp::Duration & tolerance,
  Ogre::Vector3 & position,
  Ogre::Quaternion & orientation)
{
  position = Ogre::Vector3::ZERO;
  orientation = Ogre::Quaternion::IDENTITY;

  // identity pose
  geometry_msgs::msg::PoseStamped pose_in;
  pose_in.header.stamp = t;
  pose_in.header.frame_id = frame_id;

  auto fixed_frame = frame_manager->getFixedFrame();
  auto transformer = frame_manager->getTransformer();
  geometry_msgs::msg::PoseStamped pose_out;
  try {
    pose_out = transformer->transform(pose_in, fixed_frame);
  } catch (const rviz_common::transformation::FrameTransformerException & exception) {
    pose_in.header.stamp = rclcpp::Time(0);
    pose_out = transformer->transform(pose_in, fixed_frame);
    rclcpp::Time latest_stamp = pose_out.header.stamp;
    if (tolerance != rclcpp::Duration::from_nanoseconds(0)) {
      if (latest_stamp < (t - tolerance)) {
        throw;
      }
    }
  }

  position = rviz_common::pointMsgToOgre(pose_out.pose.position);
  orientation = rviz_common::quaternionMsgToOgre(pose_out.pose.orientation);
}

void AerialMapDisplay::update(float, float)
{
  std::unique_lock<std::mutex> lock(tiles_mutex_, std::try_to_lock);
  if (!lock.owns_lock()) {
    // if tiles are currently written to, just wait until the next update
    return;
  }

  // resolve pending tile requests, and set the received images as textures of their tiles
  for (auto it = pending_tiles_.begin(); it != pending_tiles_.end(); ) {
    try {
      if (it->second.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
        try {
          auto image = it->second.get();
          if (image.isNull()) {
            // failed to read file, e.g., from filesystem
            continue;
          }
          auto tile_to_update = tiles_.find(it->first);
          if (tile_to_update == tiles_.end()) {
            // request was cleared since it was issued
            continue;
          }
          auto & tile_object = tile_to_update->second;
          tile_object.updateData(image);
          tile_object.setVisible(true);
          // remove from pending requests
          it = pending_tiles_.erase(it);
        } catch (const tile_request_error & e) {
          // log error and abort requests
          RVIZ_COMMON_LOG_ERROR_STREAM("Tile request failed: " << e.what());
          it = pending_tiles_.end();
          setStatus(rviz_common::properties::StatusProperty::Error, TILE_REQUEST_STATUS, e.what());
          // disable requests until tile server relevant properties change
          tile_server_had_errors_ = true;
        }
      } else {
        // check next request
        ++it;
      }
    } catch (const std::future_error & e) {
      RVIZ_COMMON_LOG_DEBUG_STREAM("Tile request destroyed before resolved: " << e.what());
      it = pending_tiles_.erase(it);
    }
  }
  // if an error was just discovered
  if (tile_server_had_errors_) {
    pending_tiles_.clear();
    tiles_.clear();
  }

  if (!last_fix_) {
    return;
  }
  if (tiles_.empty()) {
    return;
  }

  auto t = context_->getFrameManager()->getTime();

  Ogre::Vector3 _ignored_translation;
  Ogre::Quaternion orientation_to_map = Ogre::Quaternion::IDENTITY;
  try {
    // get transformation of fixed frame to map, to set the aerial map orientation to be aligned with ENU
    get_fixed_frame_transform_fallback_to_latest(
      context_->getFrameManager(), MAP_FRAME, t, tf_tolerance(), _ignored_translation,
      orientation_to_map);
    setStatus(
      rviz_common::properties::StatusProperty::Ok, ORIENTATION_STATUS,
      "Map transform OK");
  } catch (const rviz_common::transformation::FrameTransformerException & e) {
    setStatus(
      rviz_common::properties::StatusProperty::Ok, TRANSFORM_STATUS, e.what());
  }

  Ogre::Vector3 sensor_translation;
  Ogre::Quaternion _ignored_orientation;
  try {
    // get transformation of sensor frame
    get_fixed_frame_transform_fallback_to_latest(
      context_->getFrameManager(),
      last_fix_->header.frame_id, t, tf_tolerance(), sensor_translation,
      _ignored_orientation);
    setStatus(rviz_common::properties::StatusProperty::Ok, TRANSFORM_STATUS, "Transform OK");
  } catch (const rviz_common::transformation::FrameTransformerException & e) {
    setStatus(
      rviz_common::properties::StatusProperty::Error, TRANSFORM_STATUS, e.what());
    return;
  }

  // "example tile", since their zoom should be uniform
  auto example_tile = tiles_.begin();
  auto center_tile_offset = tileOffset(*last_fix_, example_tile->first.coord.z);
  Ogre::Vector3 aerial_map_offset(center_tile_offset.x, -center_tile_offset.y, 0.0);

  scene_node_->setPosition(
    sensor_translation - orientation_to_map *
    (aerial_map_offset * example_tile->second.tileSize()));
  scene_node_->setOrientation(-orientation_to_map);

  // update alpha here to account for changing age
  updateAlpha(t);
}

void AerialMapDisplay::resetMap()
{
  const std::lock_guard<std::mutex> lock(tiles_mutex_);
  tiles_.clear();
  pending_tiles_.clear();
}

void AerialMapDisplay::resetTileServerError()
{
  tile_server_had_errors_ = false;
  setStatus(
    rviz_common::properties::StatusProperty::Ok, TILE_REQUEST_STATUS,
    "Last tile request OK");
}

void AerialMapDisplay::updateAlpha(const rclcpp::Time & t)
{
  auto max_alpha = alpha_property_->getFloat();
  float alpha = max_alpha;
  if (last_fix_) {
    // if message is displayed, fade it out according to configured timeout by reducing alpha
    auto timeout_s = timeout_property_->getFloat();
    if (std::abs(timeout_s) < std::numeric_limits<float>::epsilon()) {
      alpha = max_alpha;
    } else {
      auto timeout = rclcpp::Duration(std::chrono::duration<double>(timeout_s));
      auto age = t - last_fix_->header.stamp;
      // age ratio is a value from 0 to 1, where 1 means timeout is reached
      auto age_ratio = std::min(
        1.0,
        age.nanoseconds() / static_cast<double>(timeout.nanoseconds()));
      // only start fading out from ratio 0.5 to 1
      age_ratio = std::max(0.0, age_ratio - 0.5) * 2;
      alpha = max_alpha * (1.0 - age_ratio);
    }
  }
  for (auto & tile : tiles_) {
    tile.second.updateAlpha(alpha);
  }
}

rclcpp::Duration AerialMapDisplay::tf_tolerance() const {
  auto tf_tolerance_sec = tf_tolerance_property_->getFloat();
  return rclcpp::Duration(std::chrono::duration<double>(tf_tolerance_sec));
}

TileCoordinate AerialMapDisplay::centerTile() const
{
  // when calling this function internally, the tiles must already be created
  // they must also always have an uneven count, since the center tile is surrounded by a field of
  // an uneven number of sides; thus, there is a clear center tile
  assert(!tiles_.empty());
  assert((tiles_.size() % 2) == 1);
  auto it = tiles_.begin();
  std::advance(it, tiles_.size() / 2);
  return it->first.coord;
}

void AerialMapDisplay::reset()
{
  RTDClass::reset();
  resetMap();
  last_fix_.reset();
  resetTileServerError();
}

}  // namespace rviz_satellite

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(rviz_satellite::AerialMapDisplay, rviz_common::Display)
