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
#include "aerialmap_display.h"

#include <iomanip>

#include <OgreManualObject.h>
#include <OgreMaterialManager.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreTextureManager.h>
#include <OgreTechnique.h>

#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/properties/int_property.hpp"
#include "rviz_common/properties/status_property.hpp"
#include "rviz_common/properties/property.hpp"
#include "rviz_common/properties/quaternion_property.hpp"
#include "rviz_common/properties/ros_topic_property.hpp"
#include "rviz_common/properties/vector_property.hpp"
#include "rviz_common/validate_floats.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/logging.hpp"

#include "rviz_default_plugins/transformation/tf_wrapper.hpp"

#include "aerialmap_display.h"
#include "mercator.h"

namespace rviz
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

using rviz_common::properties::FloatProperty;
using rviz_common::properties::IntProperty;
using rviz_common::properties::Property;
using rviz_common::properties::RosTopicProperty;
using rviz_common::properties::StringProperty;
using rviz_common::properties::StatusProperty;

std::string const AerialMapDisplay::MAP_FRAME = "map";

AerialMapDisplay::AerialMapDisplay() : Display()
{

  topic_property_ =
      new RosTopicProperty("Topic", "", "",
                           "sensor_msgs::NavSatFix topic to subscribe to.", this, SLOT(updateTopic()));

  alpha_property_ =
      new FloatProperty("Alpha", 0.7, "Amount of transparency to apply to the map.", this, SLOT(updateAlpha()));
  alpha_property_->setMin(0);
  alpha_property_->setMax(1);
  alpha_property_->setShouldBeSaved(true);
  alpha_ = alpha_property_->getValue().toFloat();

  draw_under_property_ = new Property("Draw Behind", false,
                                      "Rendering option, controls whether or not the map is always"
                                      " drawn behind everything else.",
                                      this, SLOT(updateDrawUnder()));
  draw_under_property_->setShouldBeSaved(true);
  draw_under_ = draw_under_property_->getValue().toBool();

  // properties for map
  tile_url_property_ =
      new StringProperty("Object URI", "", "URL from which to retrieve map tiles.", this, SLOT(updateTileUrl()));
  tile_url_property_->setShouldBeSaved(true);
  tile_url_ = tile_url_property_->getStdString();

  QString const zoom_desc = QString::fromStdString("Zoom level (0 - " + std::to_string(MAX_ZOOM) + ")");
  zoom_property_ = new IntProperty("Zoom", 16, zoom_desc, this, SLOT(updateZoom()));
  zoom_property_->setMin(0);
  zoom_property_->setMax(MAX_ZOOM);
  zoom_property_->setShouldBeSaved(true);
  zoom_ = zoom_property_->getInt();

  QString const blocks_desc = QString::fromStdString("Adjacent blocks (0 - " + std::to_string(MAX_BLOCKS) + ")");
  blocks_property_ = new IntProperty("Blocks", 3, blocks_desc, this, SLOT(updateBlocks()));
  blocks_property_->setMin(0);
  blocks_property_->setMax(MAX_BLOCKS);
  blocks_property_->setShouldBeSaved(true);
  blocks_ = blocks_property_->getInt();
}

AerialMapDisplay::~AerialMapDisplay()
{
  unsubscribe();
  clearAll();
}

void AerialMapDisplay::onInitialize()
{
  Display::onInitialize();
  rviz_ros_node_ = context_->getRosNodeAbstraction();
  topic_property_->initialize(rviz_ros_node_);
}

void AerialMapDisplay::onEnable()
{
  // this should be set in onInitialize (when the context_ becomes available) - but lets save the additional function
  auto tf_wrapper = std::dynamic_pointer_cast<rviz_default_plugins::transformation::TFWrapper>(
    context_->getFrameManager()->getConnector().lock());

  if (tf_wrapper) tf_buffer_ = tf_wrapper->getBuffer();

  createTileObjects();
  subscribe();
}

void AerialMapDisplay::onDisable()
{
  unsubscribe();
  clearAll();
}

void AerialMapDisplay::subscribe()
{
  if (!isEnabled())
  {
    return;
  }

  if (!topic_property_->getTopic().isEmpty())
  {
    try {
      navsat_fix_sub_ =
        rviz_ros_node_.lock()->get_raw_node()->
        template create_subscription<sensor_msgs::msg::NavSatFix>(
        topic_property_->getTopicStd(), update_profile_,
        std::bind(&AerialMapDisplay::navFixCallback, this, std::placeholders::_1));
      setStatus(rviz_common::properties::StatusProperty::Ok, "Topic", "OK");
    } catch (rclcpp::exceptions::InvalidTopicNameError & e) {
      setStatus(
        rviz_common::properties::StatusProperty::Error, "Topic",
        QString("Error subscribing: ") + e.what());
    }
  }
}

void AerialMapDisplay::unsubscribe()
{
  navsat_fix_sub_.reset();
}

void AerialMapDisplay::updateAlpha()
{
  // if draw_under_ texture property changed, we need to
  //  - repaint textures
  // we don't need to
  //  - query textures
  //  - re-create tile grid geometry
  //  - update the center tile
  //  - update transforms

  auto const alpha = alpha_property_->getFloat();
  if (alpha == alpha_)
  {
    return;
  }

  alpha_ = alpha;

  if (!isEnabled())
  {
    return;
  }

  triggerSceneAssembly();
}

void AerialMapDisplay::updateDrawUnder()
{
  // if draw_under_ texture property changed, we need to
  //  - repaint textures
  // we don't need to
  //  - query textures
  //  - re-create tile grid geometry
  //  - update the center tile
  //  - update transforms

  auto const draw_under = draw_under_property_->getValue().toBool();
  if (draw_under == draw_under_)
  {
    return;
  }

  draw_under_ = draw_under;

  if (!isEnabled())
  {
    return;
  }

  triggerSceneAssembly();
}

void AerialMapDisplay::updateTileUrl()
{
  // if the tile url changed, we need to
  //  - query textures
  //  - repaint textures
  // we don't need to
  //  - re-create tile grid geometry
  //  - update the center tile
  //  - update transforms

  auto const tile_url = tile_url_property_->getStdString();
  if (tile_url == tile_url_)
  {
    return;
  }

  tile_url_ = tile_url;

  if (!isEnabled())
  {
    return;
  }

  requestTileTextures();
}

void AerialMapDisplay::updateZoom()
{
  // if the zoom changed, we need to
  //  - re-create tile grid geometry
  //  - update the center tile
  //  - query textures
  //  - repaint textures
  //  - update transforms

  auto const zoom = zoom_property_->getInt();
  if (zoom == zoom_)
  {
    return;
  }

  zoom_ = zoom;

  if (!isEnabled())
  {
    return;
  }

  createTileObjects();

  if (ref_fix_)
  {
    updateCenterTile(ref_fix_, true);
  }
}

void AerialMapDisplay::updateBlocks()
{
  // if the number of blocks changed, we need to
  //  - re-create tile grid geometry
  //  - query textures
  //  - repaint textures
  // we don't need to
  //  - update the center tile
  //  - update transforms

  auto const blocks = blocks_property_->getInt();
  if (blocks == blocks_)
  {
    return;
  }

  blocks_ = blocks;

  if (!isEnabled())
  {
    return;
  }

  createTileObjects();
  requestTileTextures();
}

void AerialMapDisplay::updateTopic()
{
  // if the NavSat topic changes, we reset everything

  if (!isEnabled())
  {
    return;
  }

  unsubscribe();
  clearAll();
  createTileObjects();
  subscribe();
}

void AerialMapDisplay::clearAll()
{
  ref_fix_ = nullptr;
  center_tile_ = boost::none;
  destroyTileObjects();

  setStatus(StatusProperty::Warn, "Message", "No NavSatFix message received yet");
}

void AerialMapDisplay::destroyTileObjects()
{
  for (MapObject& obj : objects_)
  {
    // destroy object
    scene_node_->detachObject(obj.object);
    scene_manager_->destroyManualObject(obj.object);

    // destroy material
    if (obj.material)
    {
      Ogre::MaterialManager::getSingleton().remove(obj.material->getName());
    }
  }
  objects_.clear();
}

void AerialMapDisplay::createTileObjects()
{
  if (not objects_.empty())
  {
    destroyTileObjects();
  }

  for (int block = 0; block < (2 * blocks_ + 1) * (2 * blocks_ + 1); ++block)
  {
    // generate an unique name
    static size_t count = 0;
    std::string const name_suffix = std::to_string(count);
    ++count;

    // one material per texture
    Ogre::MaterialPtr material = Ogre::MaterialManager::getSingleton().create(
        "satellite_material_" + name_suffix, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
    material->setReceiveShadows(false);
    material->getTechnique(0)->setLightingEnabled(false);
    material->setDepthBias(-16.0f, 0.0f);
    material->setCullingMode(Ogre::CULL_NONE);
    material->setDepthWriteEnabled(false);

    // create texture and initialize it
    Ogre::TextureUnitState* tex_unit = material->getTechnique(0)->getPass(0)->createTextureUnitState();
    tex_unit->setTextureFiltering(Ogre::TFO_BILINEAR);

    // create an object
    Ogre::ManualObject* obj = scene_manager_->createManualObject("satellite_object_" + name_suffix);
    obj->setVisible(false);
    scene_node_->attachObject(obj);

    assert(material);
    objects_.emplace_back(obj, material);
  }
}

void AerialMapDisplay::update(float, float)
{
  if (not ref_fix_ or not center_tile_)
  {
    return;
  }

  // update tiles, if necessary
  assembleScene();
  // transform scene object into fixed frame
  transformMapTileToFixedFrame();
}

void AerialMapDisplay::navFixCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
  updateCenterTile(msg, false);
  navframe = msg->header.frame_id;

  setStatus(StatusProperty::Ok, "Message", "NavSatFix message received");
}

void AerialMapDisplay::updateCenterTile(const sensor_msgs::msg::NavSatFix::SharedPtr msg, bool new_zoom_)
{
  if (!isEnabled())
  {
    return;
  }

  // check if update is necessary
  TileCoordinate const tile_coordinates = fromWGSCoordinate<int>({ msg->latitude, msg->longitude }, zoom_);
  TileId const new_center_tile_id{ tile_url_, tile_coordinates, zoom_ };
  bool const center_tile_changed = (!center_tile_ || new_zoom_);// || !(new_center_tile_id == *center_tile_));

  if (not center_tile_changed)
  {
    // TODO: Maybe we should update the transform here even if the center tile did not change?
    // The localization might have been updated.
    transformTileToMapFrame();
    return;
  }

  RVIZ_COMMON_LOG_INFO("Updating center tile");

  center_tile_ = new_center_tile_id;
  ref_fix_ = msg;

  requestTileTextures();
  transformTileToMapFrame();
}

void AerialMapDisplay::requestTileTextures()
{
  if (!isEnabled())
  {
    return;
  }

  if (tile_url_.empty())
  {
    setStatus(StatusProperty::Error, "TileRequest", "Tile URL is not set");
    return;
  }

  if (not center_tile_)
  {
    setStatus(StatusProperty::Error, "Message", "No NavSatFix received yet");
    return;
  }

  try
  {
    RVIZ_COMMON_LOG_INFO("Requesting new tile images from the server");
    tile_cache_.request({ *center_tile_, blocks_ });
    triggerSceneAssembly();
  }
  catch (std::exception const& e)
  {
    setStatus(StatusProperty::Error, "TileRequest", QString(e.what()));
    return;
  }
}

void AerialMapDisplay::checkRequestErrorRate()
{
  // the following error rate thresholds are randomly chosen
  float const error_rate = tile_cache_.getTileServerErrorRate(tile_url_);
  if (error_rate > 0.95)
  {
    setStatus(StatusProperty::Level::Error, "TileRequest", "Few or no tiles received");
  }
  else if (error_rate > 0.3)
  {
    setStatus(StatusProperty::Level::Warn, "TileRequest",
              "Not all requested tiles have been received. Possibly the server is throttling?");
  }
  else
  {
    setStatus(StatusProperty::Level::Ok, "TileRequest", "OK");
  }
}

void AerialMapDisplay::triggerSceneAssembly()
{
  RVIZ_COMMON_LOG_INFO("Starting to repaint all tiles");
  dirty_ = true;
}

void AerialMapDisplay::assembleScene()
{
  // TODO: split this function into pieces, and only call the pieces when needed
  // E.g. into: grid geometry, tile geometry, material/texture (only this is asynchronous and depends on the incoming
  // tiles from the cache/server)

  if (!isEnabled() || !dirty_ || !center_tile_)
  {
    return;
  }

  if (objects_.empty())
  {
    // FIXME - Not supported yet
    // RVIZ_COMMON_LOG_ERROR_THROTTLE(std::chrono::seconds(5), "No objects to draw on, call createTileObjects() first!");
    return;
  }

  dirty_ = false;

  Area area(*center_tile_, blocks_);

  TileCacheGuard guard(tile_cache_);

  bool all_tiles_loaded = true;

  auto it = objects_.begin();
  for (int xx = area.left_top.x; xx <= area.right_bottom.x; ++xx)
  {
    for (int yy = area.left_top.y; yy <= area.right_bottom.y; ++yy)
    {
      auto obj = it->object;
      auto& material = it->material;
      assert(material);
      ++it;

      TileId const to_find{ center_tile_->tile_server, { xx, yy }, center_tile_->zoom };

      OgreTile const* tile = tile_cache_.ready(to_find);
      if (!tile)
      {
        // don't show tiles with old textures
        obj->setVisible(false);
        all_tiles_loaded = false;
        continue;
      }

      obj->setVisible(true);

      // update texture
      Ogre::TextureUnitState* tex_unit = material->getTechnique(0)->getPass(0)->getTextureUnitState(0);
      tex_unit->setTextureName(tile->texture->getName());

      // configure depth & alpha properties
      if (alpha_ >= 0.9998)
      {
        material->setDepthWriteEnabled(!draw_under_);
        material->setSceneBlending(Ogre::SBT_REPLACE);
      }
      else
      {
        material->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
        material->setDepthWriteEnabled(false);
      }

      if (draw_under_)
      {
        // render under everything else
        obj->setRenderQueueGroup(Ogre::RENDER_QUEUE_3);
      }
      else
      {
        obj->setRenderQueueGroup(Ogre::RENDER_QUEUE_MAIN);
      }

      tex_unit->setAlphaOperation(Ogre::LBX_SOURCE1, Ogre::LBS_MANUAL, Ogre::LBS_CURRENT, alpha_);

      // tile width/ height in meter
      double const tile_w_h_m = getTileWH(ref_fix_->latitude, zoom_);

      // Note: In the following we will do two things:
      //
      // * We flip the position's y coordinate.
      // * We flip the texture's v coordinate.
      //
      // For more explanation see the function transformAerialMap()

      // The center tile has the coordinates left-bot = (0,0) and right-top = (1,1) in the AerialMap frame.
      double const x = (xx - center_tile_->coord.x) * tile_w_h_m;
      // flip the y coordinate because we need to flip the tiles to align the tile's frame with the ENU "map" frame
      double const y = -(yy - center_tile_->coord.y) * tile_w_h_m;

      // create a quad for this tile
      // note: We have to recreate the vertices and cannot reuse the old vertices: tile_w_h_m depends on the latitude
      obj->clear();

      obj->begin(material->getName(), Ogre::RenderOperation::OT_TRIANGLE_LIST);

      // We assign the Ogre texture coordinates in a way so that we flip the
      // texture along the v coordinate. For example, we assign the bottom left
      //
      // Note that the Ogre texture coordinate system is: (0,0) = top left of the loaded image and (1,1) = bottom right
      // of the loaded image

      // bottom left
      obj->position(x, y, 0.0f);
      obj->textureCoord(0.0f, 0.0f);
      obj->normal(0.0f, 0.0f, 1.0f);

      // top right
      obj->position(x + tile_w_h_m, y + tile_w_h_m, 0.0f);
      obj->textureCoord(1.0f, 1.0f);
      obj->normal(0.0f, 0.0f, 1.0f);

      // top left
      obj->position(x, y + tile_w_h_m, 0.0f);
      obj->textureCoord(0.0f, 1.0f);
      obj->normal(0.0f, 0.0f, 1.0f);

      // bottom left
      obj->position(x, y, 0.0f);
      obj->textureCoord(0.0f, 0.0f);
      obj->normal(0.0f, 0.0f, 1.0f);

      // bottom right
      obj->position(x + tile_w_h_m, y, 0.0f);
      obj->textureCoord(1.0f, 0.0f);
      obj->normal(0.0f, 0.0f, 1.0f);

      // top right
      obj->position(x + tile_w_h_m, y + tile_w_h_m, 0.0f);
      obj->textureCoord(1.0f, 1.0f);
      obj->normal(0.0f, 0.0f, 1.0f);

      obj->end();
    }
  }

  // since not all tiles were loaded yet, this function has to be called again
  if (!all_tiles_loaded)
  {
    dirty_ = true;
  }
  else
  {
    RVIZ_COMMON_LOG_INFO("Finished assembling all tiles");
  }

  tile_cache_.purge({ *center_tile_, blocks_ });

  checkRequestErrorRate();
}

void AerialMapDisplay::transformTileToMapFrame()
{
  if (not ref_fix_ or not center_tile_)
  {
    // FIXME - Not supported yet
    // RVIZ_COMMON_LOG_FATAL_THROTTLE(std::chrono::seconds(2), "ref_fix_ not set, can't create transforms");
    return;
  }

  // We will use three frames in this function:
  //
  // * The frame from the NavSatFix message. It is rigidly attached to the robot.
  // * The ENU world frame "map".
  // * The frame of the tiles. We assume that the tiles are in a frame where x points eastwards and y southwards (ENU).
  // This
  //   frame is used by OSM and Google Maps, see https://en.wikipedia.org/wiki/Web_Mercator_projection and
  //   https://developers.google.com/maps/documentation/javascript/coordinates.

  // translation of NavSatFix frame w.r.t. the map frame
  // NOTE: due to ENU convention, orientation is not needed, the tiles are rigidly attached to ENU
  Ogre::Vector3 t_navsat_map;

  try
  {
    // Use a real TfBuffer for looking up this transform. The FrameManager only supplies transform to/from the
    // currently selected RViz fixed-frame, which is of no help here.
    auto const tf_navsat_map =
        tf_buffer_->lookupTransform(navframe, ref_fix_->header.frame_id, ref_fix_->header.stamp, tf2::durationFromSec(0.1));
    auto const tf_pos = tf_navsat_map.transform.translation;
    t_navsat_map = Ogre::Vector3(tf_pos.x, tf_pos.y, tf_pos.z);
  }
  catch (tf2::TransformException const& ex)
  {
    setStatus(StatusProperty::Error, "Transform", QString::fromStdString(ex.what()));
    return;
  }

  // FIXME: note the <double> template! this is different from center_tile_.coord, otherwise we could just use that
  // since center_tile_ and ref_fix_ are in sync
  auto const ref_fix_tile_coords = fromWGSCoordinate<double>({ ref_fix_->latitude, ref_fix_->longitude }, zoom_);

  // In assembleScene() we shift the AerialMap so that the center tile's left-bottom corner has the coordinate (0,0).
  // Therefore we can calculate the NavSatFix coordinate (in the AerialMap frame) by just looking at the fractional part
  // of the coordinate. That is we calculate the offset from the left bottom corner of the center tile.
  auto const center_tile_offset_x = ref_fix_tile_coords.x - std::floor(ref_fix_tile_coords.x);
  // In assembleScene() the tiles are created so that the texture is flipped along the y coordinate. Since we want to
  // calculate the positions of the center tile, we also need to flip the texture's v coordinate here.
  auto const center_tile_offset_y = 1 - (ref_fix_tile_coords.y - std::floor(ref_fix_tile_coords.y));

  double const tile_w_h_m = getTileWH(ref_fix_->latitude, zoom_);
  //RVIZ_COMMON_LOG_INFO_STREAM("Tile resolution is " << std::setprecision(1) << std::fixed << tile_w_h_m << "m");

  // translation of the center-tile w.r.t. the NavSatFix frame
  auto const t_centertile_navsat =
      Ogre::Vector3(center_tile_offset_x * tile_w_h_m, center_tile_offset_y * tile_w_h_m, 0);

  t_centertile_map_ = t_navsat_map - t_centertile_navsat;
}

void AerialMapDisplay::transformMapTileToFixedFrame()
{
  // orientation of the fixed-frame w.r.t. the map-frame
  Ogre::Quaternion o_fixed_map;
  // translation of the fixed-frame w.r.t. the map-frame
  Ogre::Vector3 t_fixed_map;

  // get transform between map-frame and fixed-frame from the FrameManager
  if (context_->getFrameManager()->getTransform(MAP_FRAME, t_fixed_map, o_fixed_map))
  {
    setStatus(::rviz::StatusProperty::Ok, "Transform", "Transform OK");

    // the translation of the tile w.r.t. the fixed-frame
    auto const t_centertile_fixed = t_fixed_map + o_fixed_map * t_centertile_map_;

    scene_node_->setPosition(t_centertile_fixed);
    scene_node_->setOrientation(o_fixed_map);
  }
  else
  {
    // display error
    std::string error;
    if (context_->getFrameManager()->transformHasProblems(MAP_FRAME, error))
    {
      setStatus(::rviz::StatusProperty::Error, "Transform", QString::fromStdString(error));
    }
    else
    {
      setStatus(::rviz::StatusProperty::Error, "Transform",
                QString::fromStdString("Could not transform from [" + MAP_FRAME + "] to Fixed Frame [" +
                                       fixed_frame_.toStdString() + "] for an unknown reason"));
    }
  }
}

void AerialMapDisplay::reset()
{
  Display::reset();
  // unsubscribe, clear, resubscribe
  updateTopic();
}

}  // namespace rviz

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(rviz::AerialMapDisplay, rviz_common::Display)
