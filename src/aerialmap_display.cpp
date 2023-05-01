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

#include <OGRE/OgreManualObject.h>
#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreTextureManager.h>
#include <OGRE/OgreTechnique.h>

#include <GeographicLib/UTMUPS.hpp>

#include "rviz/display_context.h"
#include "rviz/frame_manager.h"
#include "rviz/properties/enum_property.h"
#include "rviz/properties/float_property.h"
#include "rviz/properties/int_property.h"
#include "rviz/properties/property.h"
#include "rviz/properties/ros_topic_property.h"
#include "rviz/properties/string_property.h"
#include "rviz/properties/tf_frame_property.h"

#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "aerialmap_display.h"
#include "mercator.h"
#include "position_reference_property.h"

#include <regex>
#include <unordered_map>


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

std::unordered_map<MapTransformType, QString> mapTransformTypeStrings =
{
  {MapTransformType::VIA_MAP_FRAME, "NavSatFix Messages and Map Frame"},
  {MapTransformType::VIA_UTM_FRAME, "Explicit UTM Frame"},
};

AerialMapDisplay::AerialMapDisplay() : Display()
{
  center_tile_pose_.pose.orientation.w = 1;

  topic_property_ =
      new RosTopicProperty("Topic", "", QString::fromStdString(ros::message_traits::datatype<sensor_msgs::NavSatFix>()),
                           "sensor_msgs::NavSatFix topic to subscribe to.", this, SLOT(updateTopic()));

  map_transform_type_property_ =
      new EnumProperty("Map transform type", mapTransformTypeStrings[MapTransformType::VIA_MAP_FRAME],
                       "Whether to transform tiles via map frame and fix messages or via UTM frame",
                       this, SLOT(updateMapTransformType()));
  map_transform_type_property_->addOption(mapTransformTypeStrings[MapTransformType::VIA_MAP_FRAME],
                                          static_cast<int>(MapTransformType::VIA_MAP_FRAME));
  map_transform_type_property_->addOption(mapTransformTypeStrings[MapTransformType::VIA_UTM_FRAME],
                                          static_cast<int>(MapTransformType::VIA_UTM_FRAME));
  map_transform_type_property_->setShouldBeSaved(true);
  map_transform_type_ = static_cast<MapTransformType>(map_transform_type_property_->getOptionInt());
  
  map_frame_property_ = new TfFrameProperty(
      "Map Frame", "map", "Frame ID of the map.", this, nullptr, false, SLOT(updateMapFrame()));
  map_frame_property_->setShouldBeSaved(true);
  map_frame_ = map_frame_property_->getFrameStd();
  
  utm_frame_property_ = new TfFrameProperty(
      "UTM Frame", "utm", "Frame ID of the UTM frame.", this, nullptr, false, SLOT(updateUtmFrame()));
  utm_frame_property_->setShouldBeSaved(true);
  utm_frame_ = utm_frame_property_->getFrameStd();
  
  utm_zone_property_ =
      new IntProperty("UTM Zone", GeographicLib::UTMUPS::STANDARD, "UTM zone (-1 means autodetect).",
                      this, SLOT(updateUtmZone()));
  utm_zone_property_->setMin(GeographicLib::UTMUPS::STANDARD);
  utm_zone_property_->setMax(GeographicLib::UTMUPS::MAXZONE);
  utm_zone_property_->setShouldBeSaved(true);
  utm_zone_ = utm_zone_property_->getInt();

  xy_reference_property_ =
      new PositionReferenceProperty("XY Reference", PositionReferenceProperty::FIX_MSG_STRING,
                                    "How to determine XY coordinates that define the displayed tiles.",
                                    this, nullptr, SLOT(updateXYReference()));
  xy_reference_property_->setShouldBeSaved(true);
  xy_reference_type_ = PositionReferenceType::NAV_SAT_FIX_MESSAGE;
  
  z_reference_property_ =
      new PositionReferenceProperty("Z Reference", PositionReferenceProperty::FIX_MSG_STRING,
                                    "How to determine height of the displayed tiles.",
                                    this, nullptr, SLOT(updateZReference()));
  z_reference_property_->setShouldBeSaved(true);
  z_reference_type_ = PositionReferenceType::NAV_SAT_FIX_MESSAGE;
  
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
  
  z_offset_property_ =
      new FloatProperty("Z Offset", 0.0, "Offset in Z direction (in meters).", this, SLOT(updateZOffset()));
  z_offset_property_->setShouldBeSaved(true);
  z_offset_ = z_offset_property_->getValue().toFloat();
  
  tf_reference_update_duration_ = {1, 0};
  tf_reference_update_timer_ = threaded_nh_.createTimer(
      tf_reference_update_duration_, &AerialMapDisplay::tfReferencePeriodicUpdate, this);
}

AerialMapDisplay::~AerialMapDisplay()
{
  unsubscribe();
  clearAll();
}

void AerialMapDisplay::onInitialize()
{
  tf_buffer_ = context_->getFrameManager()->getTF2BufferPtr();
  map_frame_property_->setFrameManager(context_->getFrameManager());
  utm_frame_property_->setFrameManager(context_->getFrameManager());
  xy_reference_property_->setFrameManager(context_->getFrameManager());
  z_reference_property_->setFrameManager(context_->getFrameManager());
  updateMapTransformType();
}

void AerialMapDisplay::onEnable()
{
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
    try
    {
      ROS_INFO("Subscribing to %s", topic_property_->getTopicStd().c_str());
      navsat_fix_sub_ =
          update_nh_.subscribe(topic_property_->getTopicStd(), 1, &AerialMapDisplay::navFixCallback, this);

      setStatus(StatusProperty::Ok, "Topic", "OK");
    }
    catch (ros::Exception& e)
    {
      setStatus(StatusProperty::Error, "Topic", QString("Error subscribing: ") + e.what());
    }
  }
}

void AerialMapDisplay::unsubscribe()
{
  navsat_fix_sub_.shutdown();
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

  // Check for valid url: https://stackoverflow.com/a/38608262
  if (!std::regex_match(tile_url, std::regex(R"(^(https?:\/\/).*)")))
  {
    ROS_ERROR("Invalid Object URI: %s", tile_url.c_str());
    // Still setting the url since keeping the old is probably unexpected
  }
  else if (!std::regex_match(tile_url, std::regex(R"(.*\{([xyz]|lat|lon)\}.*)")))
  {
    ROS_ERROR("No coordinates ({lat}, {lon} or {x}, {y}, {z}) found in URI: %s", tile_url.c_str());
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
    updateCenterTile(ref_fix_);
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

void AerialMapDisplay::updateMapTransformType()
{
  // if the map transform type changed, we need to
  //  - enable/disable map/utm frame inputs
  //  - update XY and Z reference settings to only use values valid for the given mode
  //  - query textures
  //  - repaint textures
  //  - reset and update the center tile
  //  - update transforms
  // we don't need to
  //  - re-create tile grid geometry

  auto const map_transform_type = static_cast<MapTransformType>(map_transform_type_property_->getOptionInt());

  map_transform_type_ = map_transform_type;
  
  switch (map_transform_type_)
  {
    case MapTransformType::VIA_MAP_FRAME:
      utm_frame_property_->hide();
      utm_zone_property_->hide();
      map_frame_property_->show();
      break;
    case MapTransformType::VIA_UTM_FRAME:
      utm_frame_property_->show();
      utm_zone_property_->show();
      map_frame_property_->hide();
      break;
  }

  updateXYReference();
  updateZReference();
  
  if (ref_fix_)
  {
    updateCenterTile(ref_fix_);
  }
}

void AerialMapDisplay::updateMapFrame()
{
  // if the map frame changed, we need to
  //  - query textures
  //  - repaint textures
  //  - reset and update the center tile
  //  - update transforms
  // we don't need to
  //  - re-create tile grid geometry

  auto const map_frame = map_frame_property_->getFrameStd();
  if (map_frame == map_frame_)
  {
    return;
  }

  map_frame_ = map_frame;

  if (!isEnabled())
  {
    return;
  }

  if (ref_fix_)
  {
    updateCenterTile(ref_fix_);
  }
}

void AerialMapDisplay::updateUtmFrame()
{
  // if the UTM frame changed, we need to
  //  - query textures
  //  - repaint textures
  //  - reset and update the center tile
  //  - update transforms
  // we don't need to
  //  - re-create tile grid geometry

  auto const utm_frame = utm_frame_property_->getFrameStd();
  if (utm_frame == utm_frame_)
  {
    return;
  }

  utm_frame_ = utm_frame;

  if (!isEnabled())
  {
    return;
  }

  if (ref_fix_)
  {
    updateCenterTile(ref_fix_);
  }
}

void AerialMapDisplay::updateUtmZone()
{
  // if the UTM zone changed, we need to
  //  - query textures
  //  - repaint textures
  //  - reset and update the center tile
  //  - update transforms
  // we don't need to
  //  - re-create tile grid geometry

  auto const utm_zone = utm_zone_property_->getInt();
  if (utm_zone == utm_zone_)
  {
    return;
  }

  utm_zone_ = utm_zone;

  if (!isEnabled())
  {
    return;
  }

  if (ref_fix_)
  {
    updateCenterTile(ref_fix_);
  }
}

void AerialMapDisplay::updateXYReference()
{
  // if the XY reference changed, we need to
  //  - query textures
  //  - repaint textures
  //  - reset and update the center tile
  //  - update transforms
  // we don't need to
  //  - re-create tile grid geometry

  const auto old_reference_type = xy_reference_type_;
  const auto old_reference_frame = xy_reference_frame_;

  switch (map_transform_type_)
  {
    case MapTransformType::VIA_MAP_FRAME:
      xy_reference_type_ = PositionReferenceType::NAV_SAT_FIX_MESSAGE;
      xy_reference_frame_ = "";
      xy_reference_property_->hide();
      break;
    case MapTransformType::VIA_UTM_FRAME:
      xy_reference_type_ = xy_reference_property_->getPositionReferenceType();
      xy_reference_frame_ = xy_reference_property_->getFrameStd();
      xy_reference_property_->show();
      if (xy_reference_type_ == PositionReferenceType::TF_FRAME && xy_reference_frame_ == utm_frame_)
      {
        ROS_WARN_THROTTLE(2.0, "Setting UTM frame '%s' as XY reference is invalid, as the computed easting and "
                               "northing of zero is out of bounds. Select a different frame.", utm_frame_.c_str());
      }
      break;
  }
  
  if (!isEnabled() || (old_reference_type == xy_reference_type_ && old_reference_frame == xy_reference_frame_))
  {
    return;
  }
  
  if (xy_reference_type_ != PositionReferenceType::TF_FRAME)
  {
    deleteStatus("UTM");
    deleteStatus("XY Reference Transform");
    deleteStatus("XY reference UTM conversion");
  }
  
  if (ref_fix_)
  {
    updateCenterTile(ref_fix_);
  }
}

void AerialMapDisplay::updateZReference()
{
  // if the Z reference changed, we need to
  //  - update transforms
  // we don't need to
  //  - re-create tile grid geometry
  //  - query textures
  //  - repaint textures
  //  - reset and update the center tile

  const auto old_reference_type = z_reference_type_;
  const auto old_reference_frame = z_reference_frame_;

  switch (map_transform_type_)
  {
    case MapTransformType::VIA_MAP_FRAME:
      z_reference_type_ = PositionReferenceType::NAV_SAT_FIX_MESSAGE;
      z_reference_frame_ = "";
      z_reference_property_->hide();
      break;
    case MapTransformType::VIA_UTM_FRAME:
      z_reference_type_ = z_reference_property_->getPositionReferenceType();
      z_reference_frame_ = z_reference_property_->getFrameStd();
      z_reference_property_->show();
      break;
  }
  
  if (!isEnabled() || (old_reference_type == z_reference_type_ && old_reference_frame == z_reference_frame_))
  {
    return;
  }
  
  if (z_reference_type_ != PositionReferenceType::TF_FRAME)
  {
    deleteStatus("Z Reference Transform");
  }
  
  transformTileToReferenceFrame();
}

void AerialMapDisplay::updateZOffset()
{
  // if the Z offset changed, we don't need to do anything as the value is directly read in each update() call
  
  z_offset_ = z_offset_property_->getFloat();
}

void AerialMapDisplay::clearAll()
{
  ref_fix_ = nullptr;
  center_tile_ = boost::none;
  ref_coords_ = boost::none;
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
    if (!obj.material.isNull())
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

    assert(!material.isNull());
    objects_.emplace_back(obj, material);
  }
}

void AerialMapDisplay::update(float, float)
{
  if (!ref_fix_ or !center_tile_)
  {
    return;
  }

  // update tiles, if necessary
  assembleScene();
  // transform scene object into fixed frame
  transformMapTileToFixedFrame();
}

void AerialMapDisplay::navFixCallback(sensor_msgs::NavSatFixConstPtr const& msg)
{
  // reset the periodic update timer as we got an update right now
  tf_reference_update_timer_.setPeriod(tf_reference_update_duration_);

  updateCenterTile(msg);

  setStatus(StatusProperty::Ok, "Message", "NavSatFix message received");
}

bool AerialMapDisplay::updateCenterTile(sensor_msgs::NavSatFixConstPtr const& msg)
{
  if (!isEnabled())
  {
    return false;
  }

  WGSCoordinate reference_wgs{};

  // find the WGS lat/lon of the XY reference point
  switch (xy_reference_type_)
  {
    case PositionReferenceType::NAV_SAT_FIX_MESSAGE: {
      reference_wgs = { msg->latitude, msg->longitude };
      break;
    }
    case PositionReferenceType::TF_FRAME: {
      try
      {
        auto const tf_reference_utm = tf_buffer_->lookupTransform(utm_frame_, xy_reference_frame_, ros::Time(0));
        setStatus(::rviz::StatusProperty::Ok, "XY Reference Transform", "Transform OK.");

        try
        {
          // If UTM zone is set to be autodetected and detection has not yet been performed, do it now
          if (utm_zone_ < GeographicLib::UTMUPS::MINZONE)
          {
            int zone;
            bool northp;
            double e, n;
            GeographicLib::UTMUPS::Forward(msg->latitude, msg->longitude, zone, northp, e, n, utm_zone_);
            utm_zone_property_->setValue(zone);
            ROS_INFO("UTM zone has been automatically determined from NavSatFix message to %s.",
                     GeographicLib::UTMUPS::EncodeZone(zone, northp).c_str());
          }

          const auto& utm_coords = tf_reference_utm.transform.translation;
          GeographicLib::UTMUPS::Reverse(utm_zone_, msg->latitude >= 0, utm_coords.x, utm_coords.y, reference_wgs.lat,
                                         reference_wgs.lon, true);
          setStatus(::rviz::StatusProperty::Ok, "XY reference UTM conversion", "UTM conversion OK.");
        }
        catch (GeographicLib::GeographicErr& err)
        {
          setStatus(::rviz::StatusProperty::Error, "XY reference UTM conversion", QString::fromStdString(err.what()));
          ROS_ERROR_THROTTLE(2.0, "%s", err.what());
          return false;
        }
      }
      catch (tf2::TransformException const& ex)
      {
        setStatus(::rviz::StatusProperty::Error, "XY Reference Transform", QString::fromStdString(ex.what()));
        ROS_ERROR_THROTTLE(2.0, "%s", ex.what());
        return false;
      }
      break;
    }
  }
 
  // check if update is necessary
  TileCoordinate const tile_coordinates = fromWGSCoordinate<int>(reference_wgs, zoom_);
  TileId const new_center_tile_id{ tile_url_, tile_coordinates, zoom_ };
  bool const center_tile_changed = (!center_tile_ || !(new_center_tile_id == *center_tile_));

  if (not center_tile_changed)
  {
    // TODO: Maybe we should update the transform here even if the center tile did not change?
    // The localization might have been updated.
    return false;
  }

  ROS_DEBUG_NAMED("rviz_satellite", "Updating center tile to (%i, %i)", tile_coordinates.x, tile_coordinates.y);

  center_tile_ = new_center_tile_id;
  ref_coords_ = reference_wgs;
  ref_fix_ = msg;

  requestTileTextures();
  transformTileToReferenceFrame();
  return true;
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
    ROS_DEBUG_NAMED("rviz_satellite", "Requesting new tile images from the server");
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
  ROS_DEBUG_NAMED("rviz_satellite", "Starting to repaint all tiles");
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
    ROS_ERROR_THROTTLE_NAMED(5, "rviz_satellite", "No objects to draw on, call createTileObjects() first!");
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
      assert(!material.isNull());
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
      double const tile_w_h_m = getTileWH(ref_coords_->lat, zoom_);

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
    ROS_DEBUG_NAMED("rviz_satellite", "Finished assembling all tiles");
  }

  tile_cache_.purge({ *center_tile_, blocks_ });

  checkRequestErrorRate();
}

void AerialMapDisplay::transformTileToReferenceFrame()
{
  switch (map_transform_type_)
  {
    case MapTransformType::VIA_MAP_FRAME:
      transformTileToMapFrame();
      break;
    case MapTransformType::VIA_UTM_FRAME:
      transformTileToUtmFrame();
      break;
  }
}

void AerialMapDisplay::transformTileToMapFrame()
{
  if (!ref_fix_ or !center_tile_)
  {
    ROS_FATAL_THROTTLE_NAMED(2, "rviz_satellite", "ref_fix_ not set, can't create transforms");
    return;
  }

  // We will use three frames in this function:
  //
  // * The frame from the NavSatFix message. It is rigidly attached to the robot.
  // * The ENU world frame "map_frame_".
  // * The frame of the tiles. We assume that the tiles are in a frame where x points eastwards and y southwards (ENU).
  // This
  //   frame is used by OSM and Google Maps, see https://en.wikipedia.org/wiki/Web_Mercator_projection and
  //   https://developers.google.com/maps/documentation/javascript/coordinates.

  // translation of NavSatFix frame w.r.t. the map frame
  // NOTE: due to ENU convention, orientation is not needed, the tiles are rigidly attached to ENU
  tf2::Vector3 t_navsat_map;

  try
  {
    // Use a real TfBuffer for looking up this transform. The FrameManager only supplies transform to/from the
    // currently selected RViz fixed-frame, which is of no help here.
    auto const tf_navsat_map =
        tf_buffer_->lookupTransform(map_frame_, ref_fix_->header.frame_id, ref_fix_->header.stamp);
    tf2::fromMsg(tf_navsat_map.transform.translation, t_navsat_map);
  }
  catch (tf2::TransformException const& ex)
  {
    // retry the lookup after a while; we do not use the timeout parameter of lookupTransform() as that timeout is in
    // ROS time and the waiting can freeze when ROS time is paused; this freeze would then freeze the whole Rviz UI
    try
    {
      ros::WallDuration(0.01).sleep();
      auto const tf_navsat_map =
          tf_buffer_->lookupTransform(map_frame_, ref_fix_->header.frame_id, ref_fix_->header.stamp);
      tf2::fromMsg(tf_navsat_map.transform.translation, t_navsat_map);
    }
    catch (tf2::TransformException const& ex)
    {
      setStatus(StatusProperty::Error, "Transform", QString::fromStdString(ex.what()));
      return;
    }
  }

  // FIXME: note the <double> template! this is different from center_tile_.coord, otherwise we could just use that
  // since center_tile_ and ref_fix_ are in sync
  auto const ref_fix_tile_coords = fromWGSCoordinate<double>(*ref_coords_, zoom_);

  // In assembleScene() we shift the AerialMap so that the center tile's left-bottom corner has the coordinate (0,0).
  // Therefore we can calculate the NavSatFix coordinate (in the AerialMap frame) by just looking at the fractional part
  // of the coordinate. That is we calculate the offset from the left bottom corner of the center tile.
  auto const center_tile_offset_x = ref_fix_tile_coords.x - std::floor(ref_fix_tile_coords.x);
  // In assembleScene() the tiles are created so that the texture is flipped along the y coordinate. Since we want to
  // calculate the positions of the center tile, we also need to flip the texture's v coordinate here.
  auto const center_tile_offset_y = 1 - (ref_fix_tile_coords.y - std::floor(ref_fix_tile_coords.y));

  double const tile_w_h_m = getTileWH(ref_coords_->lat, zoom_);
  ROS_DEBUG_NAMED("rviz_satellite", "Tile resolution is %.1fm", tile_w_h_m);

  // translation of the center-tile w.r.t. the NavSatFix frame
  tf2::Vector3 t_centertile_navsat = { center_tile_offset_x * tile_w_h_m, center_tile_offset_y * tile_w_h_m, 0 };

  center_tile_pose_.header.frame_id = map_frame_;
  center_tile_pose_.header.stamp = ref_fix_->header.stamp;
  tf2::toMsg(t_navsat_map - t_centertile_navsat, center_tile_pose_.pose.position);
}

void AerialMapDisplay::transformTileToUtmFrame()
{
  if (!ref_fix_ or !center_tile_)
  {
    ROS_FATAL_THROTTLE_NAMED(2, "rviz_satellite", "ref_fix_ not set, can't create transforms");
    return;
  }
  
  // tile ID (integer x/y/zoom) corresponding to the downloaded tile / navsat message
  auto const tile = fromWGSCoordinate<int>(*ref_coords_, zoom_);
  
  // Latitude and longitude of this tile's origin
  auto const tileWGS = toWGSCoordinate<int>(tile, zoom_);

  // Tile's origin in UTM coordinates
  double northing, easting;
  int utm_zone;
  bool northp;

  try
  {
    GeographicLib::UTMUPS::Forward(tileWGS.lat, tileWGS.lon, utm_zone, northp, easting, northing, utm_zone_);
  }
  catch (GeographicLib::GeographicErr& err)
  {
    ROS_ERROR_THROTTLE(2.0, "Error transforming lat-lon to UTM: %s", err.what());
    if (utm_zone_ != GeographicLib::UTMUPS::STANDARD)
    {
      try
      {
        GeographicLib::UTMUPS::Forward(tileWGS.lat, tileWGS.lon, utm_zone, northp, easting, northing,
                                       GeographicLib::UTMUPS::STANDARD);
        ROS_INFO_THROTTLE(2.0, "Trying to autodetect UTM zone instead of using zone %i", utm_zone_);
      }
      catch (GeographicLib::GeographicErr& err)
      {
        setStatus(::rviz::StatusProperty::Error, "UTM", QString::fromStdString(err.what()));
        return;
      }
    }
    else
    {
      setStatus(::rviz::StatusProperty::Error, "UTM", QString::fromStdString(err.what()));
      return;
    }
  }
  
  setStatus(::rviz::StatusProperty::Ok, "UTM", "Conversion from lat/lon to UTM is OK.");

  if (utm_zone != utm_zone_)
  {
    ROS_INFO("UTM zone has been updated to %s.", GeographicLib::UTMUPS::EncodeZone(utm_zone, northp).c_str());
    utm_zone_property_->setInt(utm_zone);
  }

  center_tile_pose_.header.stamp = ref_fix_->header.stamp;
  center_tile_pose_.header.frame_id = utm_frame_;
  center_tile_pose_.pose.position.x = easting;
  center_tile_pose_.pose.position.y = northing;

  switch (z_reference_type_)
  {
    case PositionReferenceType::NAV_SAT_FIX_MESSAGE:
      center_tile_pose_.pose.position.z = ref_fix_->altitude;
      break;
    case PositionReferenceType::TF_FRAME:
      if (z_reference_frame_ == utm_frame_)
      {
        center_tile_pose_.pose.position.z = 0;
        setStatus(StatusProperty::Ok, "Z Reference Transform", "Transform OK.");
      }
      else
      {
        try
        {
          auto const tf_reference_utm =
              tf_buffer_->lookupTransform(utm_frame_, z_reference_frame_, ros::Time(0));
          center_tile_pose_.pose.position.z = tf_reference_utm.transform.translation.z;
          setStatus(StatusProperty::Ok, "Z Reference Transform", "Transform OK.");
        }
        catch (tf2::TransformException const& ex)
        {
          setStatus(StatusProperty::Error, "Z Reference Transform", QString::fromStdString(ex.what()));
        }
      }
      break;
  }
}

void AerialMapDisplay::tfReferencePeriodicUpdate(const ros::TimerEvent&)
{
  if (map_transform_type_ != MapTransformType::VIA_UTM_FRAME || xy_reference_type_ != PositionReferenceType::TF_FRAME)
  {
    return;
  }
  
  if (!ref_fix_ || !center_tile_)
  {
    return;
  }

  try
  {
    auto const tf_reference_utm =
        tf_buffer_->lookupTransform(utm_frame_, xy_reference_frame_, ros::Time(0));
    setStatus(::rviz::StatusProperty::Ok, "XY Reference Transform", "Transform OK.");
    
    try
    {
      WGSCoordinate reference_wgs{};
      const auto& utm_coords = tf_reference_utm.transform.translation;
      GeographicLib::UTMUPS::Reverse(utm_zone_, ref_fix_->latitude >= 0, utm_coords.x, utm_coords.y, reference_wgs.lat,
                                     reference_wgs.lon, true);
      setStatus(::rviz::StatusProperty::Ok, "XY reference UTM conversion", "UTM conversion OK.");

      auto new_fix = boost::make_shared<sensor_msgs::NavSatFix>();
      *new_fix = *ref_fix_;
      new_fix->header.stamp = ros::Time::now();
      new_fix->latitude = reference_wgs.lat;
      new_fix->longitude = reference_wgs.lon;
      new_fix->altitude = utm_coords.z;

      // update the center tile; if it stays the same, at least update the transforms so that z reference is updated
      if (!updateCenterTile(new_fix))
      {
        transformTileToReferenceFrame();
      }
      
      setStatus(StatusProperty::Ok, "Message", "Position reference updated.");
    }
    catch (GeographicLib::GeographicErr& err)
    {
      setStatus(::rviz::StatusProperty::Error, "XY reference UTM conversion", QString::fromStdString(err.what()));
      ROS_ERROR_THROTTLE(2.0, "%s", err.what());
      return;
    }
  }
  catch (tf2::TransformException const& ex)
  {
    setStatus(::rviz::StatusProperty::Error, "XY Reference Transform", QString::fromStdString(ex.what()));
    ROS_ERROR_THROTTLE(2.0, "%s", ex.what());
    return;
  }
}

void AerialMapDisplay::transformMapTileToFixedFrame()
{
  // orientation of the tile w.r.t. the fixed-frame
  Ogre::Quaternion o_centertile_fixed;
  // translation of the tile w.r.t. the fixed-frame
  Ogre::Vector3 t_centertile_fixed;

  auto header = center_tile_pose_.header;
  header.stamp = ros::Time();  // ros::Time::now() would be wrong, see the discussion in #105
  const auto& frame_name = header.frame_id;
  
  auto tile_pose = center_tile_pose_.pose;
  if (z_offset_ != 0.0)
  {
    tile_pose.position.z += z_offset_;
  }
  
  // transform the tile origin to fixed frame
  if (context_->getFrameManager()->transform(header, tile_pose, t_centertile_fixed, o_centertile_fixed))
  {
    setStatus(::rviz::StatusProperty::Ok, "Transform", "Transform OK");

    scene_node_->setPosition(t_centertile_fixed);
    scene_node_->setOrientation(o_centertile_fixed);
  }
  else
  {
    // display error
    std::string error;
    if (context_->getFrameManager()->transformHasProblems(frame_name, ros::Time(), error))
    {
      setStatus(::rviz::StatusProperty::Error, "Transform", QString::fromStdString(error));
    }
    else
    {
      setStatus(::rviz::StatusProperty::Error, "Transform",
                QString::fromStdString("Could not transform from [" + frame_name + "] to Fixed Frame [" +
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

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz::AerialMapDisplay, rviz::Display)
