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

#include <unordered_map>
#include <QtGlobal>
#include <QImage>

#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <OGRE/OgreManualObject.h>
#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreTextureManager.h>
#include <OGRE/OgreImageCodec.h>

#include "rviz/frame_manager.h"
#include "rviz/ogre_helpers/grid.h"
#include "rviz/properties/float_property.h"
#include "rviz/properties/int_property.h"
#include "rviz/properties/property.h"
#include "rviz/properties/quaternion_property.h"
#include "rviz/properties/ros_topic_property.h"
#include "rviz/properties/vector_property.h"
#include "rviz/validate_floats.h"
#include "rviz/display_context.h"

#include "aerialmap_display.h"
#include "General.h"

namespace rviz
{
std::string const AerialMapDisplay::MAP_FRAME = "map";

AerialMapDisplay::AerialMapDisplay() : Display(), dirty_(false)
{
  topic_property_ =
      new RosTopicProperty("Topic", "", QString::fromStdString(ros::message_traits::datatype<sensor_msgs::NavSatFix>()),
                           "sensor_msgs::NavSatFix topic to subscribe to.", this, SLOT(updateTopic()));

  alpha_property_ =
      new FloatProperty("Alpha", 0.7, "Amount of transparency to apply to the map.", this, SLOT(updateAlpha()));
  alpha_ = alpha_property_->getValue().toFloat();
  alpha_property_->setMin(0);
  alpha_property_->setMax(1);
  alpha_property_->setShouldBeSaved(true);

  draw_under_property_ = new Property("Draw Behind", false,
                                      "Rendering option, controls whether or not the map is always"
                                      " drawn behind everything else.",
                                      this, SLOT(updateDrawUnder()));
  draw_under_property_->setShouldBeSaved(true);
  draw_under_ = draw_under_property_->getValue().toBool();

  // output, resolution of the map in meters/pixel
  resolution_property_ = new FloatProperty("Resolution", 0, "Resolution of the map. (Read only)", this);
  resolution_property_->setReadOnly(true);

  // properties for map
  tile_url_property_ =
      new StringProperty("Object URI", "", "URL from which to retrieve map tiles.", this, SLOT(updateTileUrl()));
  tile_url_property_->setShouldBeSaved(true);
  tile_url_ = tile_url_property_->getStdString();

  QString const zoom_desc = QString::fromStdString("Zoom level (0 - " + std::to_string(maxZoom) + ")");
  zoom_property_ = new IntProperty("Zoom", 16, zoom_desc, this, SLOT(updateZoom()));
  zoom_property_->setShouldBeSaved(true);
  zoom_property_->setMin(0);
  zoom_property_->setMax(maxZoom);
  zoom_ = zoom_property_->getInt();

  QString const blocks_desc = QString::fromStdString("Adjacent blocks (0 - " + std::to_string(maxBlocks) + ")");
  blocks_property_ = new IntProperty("Blocks", 3, blocks_desc, this, SLOT(updateBlocks()));
  blocks_property_->setShouldBeSaved(true);
  blocks_property_->setMin(0);
  blocks_property_->setMax(maxBlocks);
  blocks_ = blocks_property_->getInt();
}

AerialMapDisplay::~AerialMapDisplay()
{
  unsubscribe();
  clear();
}

void AerialMapDisplay::onEnable()
{
  subscribe();
}

void AerialMapDisplay::onDisable()
{
  unsubscribe();
  clear();
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
      coord_sub_ = update_nh_.subscribe(topic_property_->getTopicStd(), 1, &AerialMapDisplay::navFixCallback, this);

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
  coord_sub_.shutdown();
  ROS_INFO("Unsubscribing.");
}

void AerialMapDisplay::updateAlpha()
{
  alpha_ = alpha_property_->getFloat();
  dirty_ = true;  // force texture repaint
  ROS_INFO("Changing alpha to %f", alpha_);
}

void AerialMapDisplay::updateDrawUnder()
{
  draw_under_ = draw_under_property_->getValue().toBool();
  dirty_ = true;  // force texture repaint
  ROS_INFO("Changing draw_under to %s", ((draw_under_) ? "true" : "false"));
}

void AerialMapDisplay::updateTileUrl()
{
  tile_url_ = tile_url_property_->getStdString();
  loadTileTextures();
}

void AerialMapDisplay::updateZoom()
{
  int const zoom = std::max(0, std::min(maxZoom, zoom_property_->getInt()));
  if (zoom != zoom_)
  {
    zoom_ = zoom;

    clear(); // force tile geometry rebuild
    if(ref_fix_) {
      updateCenterTile(ref_fix_); // force texture query
    }
  }
}

void AerialMapDisplay::updateBlocks()
{
  int const blocks = std::max(0, std::min(maxBlocks, blocks_property_->getInt()));
  if (blocks != blocks_)
  {
    blocks_ = blocks;

    clear();             // force tile geometry rebuild
    loadTileTextures();  // force texture query
  }
}

void AerialMapDisplay::updateTopic()
{
  unsubscribe();
  clear();
  subscribe();
}

void AerialMapDisplay::clear()
{
  ref_fix_ = nullptr;
  lastCenterTile_ = boost::none;
  clearGeometry();

  setStatus(StatusProperty::Warn, "Message", "No map received yet");
}

void AerialMapDisplay::clearGeometry()
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
  dirty_ = true;
}

void AerialMapDisplay::createGeometry()
{
  for (int block = 0; block < (2 * blocks_ + 1) * (2 * blocks_ + 1); ++block)
  {
    // generate an unique name
    static int count = 0;
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
  // update tiles, if necessary
  assembleScene();
  // transform map-fixed scene object into fixed frame
  transformMapTileToFixedFrame();
}

void AerialMapDisplay::navFixCallback(sensor_msgs::NavSatFixConstPtr const& msg)
{
  // TODO: even if the center tile does not change, our lat/lon and tf relationship could have been changed? so update the transform either way?
  updateCenterTile(msg);
}

void AerialMapDisplay::updateCenterTile(sensor_msgs::NavSatFixConstPtr const& msg)
{
  if (!isEnabled())
  {
    return;
  }

  // check if update is necessary
  auto const tileCoordinates = fromWGSCoordinate({ msg->latitude, msg->longitude }, zoom_);
  TileId const newCenterTileID { tile_url_, tileCoordinates, zoom_ };
  bool const centerTileChanged = (!lastCenterTile_ || !(newCenterTileID == *lastCenterTile_));

  if (not centerTileChanged)
  {
    return;
  }

  lastCenterTile_ = newCenterTileID;
  ref_fix_ = msg;

  loadTileTextures();
  transformTileToMapFrame();
}

void AerialMapDisplay::loadTileTextures()
{
  if (!isEnabled())
  {
    return;
  }

  if (tile_url_.empty())
  {
    setStatus(StatusProperty::Error, "Message", "Tile URL is not set");
    return;
  }

  if (not lastCenterTile_)
  {
    setStatus(StatusProperty::Error, "Message", "No NavSatFix received yet");
    return;
  }

  try
  {
    tileCache_.request({ *lastCenterTile_, blocks_ });
    dirty_ = true;
  }
  catch (std::exception const& e)
  {
    setStatus(StatusProperty::Error, "Message", QString(e.what()));
    return;
  }

  // the following error rate thresholds are randomly chosen
  float const errorRate = tileCache_.getTileServerErrorRate(tile_url_);
  if (errorRate > 0.95)
  {
    setStatus(StatusProperty::Level::Error, "Message", "Few or no tiles received");
  }
  else if (errorRate > 0.3)
  {
    setStatus(StatusProperty::Level::Warn, "Message",
              "Not all requested tiles have been received. Possibly the server is throttling?");
  }
  else
  {
    setStatus(StatusProperty::Level::Ok, "Message", "OK");
  }
}

void AerialMapDisplay::assembleScene()
{
  if (!isEnabled() || !dirty_ || !lastCenterTile_)
  {
    return;
  }
  dirty_ = false;

  // was clearGeometry() called?
  if (objects_.empty())
  {
    createGeometry();

    // e.g. when the number of blocks got bigger, the new tiles have to be loaded
    tileCache_.request({ *lastCenterTile_, blocks_ });
  }

  TileId tileId{ tile_url_, fromWGSCoordinate({ ref_fix_->latitude, ref_fix_->longitude }, zoom_), zoom_ };
  Area area(tileId, blocks_);

  TileCacheGuard guard(tileCache_);

  bool loadedAllTiles = true;

  auto it = objects_.begin();
  for (int xx = area.leftTop.x; xx <= area.rightBottom.x; ++xx)
  {
    for (int yy = area.leftTop.y; yy <= area.rightBottom.y; ++yy)
    {
      auto obj = it->object;
      auto& material = it->material;
      assert(!material.isNull());
      ++it;

      TileId const toFind{ tileId.tileServer, { xx, yy }, tileId.zoom };

      OgreTile const* tile = tileCache_.ready(toFind);
      if (!tile)
      {
        // don't show tiles with old textures
        obj->setVisible(false);
        loadedAllTiles = false;
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
      double const tile_w_h_m = getTileWH();

      // Note: In the following we will do two things:
      //
      // * We flip the position's y coordinate.
      // * We flip the texture's v transformAerialMap.
      //
      // For more explanation see the function transformAerialMap()

      // In the AerialMap frame we to have the center tile at the position left-bot = (0,0) and right-top = (1,1)
      double const x = (xx - lastCenterTile_->coord.x) * tile_w_h_m;
      // flip the y coordinate because we need to flip the tiles to align the tile's frame with the ENU "map" frame
      double const y = -(yy - lastCenterTile_->coord.y) * tile_w_h_m;

      // create a quad for this tile
      // note: We have to recreate the vertices and cannot reuse the old vertices: tile_w_h_m depends on the latitude
      obj->clear();

      obj->begin(material->getName(), Ogre::RenderOperation::OT_TRIANGLE_LIST);

      // We flip the texture along the v coordinate by just assigning the Ogre texture coordinates appropriately:
      // The idea we e.g. assign the bottom left corner of the tile the top left texture corner.
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

  // since not all tiles were loaded, this function has to be called again
  if (!loadedAllTiles)
  {
    dirty_ = true;
  }

  tileCache_.purge({ tileId, blocks_ });
}

void AerialMapDisplay::transformTileToMapFrame()
{
  auto const currentFixedFrame = context_->getFrameManager()->getFixedFrame();

  // orientation from nav sat fix frame to the map frame
  Ogre::Quaternion o_navsat_map;
  // translation from nav sat fix frame to the map frame
  Ogre::Vector3 t_navsat_map;
  // temporarily set the fixed frame to map
  // (Unfortunately the API does not allow looking up arbitrary frame transforms, only towards the currently selected
  // fixed-frame. Alternatively, we could use the TfBuffer, or work with two transforms (ref_fix to fixed-frame and map
  // to fixed-frame.)
  context_->getFrameManager()->setFixedFrame(MAP_FRAME);
  if (!context_->getFrameManager()->getTransform(ref_fix_->header.frame_id, ref_fix_->header.stamp, t_navsat_map,
                                                 o_navsat_map))
  {
    // display error
    std::string error;
    if (context_->getFrameManager()->transformHasProblems(ref_fix_->header.frame_id, ref_fix_->header.stamp, error))
    {
      setStatus(StatusProperty::Error, "Transform", QString::fromStdString(error));
    }
    else
    {
      setStatus(StatusProperty::Error, "Transform",
                "Could not transform from [" + QString::fromStdString(ref_fix_->header.frame_id) +
                    "] to Map Frame [map] for an unknown reason");
    }
    // don't forget to reset the fixed frame :)
    context_->getFrameManager()->setFixedFrame(currentFixedFrame);
    return;
  }

  // don't forget to reset the fixed frame :)
  context_->getFrameManager()->setFixedFrame(currentFixedFrame);

  auto const centerTile = fromWGSCoordinate<double>({ ref_fix_->latitude, ref_fix_->longitude }, zoom_);

  // In assembleScene() we shifted the AerialMap so that the center tile's left-bottom corner has the coordinate (0,0).
  // Therefore we can calculate the NavSatFix coordinate (in the AerialMap frame) by just looking at the fractional part
  // of the coordinate. That is we calculate the offset from the left bottom corner of the center tile.
  auto const centerTileOffsetX = centerTile.x - std::floor(centerTile.x);
  // In assembleScene() the tiles are created so that the texture is flipped along the y coordinate. Since we want to
  // calculate the positions of the center tile, we also need to flip the texture's v coordinate here.
  auto const centerTileOffsetY = 1 - (centerTile.y - std::floor(centerTile.y));

  double const tile_w_h_m = getTileWH();
  auto const translationAerialMapToNavSatFix =
      Ogre::Vector3(centerTileOffsetX * tile_w_h_m, centerTileOffsetY * tile_w_h_m, 0);
  auto const translationNavSatFixToAerialMap = -translationAerialMapToNavSatFix;

  t_map_center_tile_ = t_navsat_map + translationNavSatFixToAerialMap;
}

void AerialMapDisplay::transformMapTileToFixedFrame()
{
  Ogre::Quaternion orientation;
  Ogre::Vector3 position;

  // get transform between map frame and fixed_frame from the FrameManager
  if (context_->getFrameManager()->getTransform(MAP_FRAME, ros::Time(), position, orientation))
  {
    setStatus(::rviz::StatusProperty::Ok, "Transform", "Transform OK");

    // apply transform
    scene_node_->setPosition(position + orientation * t_map_center_tile_);
    scene_node_->setOrientation(orientation);
  }
  else
  {
    // display error
    std::string error;
    if (context_->getFrameManager()->transformHasProblems(MAP_FRAME, ros::Time(), error))
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
  // unsub,clear,resub
  updateTopic();
}

}  // namespace rviz

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz::AerialMapDisplay, rviz::Display)
