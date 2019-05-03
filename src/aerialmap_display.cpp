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
#include <OGRE/OgreVector3.h>

#include "rviz/frame_manager.h"
#include "rviz/ogre_helpers/grid.h"
#include "rviz/properties/enum_property.h"
#include "rviz/properties/float_property.h"
#include "rviz/properties/int_property.h"
#include "rviz/properties/property.h"
#include "rviz/properties/tf_frame_property.h"
#include "rviz/properties/quaternion_property.h"
#include "rviz/properties/ros_topic_property.h"
#include "rviz/properties/vector_property.h"
#include "rviz/validate_floats.h"
#include "rviz/display_context.h"

#include "aerialmap_display.h"
#include "General.h"

int constexpr FRAME_CONVENTION_XYZ_ENU = 0;  //  X -> East, Y -> North
int constexpr FRAME_CONVENTION_XYZ_NED = 1;  //  X -> North, Y -> East
int constexpr FRAME_CONVENTION_XYZ_NWU = 2;  //  X -> North, Y -> West

namespace rviz
{
AerialMapDisplay::AerialMapDisplay() : Display(), dirty_(false), received_msg_(false)
{
  topic_property_ =
      new RosTopicProperty("Topic", "", QString::fromStdString(ros::message_traits::datatype<sensor_msgs::NavSatFix>()),
                           "nav_msgs::Odometry topic to subscribe to.", this, SLOT(updateTopic()));

  frame_property_ = new TfFrameProperty("Robot frame", "world", "TF frame for the moving robot.", this, nullptr, false,
                                        SLOT(updateFrame()), this);

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

  frame_convention_property_ =
      new EnumProperty("Frame Convention", "XYZ -> ENU", "Convention for mapping cartesian frame to the compass", this,
                       SLOT(updateFrameConvention()));
  frame_convention_property_->addOptionStd("XYZ -> ENU", FRAME_CONVENTION_XYZ_ENU);
  frame_convention_property_->addOptionStd("XYZ -> NED", FRAME_CONVENTION_XYZ_NED);
  frame_convention_property_->addOptionStd("XYZ -> NWU", FRAME_CONVENTION_XYZ_NWU);

  //  updating one triggers reload
  updateBlocks();
}

AerialMapDisplay::~AerialMapDisplay()
{
  unsubscribe();
  clear();
}

void AerialMapDisplay::onInitialize()
{
  frame_property_->setFrameManager(context_->getFrameManager());
}

void AerialMapDisplay::onEnable()
{
  lastFixedFrame_ = context_->getFrameManager()->getFixedFrame();
  subscribe();
}

void AerialMapDisplay::onDisable()
{
  unsubscribe();
  clear();
  context_->getFrameManager()->setFixedFrame(lastFixedFrame_);
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
  dirty_ = true;
  ROS_INFO("Changing alpha to %f", alpha_);
}

void AerialMapDisplay::updateFrame()
{
  ROS_INFO_STREAM("Changing robot frame to " << frame_property_->getFrameStd());
  transformAerialMap();
}

void AerialMapDisplay::updateDrawUnder()
{
  // @todo figure out why this property only applies to some objects
  draw_under_ = draw_under_property_->getValue().toBool();
  dirty_ = true;  //  force update
  ROS_INFO("Changing draw_under to %s", ((draw_under_) ? "true" : "false"));
}

void AerialMapDisplay::updateTileUrl()
{
  tile_url_ = tile_url_property_->getStdString();
}

void AerialMapDisplay::updateZoom()
{
  int const zoom = std::max(0, std::min(maxZoom, zoom_property_->getInt()));
  if (zoom != zoom_)
  {
    zoom_ = zoom;

    clear();
  }
}

void AerialMapDisplay::updateBlocks()
{
  int const blocks = std::max(0, std::min(maxBlocks, blocks_property_->getInt()));
  if (blocks != blocks_)
  {
    blocks_ = blocks;

    clear();
  }
}

void AerialMapDisplay::updateFrameConvention()
{
  transformAerialMap();
}

void AerialMapDisplay::updateTopic()
{
  unsubscribe();
  clear();
  subscribe();
}

void AerialMapDisplay::clear()
{
  setStatus(StatusProperty::Warn, "Message", "No map received");
  clearGeometry();
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
  // create all geometry, if necessary
  assembleScene();

  // draw
  context_->queueRender();
}

void AerialMapDisplay::navFixCallback(sensor_msgs::NavSatFixConstPtr const& msg)
{
  ref_fix_ = *msg;

  // re-load imagery
  received_msg_ = true;
  transformAerialMap();
  loadImagery();
}

void AerialMapDisplay::loadImagery()
{
  if (!isEnabled())
  {
    return;
  }

  // When the plugin starts, the properties from the config are set.
  // This will call the callbacks and these will call this function.
  // By checking if a message was received, we prevent to update the
  // images when Rviz loads the config.
  if (!received_msg_)
  {
    return;
  }

  if (!hasWorkingTransform_)
  {
    return;
  }

  if (tile_url_.empty())
  {
    setStatus(StatusProperty::Error, "Message", "Tile URL is not set");
    return;
  }

  TileId tileId{ tile_url_, fromWGSCoordinate({ ref_fix_.latitude, ref_fix_.longitude }, zoom_), zoom_ };
  if (!lastTileId_ || !(tileId == *lastTileId_))
  {
    lastTileId_ = tileId;

    try
    {
      tileCache_.request({ tileId, blocks_ });
      dirty_ = true;
    }
    catch (std::exception& e)
    {
      setStatus(StatusProperty::Error, "Message", QString(e.what()));
      return;
    }
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
  if (!isEnabled() || !dirty_ || !lastTileId_)
  {
    return;
  }
  dirty_ = false;

  // was clearGeometry() called?
  if (objects_.empty())
  {
    createGeometry();

    // e.g. when the number of blocks got bigger, the new tiles have to be loaded
    tileCache_.request({ *lastTileId_, blocks_ });
  }

  TileId tileId{ tile_url_, fromWGSCoordinate({ ref_fix_.latitude, ref_fix_.longitude }, zoom_), zoom_ };
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

      // determine position of this tile relative to the robot's position
      // note: We invert the y-axis so that positive y corresponds to north. See transformAerialMap().
      double const x = (xx - lastTileId_->coord.x) * tile_w_h_m;
      double const y = -(yy - lastTileId_->coord.y) * tile_w_h_m;

      // create a quad for this tile
      // note: We have to create the vertices everytime and cannot reuse the old vertices: tile_w_h_m depends on the
      // latitude.
      obj->clear();

      obj->begin(material->getName(), Ogre::RenderOperation::OT_TRIANGLE_LIST);

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

void AerialMapDisplay::transformAerialMap()
{
  if (!isEnabled())
  {
    return;
  }

  // the parent frame of this scene node
  static std::string const mapFrame = "map";

  // the robot's frame
  std::string const frame = frame_property_->getFrameStd();

  // note: the orientation is not used on purpose
  Ogre::Quaternion orientation;
  Ogre::Vector3 position;

  // can mapFrame be used as a fixed frame?
  std::string errMsg;
  if (context_->getFrameManager()->frameHasProblems(mapFrame, ros::Time{}, errMsg) ||
      context_->getFrameManager()->transformHasProblems(mapFrame, ros::Time{}, errMsg))
  {
    context_->getFrameManager()->setFixedFrame(lastFixedFrame_);
    setStatus(StatusProperty::Error, "Transform", QString::fromStdString(errMsg));
    hasWorkingTransform_ = false;
    clear();
    return;
  }

  // Set the pseudo fixed frame to map so that we can align the tiles to north just by setting identity as the
  // orientation
  context_->getFrameManager()->setFixedFrame(mapFrame);

  // Get the latest transform between the robot frame and fixed frame from the FrameManager
  if (!context_->getFrameManager()->getTransform(frame, ros::Time(), position, orientation))
  {
    // display error
    std::string error;
    if (context_->getFrameManager()->transformHasProblems(frame, ros::Time(), error))
    {
      setStatus(StatusProperty::Error, "Transform", QString::fromStdString(error));
    }
    else
    {
      setStatus(StatusProperty::Error, "Transform",
                "Could not transform from [" + QString::fromStdString(frame) + "] to Fixed Frame [" + fixed_frame_ +
                    "] for an unknown reason");
    }
    hasWorkingTransform_ = false;
    clear();
    return;
  }

  if (position.isNaN())
  {
    // This can occur if an invalid TF is published.
    // Show an error and don't apply anything, so OGRE does not throw an assertion.
    setStatus(StatusProperty::Error, "Transform", "Received invalid transform");
    hasWorkingTransform_ = false;
    clear();
    return;
  }

  hasWorkingTransform_ = true;
  setStatus(StatusProperty::Ok, "Transform", "Transform OK");

  // apply transform and add offset from origin
  // decimal places are needed here to calculate the offset
  auto const center = fromWGSCoordinate<double>({ ref_fix_.latitude, ref_fix_.longitude }, zoom_);
  auto const originOffsetX = center.x - std::floor(center.x);
  auto const originOffsetY = 1 - center.y + std::floor(center.y);  // y coord is flipped
  double const tile_w_h_m = getTileWH();
  position.x -= originOffsetX * tile_w_h_m;
  position.y -= originOffsetY * tile_w_h_m;
  scene_node_->setPosition(position);

  int const convention = frame_convention_property_->getOptionInt();
  if (convention == FRAME_CONVENTION_XYZ_ENU)
  {
    // ENU corresponds to our default drawing method
    scene_node_->setOrientation(Ogre::Quaternion::IDENTITY);
  }
  else if (convention == FRAME_CONVENTION_XYZ_NED)
  {
    // XYZ->NED will cause the map to appear reversed when viewed from above (from +z).
    // clang-format off
		Ogre::Matrix3 const xyz_R_ned(0, 1, 0,
		                              1, 0, 0,
		                              0, 0, -1);
    // clang-format on
    scene_node_->setOrientation(xyz_R_ned.Transpose());
  }
  else if (convention == FRAME_CONVENTION_XYZ_NWU)
  {
    // clang-format off
		Ogre::Matrix3 const xyz_R_nwu(0, -1, 0,
		                              1, 0, 0,
		                              0, 0, 1);
    // clang-format on
    scene_node_->setOrientation(xyz_R_nwu.Transpose());
  }
  else
  {
    ROS_ERROR_STREAM("Invalid convention code: " << convention);
  }
}

void AerialMapDisplay::fixedFrameChanged()
{
  transformAerialMap();
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
