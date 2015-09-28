/*
 * AerialMapDisplay.cpp
 *
 *  Copyright (c) 2014 Gaeth Cross. Apache 2 License.
 *
 *  This file is part of rviz_satellite.
 *
 *	Created on: 07/09/2014
 */

#include <QtGlobal>
#include <QImage>

#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <boost/bind.hpp>
#include <boost/regex.hpp>
#include <FreeImage.h>

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

Ogre::TexturePtr textureFromBytes(const QByteArray &ba,
                                  const std::string &name) {

  static bool fi_init = false;
  if (!fi_init) {
    FreeImage_Initialise();
  }

  void *data = const_cast<char *>(ba.data());
  FIMEMORY *mem =
      FreeImage_OpenMemory(reinterpret_cast<BYTE *>(data), ba.size());
  FREE_IMAGE_FORMAT fif = FreeImage_GetFileTypeFromMemory(mem, 0);
  if (fif == FIF_UNKNOWN) {
    FreeImage_CloseMemory(mem);
    throw std::runtime_error("Image format is not supported for loading");
  }
  FIBITMAP *bmp = FreeImage_LoadFromMemory(fif, mem, 0);
  FreeImage_CloseMemory(mem);
  if (!bmp) {
    throw std::runtime_error("Failed to decode image");
  }
  FIBITMAP *converted = FreeImage_ConvertTo24Bits(bmp);
  FreeImage_Unload(bmp);
  if (!converted) {
    throw std::runtime_error("Failed to convert image to 24 bit");
  }

  const unsigned w = FreeImage_GetWidth(converted);
  const unsigned h = FreeImage_GetHeight(converted);
  const unsigned data_size = w * h * 3;
  BYTE *image_data = FreeImage_GetBits(converted);

  ROS_INFO("Loading a %u x %u texture", w, h);

  //  create texture
  Ogre::TexturePtr texture;
  try {
    Ogre::DataStreamPtr data_stream;
    data_stream.bind(new Ogre::MemoryDataStream(image_data, data_size));

    const Ogre::String res_group =
        Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME;
    Ogre::TextureManager &texture_manager =
        Ogre::TextureManager::getSingleton();
    texture =
        texture_manager.loadRawData(name, res_group, data_stream, w, h,
                                    Ogre::PF_R8G8B8, Ogre::TEX_TYPE_2D, 0);
  }
  catch (...) {
    //  clean up FreeImage before re-throwing
    FreeImage_Unload(converted);
    throw;
  }

  return texture;
}

Ogre::TexturePtr textureFromImage(const QImage &image,
                                  const std::string &name) {
  ROS_INFO("Loading a %i x %i texture", image.width(), image.height());
  //  convert to 24bit rgb
  QImage converted = image.convertToFormat(QImage::Format_RGB888).mirrored();

  //  create texture
  Ogre::TexturePtr texture;
  Ogre::DataStreamPtr data_stream;
  data_stream.bind(new Ogre::MemoryDataStream((void *)converted.constBits(),
                                              converted.byteCount()));

  const Ogre::String res_group =
      Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME;
  Ogre::TextureManager &texture_manager = Ogre::TextureManager::getSingleton();
  //  swap byte order when going from QImage to Ogre
  texture = texture_manager.loadRawData(name, res_group, data_stream,
                                        converted.width(), converted.height(),
                                        Ogre::PF_B8G8R8, Ogre::TEX_TYPE_2D, 0);
  return texture;
}

namespace rviz {

AerialMapDisplay::AerialMapDisplay()
    : Display(), map_id_(0), scene_id_(0), dirty_(false),
      received_msg_(false), loader_(0) {

  static unsigned int map_ids = 0;
  map_id_ = map_ids++; //  global counter of map ids

  topic_property_ = new RosTopicProperty(
      "Topic", "", QString::fromStdString(
                       ros::message_traits::datatype<sensor_msgs::NavSatFix>()),
      "nav_msgs::Odometry topic to subscribe to.", this, SLOT(updateTopic()));

  alpha_property_ = new FloatProperty(
      "Alpha", 0.7, "Amount of transparency to apply to the map.", this,
      SLOT(updateAlpha()));
  alpha_ = alpha_property_->getValue().toFloat();
  alpha_property_->setMin(0);
  alpha_property_->setMax(1);
  alpha_property_->setShouldBeSaved(true);

  draw_under_property_ =
      new Property("Draw Behind", false,
                   "Rendering option, controls whether or not the map is always"
                   " drawn behind everything else.",
                   this, SLOT(updateDrawUnder()));
  draw_under_property_->setShouldBeSaved(true);
  draw_under_ = draw_under_property_->getValue().toBool();

  //  output, resolution of the map in meters/pixel
  resolution_property_ = new FloatProperty(
      "Resolution", 0, "Resolution of the map. (Read only)", this);
  resolution_property_->setReadOnly(true);

  //  properties for map
  object_uri_property_ = new StringProperty(
      "Object URI", "http://otile1.mqcdn.com/tiles/1.0.0/sat/{z}/{x}/{y}.jpg",
      "URL from which to retrieve map tiles.", this, SLOT(updateObjectURI()));
  object_uri_property_->setShouldBeSaved(true);
  object_uri_ = object_uri_property_->getStdString();

  zoom_property_ = new IntProperty("Zoom", 16, "Zoom level (0 - 19 usually)",
                                   this, SLOT(updateZoom()));
  zoom_property_->setShouldBeSaved(true);
  zoom_ = zoom_property_->getInt();

  blocks_property_ =
      new IntProperty("Blocks", 3, "Number of adjacent blocks (6 max)", this,
                      SLOT(updateBlocks()));
  blocks_property_->setShouldBeSaved(true);

  //  updating one triggers reload
  updateBlocks();
}

AerialMapDisplay::~AerialMapDisplay() {
  unsubscribe();
  clear();
}

void AerialMapDisplay::onInitialize() {}

void AerialMapDisplay::onEnable() { subscribe(); }

void AerialMapDisplay::onDisable() {
  unsubscribe();
  clear();
}

void AerialMapDisplay::subscribe() {
  if (!isEnabled()) {
    return;
  }

  if (!topic_property_->getTopic().isEmpty()) {
    try {
      ROS_INFO("Subscribing to %s", topic_property_->getTopicStd().c_str());
      coord_sub_ =
          update_nh_.subscribe(topic_property_->getTopicStd(), 1,
                               &AerialMapDisplay::navFixCallback, this);

      setStatus(StatusProperty::Ok, "Topic", "OK");
    }
    catch (ros::Exception &e) {
      setStatus(StatusProperty::Error, "Topic",
                QString("Error subscribing: ") + e.what());
    }
  }
}

void AerialMapDisplay::unsubscribe() {
  coord_sub_.shutdown();
  ROS_INFO("Unsubscribing.");
}

void AerialMapDisplay::updateAlpha() {
  alpha_ = alpha_property_->getFloat();
  dirty_ = true;
  ROS_INFO("Changing alpha to %f", alpha_);
}

void AerialMapDisplay::updateDrawUnder() {
  /// @todo: figure out why this property only applies to some objects
  draw_under_ = draw_under_property_->getValue().toBool();
  dirty_ = true; //  force update
  ROS_INFO("Changing draw_under to %s", ((draw_under_) ? "true" : "false"));
}

void AerialMapDisplay::updateObjectURI() {
  object_uri_ = object_uri_property_->getStdString();
  loadImagery(); //  reload all imagery
}

void AerialMapDisplay::updateZoom() {
  int zoom = zoom_property_->getInt();
  //  validate
  zoom = std::max(0, std::min(19, zoom));
  if (zoom != static_cast<int>(zoom_)) {
    zoom_ = zoom;
    ROS_INFO("Zoom changed to %i, will reload imagery", zoom_);
    loadImagery(); //  reload
  }
}

void AerialMapDisplay::updateBlocks() {
  int blocks = blocks_property_->getInt();
  blocks = std::max(0, std::min(6, blocks)); //  arbitrary limit for now
  if (blocks != static_cast<int>(blocks_)) {
    blocks_ = blocks;
    loadImagery();
  }
}

void AerialMapDisplay::updateTopic() {
  unsubscribe();
  clear();
  subscribe();
}

void AerialMapDisplay::clear() {
  setStatus(StatusProperty::Warn, "Message", "No map received");
  clearGeometry();
  //  the user has cleared here
  received_msg_ = false;
  //  stop any loading...
  if (loader_) {
    ROS_INFO("Clearing loaded imagery.");
    //  cancel current imagery, if any
    loader_->abort();
    delete loader_;
    loader_ = 0;
  }
}

void AerialMapDisplay::clearGeometry() {
  for (MapObject &obj : objects_) {
    //  destroy object
    scene_manager_->destroyManualObject(obj.object);
    //  destory texture
    if (!obj.texture.isNull()) {
      const std::string tex_name = obj.texture->getName();
      obj.texture.setNull();
      Ogre::TextureManager::getSingleton().unload(tex_name);
    }
    //  destroy material
    if (!obj.material.isNull()) {
      const std::string mat_name = obj.material->getName();
      obj.material.setNull();
      Ogre::MaterialManager::getSingleton().unload(mat_name);
    }
  }
  objects_.clear();
}

void AerialMapDisplay::update(float, float) {
  boost::mutex::scoped_lock lock(mutex_);
  //  creates all geometry, if necessary
  assembleScene();
  //  apply transformation
  transformAerialMap();
  //  draw
  context_->queueRender();
}

void
AerialMapDisplay::navFixCallback(const sensor_msgs::NavSatFixConstPtr &msg) {
  //  only re-load if coordinates changed, in case the topic is not latched
  if (msg->latitude != ref_lat_ || msg->longitude != ref_lon_ ||
      !received_msg_) {
    ref_lat_ = msg->latitude;
    ref_lon_ = msg->longitude;
    ROS_INFO("Reference point set to: %f, %f", ref_lat_, ref_lon_);
    setStatus(StatusProperty::Warn, "Message", "Loading map tiles.");

    //  re-load imagery
    received_msg_ = true;
    loadImagery();
  }
}

void AerialMapDisplay::loadImagery() {
  if (loader_) {
    //  cancel current imagery, if any
    loader_->abort();
    delete loader_;
    loader_ = 0;
  }
  if (!received_msg_) {
    //  no message received from publisher
    return;
  }
  if (object_uri_.empty()) {
    setStatus(StatusProperty::Error, "Message",
              "Received message but object URI is not set");
  }
  const std::string service = object_uri_;
  try {
    loader_ = new TileLoader(service, ref_lat_, ref_lon_, zoom_, blocks_, this);
  }
  catch (std::exception &e) {
    setStatus(StatusProperty::Error, "Message", QString(e.what()));
    return;
  }

  QObject::connect(loader_, SIGNAL(errorOcurred(QString)), this,
                   SLOT(errorOcurred(QString)));
  QObject::connect(loader_, SIGNAL(finishedLoading()), this,
                   SLOT(finishedLoading()));
  QObject::connect(loader_, SIGNAL(initiatedRequest(QNetworkRequest)), this,
                   SLOT(initiatedRequest(QNetworkRequest)));
  QObject::connect(loader_, SIGNAL(receivedImage(QNetworkRequest)), this,
                   SLOT(receivedImage(QNetworkRequest)));
  //  start loading images
  loader_->start();
}

void AerialMapDisplay::assembleScene() {
  if (!dirty_) {
    return; //  nothing to update
  }
  dirty_ = false;
  
  if (!loader_) {
    return; //  no tiles loaded, don't do anything
  }
  
  //  get rid of old geometry, we will re-build this
  clearGeometry();
  
  //  iterate over all tiles and create an object for each of them
  const double resolution = loader_->resolution();
  const std::vector<TileLoader::MapTile> &tiles = loader_->tiles();
  for (const TileLoader::MapTile &tile : tiles) {
    const int w = tile.image().width();
    const int h = tile.image().height();
    //  we here assume that the tiles are uniformly sized...
    const double tileW = w * resolution;
    const double tileH = h * resolution;
    const double origin_x = -loader_->originX() * tileW;
    const double origin_y = -(1 - loader_->originY()) * tileH;

    //  determine location of this tile
    const double x = (tile.x() - loader_->tileX()) * tileW + origin_x;
    const double y = -(tile.y() - loader_->tileY()) * tileH + origin_y;
    //  don't re-use any ids
    const std::string name_suffix =
        std::to_string(tile.x()) + "_" + std::to_string(tile.y()) + "_" +
        std::to_string(map_id_) + "_" + std::to_string(scene_id_);

    Ogre::TexturePtr tex;
    if (tile.hasImage()) {
      //  one material per texture
      std::string matName = "material_" + name_suffix;
      Ogre::MaterialPtr material = Ogre::MaterialManager::getSingleton().create(
          matName, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
      material->setReceiveShadows(false);
      material->getTechnique(0)->setLightingEnabled(false);
      material->setDepthBias(-16.0f,
                             0.0f); /// @todo: what the fuck does this do?
      material->setCullingMode(Ogre::CULL_NONE);
      material->setDepthWriteEnabled(false);

      //  create textureing unit
      Ogre::Pass *pass = material->getTechnique(0)->getPass(0);
      Ogre::TextureUnitState *tex_unit = NULL;
      if (pass->getNumTextureUnitStates() > 0) {
        tex_unit = pass->getTextureUnitState(0);
      } else {
        tex_unit = pass->createTextureUnitState();
      }

      //  only add if we have a texture for it
      tex = textureFromImage(tile.image(), "texture_" + name_suffix);

      ROS_INFO("Rendering with texture: %s", tex->getName().c_str());
      tex_unit->setTextureName(tex->getName());
      tex_unit->setTextureFiltering(Ogre::TFO_BILINEAR);

      //  create an object
      const std::string obj_name = "object_" + name_suffix;
      Ogre::ManualObject *obj = scene_manager_->createManualObject(obj_name);
      scene_node_->attachObject(obj);

      //  configure depth & alpha properties
      if (alpha_ >= 0.9998) {
        material->setDepthWriteEnabled(!draw_under_);
        material->setSceneBlending(Ogre::SBT_REPLACE);
      } else {
        material->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
        material->setDepthWriteEnabled(false);
      }

      if (draw_under_) {
        obj->setRenderQueueGroup(Ogre::RENDER_QUEUE_4);
      } else {
        obj->setRenderQueueGroup(Ogre::RENDER_QUEUE_MAIN);
      }

      tex_unit->setAlphaOperation(Ogre::LBX_SOURCE1, Ogre::LBS_MANUAL,
                                  Ogre::LBS_CURRENT, alpha_);

      //  create a quad for this tile
      obj->begin(material->getName(), Ogre::RenderOperation::OT_TRIANGLE_LIST);

      //  bottom left
      obj->position(x, y, 0.0f);
      obj->textureCoord(0.0f, 0.0f);
      obj->normal(0.0f, 0.0f, 1.0f);

      // top right
      obj->position(x + tileW, y + tileH, 0.0f);
      obj->textureCoord(1.0f, 1.0f);
      obj->normal(0.0f, 0.0f, 1.0f);

      // top left
      obj->position(x, y + tileH, 0.0f);
      obj->textureCoord(0.0f, 1.0f);
      obj->normal(0.0f, 0.0f, 1.0f);

      //  bottom left
      obj->position(x, y, 0.0f);
      obj->textureCoord(0.0f, 0.0f);
      obj->normal(0.0f, 0.0f, 1.0f);

      // bottom right
      obj->position(x + tileW, y, 0.0f);
      obj->textureCoord(1.0f, 0.0f);
      obj->normal(0.0f, 0.0f, 1.0f);

      // top right
      obj->position(x + tileW, y + tileH, 0.0f);
      obj->textureCoord(1.0f, 1.0f);
      obj->normal(0.0f, 0.0f, 1.0f);

      obj->end();

      if (draw_under_property_->getValue().toBool()) {
        //  render under everything else
        obj->setRenderQueueGroup(Ogre::RENDER_QUEUE_4);
      }

      MapObject object;
      object.object = obj;
      object.texture = tex;
      object.material = material;
      objects_.push_back(object);
    }
  }
  scene_id_++;
}

void AerialMapDisplay::initiatedRequest(QNetworkRequest request) {
  ROS_INFO("Requesting %s", qPrintable(request.url().toString()));
}

void AerialMapDisplay::receivedImage(QNetworkRequest request) {
  ROS_INFO("Loaded tile %s", qPrintable(request.url().toString()));
}

void AerialMapDisplay::finishedLoading() {
  ROS_INFO("Finished loading all tiles.");
  dirty_ = true;
  setStatus(StatusProperty::Ok, "Message", "Loaded all tiles.");
  //  set property for resolution display
  if (loader_) {
    resolution_property_->setValue(loader_->resolution());
  }
}

void AerialMapDisplay::errorOcurred(QString description) {
  ROS_ERROR("Error: %s", qPrintable(description));
  setStatus(StatusProperty::Error, "Message", description);
}

void AerialMapDisplay::transformAerialMap() {
  Ogre::Vector3 position;
  Ogre::Quaternion orientation;

  /// @todo: For now just use the null transform
  /// Add an option for this later
  geometry_msgs::Pose pose;
  pose.orientation.w = 1;
  pose.orientation.x = 0;
  pose.orientation.y = 0;
  pose.orientation.z = 0;
  pose.position.x = 0;
  pose.position.y = 0;
  pose.position.z = 0;
  
  if (frame_.empty()) {
    frame_ = "world";
  }
  
  if (!context_->getFrameManager()->transform(frame_, ros::Time(), pose,
                                              position, orientation)) {
    ROS_DEBUG("Error transforming map '%s' from frame '%s' to frame '%s'",
              qPrintable(getName()), frame_.c_str(), qPrintable(fixed_frame_));

    setStatus(StatusProperty::Error, "Transform",
              "No transform from [" + QString::fromStdString(frame_) +
                  "] to [" + fixed_frame_ + "]");
  } else {
    setStatus(StatusProperty::Ok, "Transform", "Transform OK");
  }

  scene_node_->setPosition(position);
  scene_node_->setOrientation(orientation);
}

void AerialMapDisplay::fixedFrameChanged() { transformAerialMap(); }

void AerialMapDisplay::reset() {
  Display::reset();
  //  unsub,clear,resub
  updateTopic();
}

} // namespace rviz

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz::AerialMapDisplay, rviz::Display)
