/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef AERIAL_MAP_DISPLAY_H
#define AERIAL_MAP_DISPLAY_H

#include <ros/ros.h>
#include <ros/time.h>
#include <rviz/display.h>

#include <OGRE/OgreTexture.h>
#include <OGRE/OgreMaterial.h>
#include <OGRE/OgreVector3.h>

#include <sensor_msgs/NavSatFix.h>

#include <QObject>
#include <QtConcurrentRun>
#include <QFuture>
#include <QByteArray>
#include <QFile>
#include <QNetworkRequest>

#include <tileloader.h>

namespace Ogre {
class ManualObject;
}

namespace rviz {

class FloatProperty;
class IntProperty;
class Property;
class RosTopicProperty;
class StringProperty;

/**
 * \class AerialMapDisplay
 * \brief Displays a satellite map along the XY plane.
 */
class AerialMapDisplay : public Display {
  Q_OBJECT
public:
  AerialMapDisplay();
  virtual ~AerialMapDisplay();

  // Overrides from Display
  virtual void onInitialize();
  virtual void fixedFrameChanged();
  virtual void reset();
  virtual void update(float, float);

protected Q_SLOTS:
  void updateAlpha();
  void updateTopic();
  void updateDrawUnder();
  void updateObjectURI();
  void updateZoom();
  void updateBlocks();

  //  slots for TileLoader messages
  void initiatedRequest(QNetworkRequest request);
  void receivedImage(QNetworkRequest request);
  void finishedLoading();
  void errorOcurred(QString description);

protected:
  // overrides from Display
  virtual void onEnable();
  virtual void onDisable();

  virtual void subscribe();
  virtual void unsubscribe();

  void navFixCallback(const sensor_msgs::NavSatFixConstPtr &msg);

  void loadImagery();

  void assembleScene();

  void clear();

  void transformAerialMap();

  unsigned int map_id_;
  unsigned int scene_id_;

  /// Instance of a tile w/ associated ogre data
  struct MapObject {
    Ogre::ManualObject *object;
    Ogre::TexturePtr texture;
    Ogre::MaterialPtr material;
  };
  std::vector<MapObject> objects_;
  bool loaded_;

  std::string topic_;
  std::string frame_;

  ros::Subscriber coord_sub_;

  //  properties
  RosTopicProperty *topic_property_;
  StringProperty *object_uri_property_;
  IntProperty *zoom_property_;
  IntProperty *blocks_property_;
  FloatProperty *resolution_property_;
  FloatProperty *alpha_property_;
  Property *draw_under_property_;

  float alpha_;
  bool draw_under_;
  std::string object_uri_;
  unsigned int zoom_;
  unsigned int blocks_;

  //  tile management
  boost::mutex mutex_;
  bool new_coords_;
  bool received_msg_;
  double ref_lat_;
  double ref_lon_;
  TileLoader *loader_;
};

} // namespace rviz

#endif
