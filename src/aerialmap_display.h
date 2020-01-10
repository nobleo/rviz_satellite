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

#pragma once

// NOTE: workaround for issue: https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <ros/time.h>
#include <rviz/display.h>
#include <sensor_msgs/NavSatFix.h>

#include <OGRE/OgreTexture.h>
#include <OGRE/OgreMaterial.h>
#endif  //  Q_MOC_RUN

#include <QObject>
#include <QtConcurrentRun>
#include <QFuture>
#include <utility>
#include <boost/optional.hpp>
#include <QByteArray>
#include <QFile>
#include <QNetworkRequest>

#include <string>
#include <vector>
#include <memory>
#include "TileCacheDelay.h"
#include "OgreTile.h"

namespace Ogre
{
class ManualObject;
}

namespace rviz
{
class FloatProperty;
class IntProperty;
class Property;
class RosTopicProperty;
class StringProperty;
class TfFrameProperty;
class EnumProperty;

/**
 * @class AerialMapDisplay
 * @brief Displays a satellite map along the XY plane.
 */
class AerialMapDisplay : public Display
{
  Q_OBJECT
public:
  AerialMapDisplay();
  ~AerialMapDisplay() override;

  // Overrides from Display
  void fixedFrameChanged() override;
  void reset() override;
  void update(float, float) override;

protected Q_SLOTS:
  void updateAlpha();
  void updateTopic();
  void updateDrawUnder();
  void updateTileUrl();
  void updateZoom();
  void updateBlocks();
  void updateFrameConvention();

protected:
  // overrides from Display
  void onEnable() override;
  void onDisable() override;

  virtual void subscribe();
  virtual void unsubscribe();

  /**
   * GPS topic callback
   */
  void navFixCallback(sensor_msgs::NavSatFixConstPtr const& msg);

  /**
   * Load images to cache (non-blocking)
   */
  void loadImagery();

  /**
   * Create geometry
   */
  void assembleScene();

  void clear();
  void clearGeometry();
  void createGeometry();

  // Convienece wrapper around FrameManager::getTransform
  bool getTransform(const std::string& frame, ros::Time time, Ogre::Vector3& position, Ogre::Quaternion& orientation);

  // Convienece wrapper around FrameManager::getTransform to only get the
  // position
  boost::optional<Ogre::Vector3> getPosition(const std::string& frame, ros::Time time);

  // Convienece wrapper around FrameManager::getTransform to only get the
  // orientation
  boost::optional<Ogre::Quaternion> getOrientation(const std::string& frame, ros::Time time);

  /**
   * @brief Transforms the tiles into the fixed frame.
   */
  void transformAerialMap();

  /**
   * Tile with associated Ogre data
   */
  struct MapObject
  {
    Ogre::ManualObject* object;
    Ogre::MaterialPtr material;

    MapObject(Ogre::ManualObject* o, Ogre::MaterialPtr m) : object(o), material(m)
    {
      assert(!material.isNull());
    }
  };
  std::vector<MapObject> objects_;

  ros::Subscriber coord_sub_;

  // properties
  RosTopicProperty* topic_property_;
  StringProperty* tile_url_property_;
  IntProperty* zoom_property_;
  IntProperty* blocks_property_;
  FloatProperty* resolution_property_;
  FloatProperty* alpha_property_;
  Property* draw_under_property_;
  EnumProperty* frame_convention_property_;

  float alpha_;
  bool draw_under_;
  std::string tile_url_;
  int zoom_;
  int blocks_;

  // tile management
  bool dirty_;
  bool received_msg_;
  sensor_msgs::NavSatFix ref_fix_;
  TileCacheDelay<OgreTile> tileCache_;
  /// Last request()ed tile id (which is the center tile)
  boost::optional<TileId> lastCenterTile_;

  /**
   * Calculate the tile width/ height in meter
   */
  double getTileWH()
  {
    // tile width/ height in pixel
    // according to https://wiki.openstreetmap.org/wiki/Slippy_map_tilenames
    int constexpr tile_w_h_px = 256;

    auto const resolution = zoomToResolution(ref_fix_.latitude, zoom_);
    double const tile_w_h_m = tile_w_h_px * resolution;
    return tile_w_h_m;
  }
};

}  // namespace rviz
