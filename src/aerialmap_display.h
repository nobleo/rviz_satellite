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
  void onInitialize() override;
  void fixedFrameChanged() override;
  void reset() override;
  void update(float, float) override;

protected Q_SLOTS:
  void updateAlpha();
  void updateTopic();
  void updateFrame();
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

  /**
   * @brief Transforms from the robot's frame to the fixed frame.
   *
   * There are four relevant frames: The robot's frame, Rviz's fixed frame, the ENU/ NED/ NWU world fixed frame, and
   * the tile frame.
   *
   * * The robot's frame is a frame rigidly attached to the robot. This frame is configurable in the plugin's GUI.
   * * The fixed frame is set in Rviz by the user.
   * * The ENU/ NED/ NWU world fixed frame is assumed to be called "map". This name is standardized, see
   * http://www.ros.org/reps/rep-0105.html The map frame can be either ENU, NED, or NWU. The code only uses ENU
   * internally and later rotates the frame into NED or NWU accordingly.
   * * We assume that the tiles are in a frame where x points eastwards and y southwards. This frame is used by
   * OSM and Google Maps, see https://developers.google.com/maps/documentation/javascript/coordinates
   *
   * Since the code works with ENU internally, the tiles' y-coordinate is flipped so that y points to north. Therefore
   * we can align the tiles to north by putting them into the frame "map" with an orientation of
   * Ogre::Quaternion::IDENTITY. We place the tiles indirectly into the "map" frame by using the "map" frame as the
   * pseudo fixed frame.
   *
   * Thus we will end up with the tile mesh being aligned to the north regardless of which fixed frame is chosen.
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
  TfFrameProperty* frame_property_;
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
  /// Last request()ed tile id
  boost::optional<TileId> lastTileId_;
  std::string lastFixedFrame_;

  bool hasWorkingTransform_ = false;

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
