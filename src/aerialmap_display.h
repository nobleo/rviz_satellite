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

#include <string>
#include <vector>

#include <boost/optional.hpp>

#include <ros/ros.h>
#include <ros/time.h>
#include <rviz/display.h>
#include <sensor_msgs/NavSatFix.h>

#include <OGRE/OgreMaterial.h>
#include <OGRE/OgreVector3.h>

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
  void reset() override;
  void update(float, float) override;

protected Q_SLOTS:
  void updateAlpha();
  void updateTopic();
  void updateDrawUnder();
  void updateTileUrl();
  void updateZoom();
  void updateBlocks();

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
  void requestTileTextures();
  void updateCenterTile(sensor_msgs::NavSatFixConstPtr const& msg);

  /**
   * Create geometry
   */
  void assembleScene();

  void clearAll();
  void destroyTileObjects();
  void createTileObjects();

  /**
   * @brief Transforms the tile objects into the map frame.
   */
  void transformTileToMapFrame();

  /**
   * @brief Transforms the tile objects into the fixed frame.
   */
  void transformMapTileToFixedFrame();

  /**
   * @brief Get the transform from frame_id w.r.t. the map-frame
   *
   * @return true if the transform lookup was successful
   * @return false if the transform lookup failed
   */
  bool getMapTransform(std::string const& query_frame, ros::Time const& timestamp, Ogre::Vector3& position,
                       Ogre::Quaternion& orientation, std::string& error);

  /**
   * @brief Checks how may tiles were loaded successfully, and sets the status accordingly.
   */
  void checkRequestErrorRate();

  /**
   * Calculate the tile width/ height in meter
   */
  double getTileWH(double const latitude, int const zoom) const;

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
  FloatProperty* alpha_property_;
  Property* draw_under_property_;

  float alpha_;
  bool draw_under_;
  std::string tile_url_;
  int zoom_;
  int blocks_;

  // tile management
  /// whether we need to re-query and re-assemble the tiles
  bool dirty_;
  /// the last NavSatFix message that lead to updating the tiles
  sensor_msgs::NavSatFixConstPtr ref_fix_{ nullptr };
  /// caches tile images, hashed by their fetch URL
  TileCacheDelay<OgreTile> tileCache_;
  /// Last request()ed tile id (which is the center tile)
  boost::optional<TileId> lastCenterTile_;
  /// translation of the center-tile w.r.t. the map frame
  Ogre::Vector3 t_centertile_map{ Ogre::Vector3::ZERO };
  /// the map frame, rigidly attached to the world with ENU convention - see https://www.ros.org/reps/rep-0105.html#map
  std::string static const MAP_FRAME;
};

}  // namespace rviz
