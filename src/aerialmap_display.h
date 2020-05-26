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

#include "ogre_tile.h"
#include "tile_cache_delay.h"

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

  /**
   * Triggers texture update if the center-tile changed w.r.t. the current one
   */
  void updateCenterTile(sensor_msgs::NavSatFixConstPtr const& msg);

  /**
   * Generates the tile's render geometry and applies the requested textures
   */
  void assembleScene();

  /**
   * Triggers to (re-) assemble the scene
   */
  void triggerSceneAssembly();

  /**
   * Destroys the scene-object and all children and their textures, along with the center-tile memory
   */
  void clearAll();

  /**
   * Destroys the tile scene-objects
   */
  void destroyTileObjects();

  /**
   * Creates the tile scene-objects and their materials
   */
  void createTileObjects();

  /**
   * Transforms the tile objects into the map frame.
   */
  void transformTileToMapFrame();

  /**
   * Transforms the tile objects into the fixed frame.
   */
  void transformMapTileToFixedFrame();

  /**
   * Get the transform from frame_id w.r.t. the map-frame
   *
   * @return true if the transform lookup was successful
   * @return false if the transform lookup failed
   */
  bool getMapTransform(std::string const& query_frame, ros::Time const& timestamp, Ogre::Vector3& position,
                       Ogre::Quaternion& orientation, std::string& error);

  /**
   * Checks how may tiles were loaded successfully, and sets the status accordingly.
   */
  void checkRequestErrorRate();

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

  /// the tile scene objects
  std::vector<MapObject> objects_;

  /// the subscriber for the NavSatFix topic
  ros::Subscriber navsat_fix_sub_;

  // properties
  RosTopicProperty* topic_property_;
  StringProperty* tile_url_property_;
  IntProperty* zoom_property_;
  IntProperty* blocks_property_;
  FloatProperty* alpha_property_;
  Property* draw_under_property_;

  /// the alpha value of the tile's material
  float alpha_;
  /// determines which render queue to use
  bool draw_under_;
  /// the URL of the tile server to use
  std::string tile_url_;
  /// the zoom to use (Mercator)
  int zoom_;
  /// the number of tiles loaded in each direction around the center tile
  int blocks_;

  // tile management
  /// whether we need to re-query and re-assemble the tiles
  bool dirty_{ false };
  /// the last NavSatFix message that lead to updating the tiles
  sensor_msgs::NavSatFixConstPtr ref_fix_{ nullptr };
  /// caches tile images, hashed by their fetch URL
  TileCacheDelay<OgreTile> tile_cache_;
  /// Last request()ed tile id (which is the center tile)
  boost::optional<TileId> center_tile_{ boost::none };
  /// translation of the center-tile w.r.t. the map frame
  Ogre::Vector3 t_centertile_map{ Ogre::Vector3::ZERO };
  /// the map frame, rigidly attached to the world with ENU convention - see https://www.ros.org/reps/rep-0105.html#map
  std::string static const MAP_FRAME;
};

}  // namespace rviz
