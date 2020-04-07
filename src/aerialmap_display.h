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

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/display.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

// #include <OgreTexture.h>
#include <OgreMaterial.h>
#include <OgreVector3.h>

#include "ogre_tile.h"
#include "tile_cache_delay.h"

namespace Ogre
{
class ManualObject;
}

namespace rviz_common::properties
{
class FloatProperty;
class IntProperty;
class Property;
class RosTopicProperty;
class StringProperty;
}  // rviz_common::properties


namespace rviz
{
/**
 * @brief Displays a satellite map along the XY plane.
 */

class AerialMapDisplay : public rviz_common::Display
{
  Q_OBJECT
public:
  AerialMapDisplay();
  ~AerialMapDisplay() override;

  // Overrides from Display
  void onInitialize() override;
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
  void navFixCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);

  /**
   * Load images to cache (non-blocking)
   */
  void requestTileTextures();

  /**
   * Triggers texture update if the center-tile changed w.r.t. the current one
   */
  void updateCenterTile(const sensor_msgs::msg::NavSatFix::SharedPtr msg);

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
  bool getMapTransform(std::string const& query_frame, rclcpp::Time const& timestamp, Ogre::Vector3& position,
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

  rclcpp::Subscription< sensor_msgs::msg::NavSatFix >::SharedPtr navsat_fix_sub_;

  // properties
  rviz_common::properties::RosTopicProperty* topic_property_ = nullptr;
  rviz_common::properties::StringProperty* tile_url_property_ = nullptr;
  rviz_common::properties::IntProperty* zoom_property_ = nullptr;
  rviz_common::properties::IntProperty* blocks_property_ = nullptr;
  rviz_common::properties::FloatProperty* alpha_property_ = nullptr;
  rviz_common::properties::Property* draw_under_property_ = nullptr;

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
  sensor_msgs::msg::NavSatFix::SharedPtr ref_fix_{ nullptr };
  /// caches tile images, hashed by their fetch URL
  TileCacheDelay<OgreTile> tile_cache_;
  /// Last request()ed tile id (which is the center tile)
  boost::optional<TileId> center_tile_{ boost::none };
  /// translation of the center-tile w.r.t. the map frame
  Ogre::Vector3 t_centertile_map{ Ogre::Vector3::ZERO };
  /// the map frame, rigidly attached to the world with ENU convention - see https://www.ros.org/reps/rep-0105.html#map
  std::string static const MAP_FRAME;
  /// rclcpp node
  rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node_;
  /// qos
  rclcpp::QoS update_profile_ = rclcpp::SystemDefaultsQoS();
};

}  // namespace rviz
