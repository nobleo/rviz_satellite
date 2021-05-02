/* Copyright 2018-2019 TomTom N.V., 2014 Gareth Cross

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

#include <cmath>
#include <stdexcept>
#include <string>
#include <tuple>
#include <Ogre.h>

#include <QMetaType>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

namespace rviz_satellite
{

/**
 * A coordinate for identifying the position of a tile at a given zoom level
 *
 * "Tile coordinates" use the coordinate system explained in
 * https://wiki.openstreetmap.org/wiki/Slippy_map_tilenames#X_and_Y
 */
struct TileCoordinate
{
  int x, y, z;

  bool operator==(const TileCoordinate & other) const
  {
    return std::tie(x, y, z) == std::tie(other.x, other.y, other.z);
  }

  bool operator!=(const TileCoordinate & other) const
  {
    return std::tie(x, y, z) != std::tie(other.x, other.y, other.z);
  }

  bool operator<(const TileCoordinate & other) const
  {
    return std::tie(x, y, z) < std::tie(other.x, other.y, other.z);
  }
};

/**
 * Unique identifier of a map tile at a tile server
 *
 * @see tileURL
 */
struct TileId
{
  std::string server_url;
  TileCoordinate coord;

  bool operator==(const TileId & other) const
  {
    return std::tie(coord, server_url) == std::tie(other.coord, other.server_url);
  }

  bool operator<(const TileId & other) const
  {
    return std::tie(coord, server_url) < std::tie(other.coord, other.server_url);
  }
};

/// Max number of adjacent blocks to support.
static constexpr int MAX_BLOCKS = 8;

/// Max zoom level to support.
static constexpr int MAX_ZOOM = 22;

/**
 * Compute length of tile in meters at given latitude and zoom level.
 *
 * @see https://wiki.openstreetmap.org/wiki/Slippy_map_tilenames#Resolution_and_Scale
 */
double zoomSize(double lat, int zoom);

std::string tileURL(const TileId & tile_id);

/**
 * Convert WGS coordinate to a tile coordinate using the Mercator projection.
 */
TileCoordinate fromWGS(const sensor_msgs::msg::NavSatFix &, int zoom);

/**
 * Get the relative offset (-0.5, 0.5) of the geo point to the center of the tile
 */
Ogre::Vector2 tileOffset(const sensor_msgs::msg::NavSatFix &, int zoom);

}  // namespace rviz_satellite

std::ostream & operator<<(std::ostream & os, const rviz_satellite::TileId & tile_id);

// Make type available for QVariant, which is required to use it as network request data
Q_DECLARE_METATYPE(rviz_satellite::TileId)
