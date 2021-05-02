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
#include "tile.hpp"
#include <angles/angles.h>
#include <string>
#include <rcpputils/find_and_replace.hpp>

namespace rviz_satellite
{

double zoomSize(double lat, int zoom)
{
  double constexpr METER_PER_PIXEL_ZOOM_0 = 156543.034;
  return METER_PER_PIXEL_ZOOM_0 * 256 * std::cos(angles::from_degrees(lat)) / (1 << zoom);
}

std::string tileURL(const TileId & tile_id)
{
  auto url = tile_id.server_url;
  url = rcpputils::find_and_replace(url, "{x}", std::to_string(tile_id.coord.x));
  url = rcpputils::find_and_replace(url, "{y}", std::to_string(tile_id.coord.y));
  url = rcpputils::find_and_replace(url, "{z}", std::to_string(tile_id.coord.z));
  return url;
}

/**
 * @see https://wiki.openstreetmap.org/wiki/Slippy_map_tilenames for explanation of these calculations.
 */
Ogre::Vector2 computeTileCoordinate(const sensor_msgs::msg::NavSatFix & point, int zoom)
{
  if (zoom > MAX_ZOOM) {
    throw std::invalid_argument("Zoom level " + std::to_string(zoom) + " too high");
  } else if (point.latitude < -85.0511 || point.latitude > 85.0511) {
    throw std::invalid_argument("Latitude " + std::to_string(point.latitude) + " invalid");
  } else if (point.longitude < -180 || point.longitude > 180) {
    throw std::invalid_argument("Longitude " + std::to_string(point.longitude) + " invalid");
  }

  const double lat = angles::from_degrees(point.latitude);
  const int n = 1 << zoom;
  const double x = n * ((point.longitude + 180) / 360.0);
  const double y = n * (1 - (std::log(std::tan(lat) + 1 / std::cos(lat)) / M_PI)) / 2;
  return Ogre::Vector2(x, y);
}

TileCoordinate fromWGS(const sensor_msgs::msg::NavSatFix & point, int zoom)
{
  auto coord = computeTileCoordinate(point, zoom);
  return TileCoordinate {
    static_cast<int>(coord.x),
    static_cast<int>(coord.y),
    zoom,
  };
}

/**
 * Compute the relative offset (-0.5, 0.5) of the WGS coordinate to the center of its tile.
 */
Ogre::Vector2 tileOffset(const sensor_msgs::msg::NavSatFix & point, int zoom)
{
  auto coord = computeTileCoordinate(point, zoom);
  return Ogre::Vector2(coord.x - floor(coord.x) - 0.5, coord.y - floor(coord.y) - 0.5);
}

}  // namespace rviz_satellite

std::ostream & operator<<(std::ostream & os, const rviz_satellite::TileId & tile_id)
{
  os << rviz_satellite::tileURL(tile_id);
  return os;
}
