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
#include <GeographicLib/UTMUPS.hpp>
#include "rviz_common/logging.hpp"
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
 * 2D vector struct with double precision
 * Ogre::Vector2 does not provide double but float precision
 * which leads to numerical errors computing the tile offset
 */
struct Vector2Double {
  const double x;
  const double y;
};

/**
 * @see https://wiki.openstreetmap.org/wiki/Slippy_map_tilenames for explanation of these calculations.
 */
Vector2Double computeTileCoordinate(const sensor_msgs::msg::NavSatFix & point, int zoom)
{
  if (zoom > MAX_ZOOM) {
    throw std::invalid_argument("Zoom level " + std::to_string(zoom) + " too high");
  } else if (point.latitude < -85.0511 || point.latitude > 85.0511) {
    throw std::invalid_argument("Latitude " + std::to_string(point.latitude) + " invalid");
  } else if (point.longitude < -180 || point.longitude > 180) {
    throw std::invalid_argument("Longitude " + std::to_string(point.longitude) + " invalid");
  }

  // convert lat lon to utm coordinates
  double utm_x, utm_y;
  int zone;
  bool northp;
  GeographicLib::UTMUPS::Forward(point.latitude, point.longitude, zone, northp, utm_x, utm_y);
  
  // according to : OpenGIS® Web Map Tile Service Implementation Standard, page 8-9
  // and WMTSCapabilities.xml from NRW DOP
  const double scale_denominator_at_0 = 17471320.750897426;
  const double tile_width_height = 256.0;
  const double pixel_size = 0.00028;

  const double tile_matrix_x_min = -46133.17;
  const double tile_matrix_y_max = 6301219.54;

  double scale_denominator = scale_denominator_at_0 / (1 << zoom);
  double pixel_span = scale_denominator * pixel_size;
  double tile_span = tile_width_height * pixel_span;

  // according to : OpenGIS® Web Map Tile Service Implementation Standard, Annex H
  const double x = (utm_x - tile_matrix_x_min) / tile_span;
  const double y = (tile_matrix_y_max - utm_y) / tile_span;
  
  RVIZ_COMMON_LOG_WARNING_STREAM("Latitude: " << point.latitude << " Longitude: " << point.longitude);
  RVIZ_COMMON_LOG_WARNING_STREAM("UTM X: " << utm_x << " UTM Y: " << utm_y);
  RVIZ_COMMON_LOG_WARNING_STREAM("X: " << x << " Y: " << y);

  return {x, y};
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
