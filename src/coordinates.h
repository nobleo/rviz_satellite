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

#include "mercator.h"

/**
 * A WGS coordinate consisting of a latitude and longitude.
 */
struct WGSCoordinate
{
  double lat, lon;
};

/**
 * A coordinate for identifying the position of a tile at a given zoom level
 *
 * "Tile coordinates" use the coordinate system explained in
 * https://wiki.openstreetmap.org/wiki/Slippy_map_tilenames#X_and_Y
 *
 * @note Please use the non-generic TileCoordinate unless you have a specific reason not to.
 */
template <typename NumericType = int>
struct TileCoordinateGeneric
{
  NumericType x, y;
};

/**
 * Convert lat/ lon to a tile coordinate with the Mercator projection.
 *
 * @see https://wiki.openstreetmap.org/wiki/Slippy_map_tilenames for explanation of these calculations.
 */
template <typename NumericType = int>
TileCoordinateGeneric<NumericType> fromWGSCoordinate(WGSCoordinate coord, int zoom)
{
  if (zoom > MAX_ZOOM)
  {
    throw std::invalid_argument("Zoom level " + std::to_string(zoom) + " too high");
  }
  else if (coord.lat < -85.0511 || coord.lat > 85.0511)
  {
    throw std::invalid_argument("Latitude " + std::to_string(coord.lat) + " invalid");
  }
  else if (coord.lon < -180 || coord.lon > 180)
  {
    throw std::invalid_argument("Longitude " + std::to_string(coord.lon) + " invalid");
  }

  double constexpr rho = M_PI / 180;
  double const lat_rad = coord.lat * rho;

  TileCoordinateGeneric<NumericType> ret;
  int const n = 1 << zoom;
  ret.x = n * ((coord.lon + 180) / 360.0);
  ret.y = n * (1 - (std::log(std::tan(lat_rad) + 1 / std::cos(lat_rad)) / M_PI)) / 2;
  return ret;
}

template <typename NumericType>
bool operator==(TileCoordinateGeneric<NumericType> self, TileCoordinateGeneric<NumericType> other)
{
  return std::tie(self.x, self.y) == std::tie(other.x, other.y);
}
template <typename NumericType>
bool operator>=(TileCoordinateGeneric<NumericType> self, TileCoordinateGeneric<NumericType> other)
{
  return std::tie(self.x, self.y) >= std::tie(other.x, other.y);
}
template <typename NumericType>
bool operator<=(TileCoordinateGeneric<NumericType> self, TileCoordinateGeneric<NumericType> other)
{
  return std::tie(self.x, self.y) <= std::tie(other.x, other.y);
}

using TileCoordinate = TileCoordinateGeneric<int>;
