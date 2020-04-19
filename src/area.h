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
#include <tuple>
#include <utility>

#include "coordinates.h"
#include "mercator.h"
#include "tile_id.h"

/**
 * A square area around a specific tile.
 *
 * @see areaContainsTile
 */
struct Area
{
  TileCoordinate left_top;
  TileCoordinate right_bottom;
  TileId center;

  /**
   * @brief Creates a square area with the center @p center and the radius @p blocks
   * @param center Center of the area
   * @param blocks Radius around @p center
   */
  Area(TileId center, int blocks) : center(std::move(center))
  {
    if (blocks < 0)
    {
      throw std::invalid_argument("The number of blocks has to be positive");
    }

    TileCoordinate const& center_tile = center.coord;

    // An Area is defined through its left top and right bottom coordinates. These points will be calculated.
    // Furthermore, constraints from the documentation https://wiki.openstreetmap.org/wiki/Slippy_map_tilenames#X_and_Y
    // are satisfied.
    int const min_x = std::max(0, center_tile.x - blocks);
    int const min_y = std::max(0, center_tile.y - blocks);
    int const max_x = std::min(zoomToMaxTiles(center.zoom), center_tile.x + blocks);
    int const max_y = std::min(zoomToMaxTiles(center.zoom), center_tile.y + blocks);

    left_top = { min_x, min_y };
    right_bottom = { max_x, max_y };
  }
};

inline bool operator==(Area const& self, Area const& other)
{
  return std::tie(self.left_top, self.right_bottom, self.center) ==
         std::tie(other.left_top, other.right_bottom, other.center);
}

/**
 * Is a TileId @p needle inside an Area @p haystack?
 *
 * @note Two areas on different tile servers always compare unequal.
 */
inline bool areaContainsTile(Area const& haystack, TileId const& needle)
{
  bool const in_area = needle.coord >= haystack.left_top && needle.coord <= haystack.right_bottom;
  bool const corresponds = haystack.center.tile_server == needle.tile_server && haystack.center.zoom == needle.zoom;
  return in_area && corresponds;
}
