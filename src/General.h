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

/// Max number of adjacent blocks to support.
static constexpr int maxBlocks = 8;

/// Max zoom level to support.
static constexpr int maxZoom = 22;

/**
 * Convert latitude and zoom level to ground resolution.
 * Resolution is how many meters per pixel are covered by a tile.
 *
 * @see https://wiki.openstreetmap.org/wiki/Slippy_map_tilenames#Resolution_and_Scale
 */
inline float zoomToResolution(double lat, int zoom)
{
  float constexpr metersPerPixelZoom0 = 156543.034;

  float const latRad = lat * M_PI / 180;
  return metersPerPixelZoom0 * std::cos(latRad) / (1 << zoom);
}

/**
 * Maximum number of tiles for the zoom level in one direction
 */
inline int zoomToMaxTiles(int zoom)
{
  return (1 << zoom) - 1;
}
