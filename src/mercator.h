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
static constexpr int MAX_BLOCKS = 8;

/// Max zoom level to support.
static constexpr int MAX_ZOOM = 22;

/**
 * Convert latitude and zoom level to ground resolution.
 * Resolution is how many meters per pixel are covered by a tile.
 *
 * @see https://wiki.openstreetmap.org/wiki/Slippy_map_tilenames#Resolution_and_Scale
 */
inline float zoomToResolution(double lat, int zoom)
{
  float constexpr METER_PER_PIXEL_ZOOM_0 = 156543.034;

  float const lat_rad = lat * M_PI / 180;
  return METER_PER_PIXEL_ZOOM_0 * std::cos(lat_rad) / (1 << zoom);
}

/**
 * Maximum number of tiles for the zoom level in one direction
 */
inline int zoomToMaxTiles(int zoom)
{
  return (1 << zoom) - 1;
}

/**
 * Calculate the tile width/height in meter
 */
inline double getTileWH(double const latitude, int const zoom)
{
  // this constant origins from how the base resolution is calculated
  // see https://wiki.openstreetmap.org/wiki/Slippy_map_tilenames

  // TODO: actually this not needed and could be removed from both formulas, since they cancel out each other;
  // it origins from most tile map applications directly rendering images with pixel dimensions;
  // in here we have OpenGL, pixel do not matter, only meters
  int constexpr tile_w_h_px = 256;

  // meter/pixel
  auto const resolution = zoomToResolution(latitude, zoom);
  // gives tile size (with and height) in meter
  double const tile_w_h_m = tile_w_h_px * resolution;

  return tile_w_h_m;
}
