/* Copyright 2018-2019 TomTom N.V.

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

#include <cstddef>
#include <functional>
#include <string>
#include <tuple>

#include <QMetaType>

#include "coordinates.h"

/**
 * All information to uniquely identify a tile at a tile server
 *
 * @see tileURL
 */
struct TileId
{
  std::string tile_server;
  TileCoordinate coord;
  int zoom;
};

namespace std
{
template <>
struct hash<TileId>
{
public:
  size_t operator()(TileId const& tile_id) const;
};
}  // namespace std

inline bool operator==(TileId const& self, TileId const& other)
{
  // optimization: check the tile_server last
  return std::tie(self.coord, self.zoom, self.tile_server) == std::tie(other.coord, other.zoom, other.tile_server);
}

// Make type available for QVariant
Q_DECLARE_METATYPE(TileId)

/**
 * Generate the URL to download a tile from
 */
std::string tileURL(TileId const& tile_id);
