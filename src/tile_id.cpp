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

#include "tile_id.h"

#include <boost/algorithm/string/replace.hpp>
#include <boost/functional/hash/hash.hpp>
namespace
{
/**
 * \brief Locale-independent conversion of doubles to strings.
 * \param[in] num The number.
 * \return The string.
 */
std::string toString(double num)
{
  std::stringstream ss;
  ss.imbue(std::locale::classic());
  ss.precision(6);
  ss << std::fixed << num;
  return ss.str();
}
}
std::string tileURL(TileId const& tile_id)
{
  auto url = tile_id.tile_server;  

  // compute latitude and longitude from tile coordinates
  const auto wgs = toWGSCoordinate(tile_id.coord, tile_id.zoom);

  // substitute placeholders
  boost::replace_all(url, "{lat}", toString(wgs.lat));
  boost::replace_all(url, "{lon}", toString(wgs.lon));
  boost::replace_all(url, "{x}", std::to_string(tile_id.coord.x));
  boost::replace_all(url, "{y}", std::to_string(tile_id.coord.y));
  boost::replace_all(url, "{z}", std::to_string(tile_id.zoom));
  
  return url;
}

size_t std::hash<TileId>::operator()(TileId const& tile_id) const
{
  size_t seed{};
  boost::hash_combine(seed, tile_id.tile_server);
  boost::hash_combine(seed, tile_id.coord.x);
  boost::hash_combine(seed, tile_id.coord.y);
  boost::hash_combine(seed, tile_id.zoom);
  return seed;
}
