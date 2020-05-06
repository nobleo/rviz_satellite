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

#include <functional>
#include <limits>
#include <mutex>
#include <unordered_map>
#include <utility>

#include <QImage>

#include <boost/optional.hpp>

#include "area.h"
#include "detail/tile_downloader.h"
#include "tile_id.h"

/**
 * Helper that you have to use when accessing tiles of a TileCache
 */
class TileCacheGuard
{
  std::mutex& mutex;

public:
  template <typename TileCache>
  TileCacheGuard(TileCache const& cache_) : mutex(cache_.cachedTilesLock)
  {
    mutex.lock();
  }

  ~TileCacheGuard()
  {
    mutex.unlock();
  }
};

/**
 * @brief A cache for tiles.
 *
 * The main purpose of this class is to synchronize the GPS nav fix callback with the rendering callback.
 * They both happen in parallel - therefore we have to cache the tiles when we request them until they are drawn.
 * This class provides an interface to request tiles and then access them later on (when they are drawn).
 *
 * Tiles will be either loaded from the file system (after they have been cached) or from a tile server.
 */
template <typename Tile>
class TileCache
{
  friend TileCacheGuard;
  std::unordered_map<TileId, Tile> cached_tiles;
  std::mutex mutable cachedTilesLock;
  detail::TileDownloader downloader;

  /**
   * Callback for `downloader`
   */
  void loadedTile(TileId tile_id, QImage image)
  {
    TileCacheGuard guard(*this);

    if (cached_tiles.find(tile_id) == cached_tiles.end())
    {
      cached_tiles.emplace(std::make_pair(tile_id, std::move(image)));
    }
  }

public:
  TileCache()
    : downloader([this](TileId tile_id, QImage image) { loadedTile(std::move(tile_id), std::move(image)); }){};

  /**
   * Load a rectangular area of tiles
   *
   * The requested tiles will be loaded into this cache from either the file system or an online tile server.
   */
  void request(Area const& area)
  {
    TileCacheGuard guard(*this);

    for (int x = area.left_top.x; x <= area.right_bottom.x; ++x)
    {
      for (int y = area.left_top.y; y <= area.right_bottom.y; ++y)
      {
        TileId const to_find{ area.center.tile_server, { x, y }, area.center.zoom };

        if (cached_tiles.find(to_find) == cached_tiles.end())
        {
          downloader.loadTile(to_find);
        }
      }
    }
  }

  /**
   * Is the tile @p to_find cached? If yes, return the associated Tile.
   * @note You have to use TileCacheGuard to guard this function call and the returned tile.
   */
  Tile const* ready(TileId const& to_find) const
  {
    auto const it = cached_tiles.find(to_find);

    if (it == cached_tiles.cend())
    {
      return nullptr;
    }

    return &it->second;
  }

  /**
   * Remove tiles from cache that are not in the @p area (anymore)
   * @note You have to use TileCacheGuard to guard this function call.
   */
  void purge(Area const& area)
  {
    for (auto it = cached_tiles.begin(); it != cached_tiles.end();)
    {
      if (!areaContainsTile(area, it->first))
      {
        it = cached_tiles.erase(it);
      }
      else
      {
        ++it;
      }
    }
  }

  /**
   * @brief Calculate the error rate of a tile server
   *
   * error rate = number of HTTP requests that resulted in an error / total number of HTTP requests
   */
  float getTileServerErrorRate(std::string const& tile_server) const
  {
    return downloader.error_rates.calculate(tile_server);
  }

protected:
  /**
   * Are all tiles in the area cached?
   * @note You have to use TileCacheGuard to guard this function call.
   */
  bool isAreaReady(Area const& area) const
  {
    for (int xx = area.left_top.x; xx <= area.right_bottom.x; ++xx)
    {
      for (int yy = area.left_top.y; yy <= area.right_bottom.y; ++yy)
      {
        TileId const to_find{ area.center.tile_server, { xx, yy }, area.center.zoom };

        if (cached_tiles.find(to_find) == cached_tiles.end())
        {
          return false;
        }
      }
    }
    return true;
  }
};
