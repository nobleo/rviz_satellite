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

#include <vector>

#include <QTimer>

#include "tile_cache.h"

template <typename Tile>
class TileCacheDelay;

namespace detail
{
/**
 * An ExpiringArea is an Area that gets ready after at most 2s
 *
 * Its only purpose is to be stored inside a AreaHistory.
 */
struct ExpiringArea
{
  QTimer timer;
  Area area;

  ExpiringArea(Area area) : area(std::move(area))
  {
    timer.setSingleShot(true);
    int constexpr timeout = 2000;  // in ms
    timer.start(timeout);
  }

  ExpiringArea(ExpiringArea&&) = default;
  ExpiringArea(ExpiringArea const& p) : area(p.area)
  {
    *this = p;
  }

  /**
   * Is this Area ready to be displayed?
   *
   * Ready means that either the whole area is fully loaded or that the area expired.
   */
  template <typename Tile>
  bool ready(TileCacheDelay<Tile> const& cache) const
  {
    if (!timer.isActive())
    {
      return true;
    }

    return cache.isAreaReady(area);
  }

  ExpiringArea& operator=(ExpiringArea&&) = default;

  ExpiringArea& operator=(ExpiringArea const& p)
  {
    area = p.area;

    // QTimer has no copy operator
    if (p.timer.isActive())
    {
      timer.start(p.timer.remainingTime());
    }
    return *this;
  }
};

/**
 * A record of Areas that were visited by the robot.
 */
class AreaHistory
{
  /// History of areas that will be or were drawn in Rviz
  std::vector<ExpiringArea> history_;

public:
  /**
   * Remove all Areas that aren't in @p area
   */
  void fit(Area const& area)
  {
    history_.erase(std::remove_if(history_.begin(), history_.end(),
                                  [&area](ExpiringArea const& p) { return !areaContainsTile(p.area, area.center); }),
                   history_.end());
  }

  /**
   * Add a new Area to the history.
   *
   * @param area Area to add to the history.
   */
  void add(Area const& area)
  {
    auto const it =
        std::find_if(history_.begin(), history_.end(), [&area](ExpiringArea const& p) { return p.area == area; });
    if (it == history_.end())
    {
      history_.emplace_back(area);
    }
  };

  /**
   * Is the tile in at least one Area that is ready?
   */
  template <typename Tile>
  bool ready(TileCacheDelay<Tile> const& cache, TileId const& to_find) const
  {
    return std::any_of(history_.begin(), history_.end(), [&to_find, &cache](ExpiringArea const& ea) {
      return areaContainsTile(ea.area, to_find) && ea.ready(cache);
    });
  }
};
}  // namespace detail

/**
 * TileCacheDelay is like TileCache but gets (mostly) rid of the effect that the tiles are drawn one-by-one by delaying
 * the tiles. This class is just a cosmetic improvement - it doesn't intent to improve performance.
 *
 * This is done by trying to load a whole square instead of just a single tile. The exact algorithm is as follows:
 *
 * Every time the robot enters a new center tile, request() is called. This functions stores the requested
 * detail::ExpiringArea, where a ExpiringArea is just an Area at a tile id with a timeout. Therefore we will collect a
 * history of Areas (detail::AreaHistory) while the robot moves.
 *
 * A tile will be drawn if it's ready(). A tile is ready() iff:
 * * it's successfully loaded from the file system or the internet, and
 * * either of the following things happened:
 *   * the tile is inside an Area which has all its tiles successfully loaded, or
 *   * the tile is inside an Area which expired, i.e. the Area was requested some seconds ago but not all tiles are
 *     loaded.
 *
 * @warning Since TileCache isn't a virtual class, you shouldn't upcast a TileCacheDelay object to a TileCache object!
 */
template <typename Tile>
class TileCacheDelay : public TileCache<Tile>
{
  detail::AreaHistory history_;
  friend detail::ExpiringArea;

public:
  void request(Area const& area)
  {
    TileCache<Tile>::request(area);

    history_.fit(area);
    history_.add(area);
  }

  /**
   * @note You have to use TileCacheGuard to guard this function call and the returned tile.
   */
  Tile const* ready(TileId const& to_find) const
  {
    Tile const* tile = TileCache<Tile>::ready(to_find);
    if (tile && history_.ready(*this, to_find))
    {
      return tile;
    }

    return nullptr;
  }
};
