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

#include <functional>
#include <limits>
#include <string>
#include <utility>

#include <QCryptographicHash>
#include <QDir>
#include <QImage>
#include <QImageReader>
#include <QStandardPaths>
#include <QString>
#include <QtCore>
#include <QtNetwork>

#include "rviz_common/logging.hpp"

#include "detail/error_rate_manager.hpp"
#include "tile_id.hpp"

/**
 * @brief Tile downloader
 *
 * This class encapsulates away all the Qt stuff regarding downloading.
 */
class TileDownloader : public QObject
{
  Q_OBJECT

  QNetworkAccessManager * manager;
  std::function<void(TileId, QImage)> callback;

public:
  detail::ErrorRateManager<std::string> error_rates;

  explicit TileDownloader(decltype(callback) callback);

  /**
   * @brief Load a specific tile
   *
   * Since QNetworkDiskCache is used, tiles will be loaded from the file system if they have been cached. Otherwise they
   * get downloaded.
   */
  void loadTile(TileId const & tile_id);

public Q_SLOTS:
  void downloadFinished(QNetworkReply * reply);
};
