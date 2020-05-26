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

#include <functional>

#include <QCryptographicHash>
#include <QDir>
#include <QImage>
#include <QImageReader>
#include <QStandardPaths>
#include <QString>
#include <QtCore>
#include <QtNetwork>

#include <ros/ros.h>

#include "detail/error_rate_manager.h"
#include "tile_id.h"

namespace detail
{
/**
 * @brief Tile downloader
 *
 * This class encapsulates away all the Qt stuff regarding downloading.
 */
class TileDownloader : public QObject
{
  Q_OBJECT
  QNetworkAccessManager* manager;
  std::function<void(TileId, QImage)> callback;

public:
  detail::ErrorRateManager<std::string> error_rates;

  TileDownloader(decltype(callback) callback) : manager(new QNetworkAccessManager(this)), callback(std::move(callback))
  {
    connect(manager, SIGNAL(finished(QNetworkReply*)), SLOT(downloadFinished(QNetworkReply*)));

    QNetworkDiskCache* disk_cache = new QNetworkDiskCache(this);
    QString const cache_path =
        QDir(QStandardPaths::writableLocation(QStandardPaths::GenericCacheLocation)).filePath("rviz_satellite");
    disk_cache->setCacheDirectory(cache_path);
    // there is no option to disable maximum cache size
    disk_cache->setMaximumCacheSize(std::numeric_limits<qint64>::max());
    manager->setCache(disk_cache);
  }

  /**
   * @brief Load a specific tile
   *
   * Since QNetworkDiskCache is used, tiles will be loaded from the file system if they have been cached. Otherwise they
   * get downloaded.
   */
  void loadTile(TileId const& tile_id)
  {
    // see https://foundation.wikimedia.org/wiki/Maps_Terms_of_Use#Using_maps_in_third-party_services
    auto const request_url = QUrl(QString::fromStdString(tileURL(tile_id)));
    ROS_DEBUG_STREAM_NAMED("rviz_satellite", "Loading tile " << request_url.toString().toStdString());

    QNetworkRequest request(request_url);
    char constexpr agent[] = "rviz_satellite/" RVIZ_SATELLITE_VERSION " (+https://github.com/gareth-cross/"
                             "rviz_satellite)";
    request.setHeader(QNetworkRequest::KnownHeaders::UserAgentHeader, agent);
    QVariant variant;
    variant.setValue(tile_id);
    request.setAttribute(QNetworkRequest::CacheLoadControlAttribute, QNetworkRequest::CacheLoadControl::PreferCache);
    request.setAttribute(QNetworkRequest::User, variant);
    manager->get(request);
  }

public slots:
  void downloadFinished(QNetworkReply* reply)
  {
    QVariant const variant = reply->request().attribute(QNetworkRequest::User);
    TileId const tile_id = variant.value<TileId>();

    QUrl const url = reply->url();
    if (reply->error())
    {
      ROS_ERROR_STREAM("Got error when loading tile: " << reply->errorString().toStdString());
      error_rates.issueError(tile_id.tile_server);
      return;
    }
    else
    {
      error_rates.issueSuccess(tile_id.tile_server);
    }

    // log if tile comes from cache or web
    bool const from_cache = reply->attribute(QNetworkRequest::SourceIsFromCacheAttribute).toBool();
    if (from_cache)
    {
      ROS_DEBUG_STREAM_NAMED("rviz_satellite", "Loaded tile from cache " << url.toString().toStdString());
    }
    else
    {
      ROS_DEBUG_STREAM_NAMED("rviz_satellite", "Loaded tile from web " << url.toString().toStdString());
    }

    QImageReader reader(reply);
    if (!reader.canRead())
    {
      ROS_ERROR_STREAM("Unable to decode image at " << reply->request().url().toString().toLatin1().data());
      return;
    }

    callback(tile_id, reader.read());

    reply->deleteLater();
  }
};

}  // namespace detail
