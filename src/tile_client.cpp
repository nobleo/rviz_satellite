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

#include <chrono>
#include <QImage>
#include <QImageReader>
#include <QStandardPaths>
#include <QString>
#include <QtCore>
#include <QtNetwork>
#include <utility>
#include <regex>

#include "rviz_common/logging.hpp"
#include "tile_client.hpp"

namespace rviz_satellite
{

TileClient::TileClient()
: manager_(new QNetworkAccessManager(this)), tile_promises_()
{
  connect(manager_, SIGNAL(finished(QNetworkReply*)), SLOT(request_finished(QNetworkReply*)));
  QNetworkDiskCache * disk_cache = new QNetworkDiskCache(this);
  QString const cache_path =
    QDir(QStandardPaths::writableLocation(QStandardPaths::GenericCacheLocation)).filePath(
    "rviz_satellite");
  disk_cache->setCacheDirectory(cache_path);
  manager_->setCache(disk_cache);
}

/**
 * @brief Request a specific tile
 *
 * Since QNetworkDiskCache is used, tiles will be loaded from the file system if they have been cached.
 * Otherwise they are fetched from the tile server given in the @p tile_id.
 */
std::future<QImage> TileClient::request(TileId const & tile_id)
{
  if (tile_id.server_url.find("file://") != std::string::npos) {
    return request_local(tile_id);
  } else {
    return request_remote(tile_id);
  }
}

std::future<QImage> TileClient::request_remote(TileId const & tile_id)
{
  // see https://foundation.wikimedia.org/wiki/Maps_Terms_of_Use#Using_maps_in_third-party_services
  auto const request_url = QUrl(QString::fromStdString(tileURL(tile_id)));
  QNetworkRequest request(request_url);
  char constexpr agent[] =
    "rviz_satellite " RVIZ_SATELLITE_VERSION " (https://github.com/Kettenhoax/rviz_satellite)";
  request.setHeader(QNetworkRequest::KnownHeaders::UserAgentHeader, agent);
  QVariant variant;
  variant.setValue(tile_id);
  request.setAttribute(
    QNetworkRequest::CacheLoadControlAttribute,
    QNetworkRequest::CacheLoadControl::PreferCache);
  request.setAttribute(QNetworkRequest::User, variant);

  std::promise<QImage> tile_promise;
  auto entry = tile_promises_.emplace(tile_id, std::move(tile_promise));
  if (!entry.second) {
    RVIZ_COMMON_LOG_WARNING_STREAM("Tile request for tile '" << tile_id << "' is already running");
    throw tile_request_error("Duplicate tile request");
  } else {
    RVIZ_COMMON_LOG_DEBUG_STREAM("Requesting tile " << request_url.toString().toStdString());
    manager_->get(request);
  }
  return entry.first->second.get_future();
}

std::future<QImage> TileClient::request_local(TileId const & tile_id)
{
  std::future<QImage> f = std::async(std::launch::async, [tile_id]{
    auto const filename_uri = tileURL(tile_id);

    auto filename = std::regex_replace(filename_uri, std::regex("file://"), "");

    QImageReader reader(QString::fromStdString(filename));

    if (!reader.canRead())
    {
      RVIZ_COMMON_LOG_DEBUG_STREAM("Unable to decode image at " << filename);
      return QImage{ };
    }

    auto image = reader.read().mirrored();

    if (image.isNull())
    {
      RVIZ_COMMON_LOG_DEBUG_STREAM("QImageReader able to decode but read failed for " << filename);
    }

    return image;
  });

  return f;
}

void TileClient::request_finished(QNetworkReply * reply)
{
  const QVariant variant = reply->request().attribute(QNetworkRequest::User);
  auto tile_id = variant.value<TileId>();

  auto promise_it = tile_promises_.find(tile_id);
  // Erase the element pointed by iterator it
  if (promise_it == tile_promises_.end()) {
    RVIZ_COMMON_LOG_ERROR_STREAM(
      "Tile request promise was removed before the network reply finished");
    return;
  }

  const QUrl url = reply->url();
  if (reply->error()) {
    promise_it->second.set_exception(
      std::make_exception_ptr(
        tile_request_error(reply->errorString().toStdString())));
    return;
  }

  // log if tile comes from cache or web
  bool const from_cache = reply->attribute(QNetworkRequest::SourceIsFromCacheAttribute).toBool();
  if (from_cache) {
    RVIZ_COMMON_LOG_DEBUG_STREAM("Loaded tile from cache " << url.toString().toStdString());
  } else {
    RVIZ_COMMON_LOG_DEBUG_STREAM("Loaded tile from web " << url.toString().toStdString());
  }

  QImageReader reader(reply);
  if (!reader.canRead()) {
    promise_it->second.set_exception(
      std::make_exception_ptr(
        tile_request_error(
          "Failed to decode tile image")));
    RVIZ_COMMON_LOG_ERROR_STREAM(
      "Failed to decode image at " << reply->request().url().toString().toStdString());
    return;
  }
  promise_it->second.set_value(reader.read().mirrored());
  tile_promises_.erase(promise_it);
  reply->deleteLater();
}

}  // namespace rviz_satellite
