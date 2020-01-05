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

#include <QDir>
#include <QtCore>
#include <QtNetwork>
#include <QImage>
#include <QImageReader>
#include <QStandardPaths>
#include <QString>
#include <QCryptographicHash>

#include <ros/ros.h>

#include "detail/ErrorRateManager.h"
#include "TileId.h"

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
  detail::ErrorRateManager<std::string> errorRates;

  TileDownloader(decltype(callback) callback) : manager(new QNetworkAccessManager(this)), callback(std::move(callback))
  {
    connect(manager, SIGNAL(finished(QNetworkReply*)), SLOT(downloadFinished(QNetworkReply*)));

    QNetworkDiskCache* diskCache = new QNetworkDiskCache(this);
    QString const cachePath =
        QDir(QStandardPaths::writableLocation(QStandardPaths::GenericCacheLocation)).filePath("rviz_satellite");
    diskCache->setCacheDirectory(cachePath);
    // there is no option to disable maximum cache size
    diskCache->setMaximumCacheSize(std::numeric_limits<qint64>::max());
    manager->setCache(diskCache);
  }

  /**
   * @brief Load a specific tile
   *
   * Since QNetworkDiskCache is used, tiles will be loaded from the file system if they have been cached. Otherwise they
   * get downloaded.
   */
  void loadTile(TileId const& tileId)
  {
    // see https://foundation.wikimedia.org/wiki/Maps_Terms_of_Use#Using_maps_in_third-party_services
    QNetworkRequest request(QUrl(QString::fromStdString(tileURL(tileId))));
    char constexpr agent[] = "rviz_satellite/" RVIZ_SATELLITE_VERSION " (+https://github.com/gareth-cross/"
                             "rviz_satellite)";
    request.setHeader(QNetworkRequest::KnownHeaders::UserAgentHeader, agent);
    QVariant variant;
    variant.setValue(tileId);
    request.setAttribute(QNetworkRequest::User, variant);
    manager->get(request);
  }

public slots:
  void downloadFinished(QNetworkReply* reply)
  {
    QVariant const variant = reply->request().attribute(QNetworkRequest::User);
    TileId const tileId = variant.value<TileId>();

    QUrl const url = reply->url();
    if (reply->error())
    {
      ROS_ERROR_STREAM(reply->errorString().toStdString());
      errorRates.issueError(tileId.tileServer);
      return;
    }
    else
    {
      errorRates.issueSuccess(tileId.tileServer);
    }

    QImageReader reader(reply);
    if (!reader.canRead())
    {
      ROS_ERROR_STREAM("Unable to decode image at " << reply->request().url().toString().toLatin1().data());
      return;
    }

    callback(tileId, reader.read());

    reply->deleteLater();
  }
};

}  // namespace detail
