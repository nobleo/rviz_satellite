/*
 * TileLoader.cpp
 *
 *  Copyright (c) 2014 Gaeth Cross. Apache 2 License.
 *
 *  This file is part of rviz_satellite.
 *
 *	Created on: 07/09/2014
 */

#include "tileloader.h"

#include <QUrl>
#include <QNetworkRequest>
#include <QVariant>
#include <QDir>
#include <QFile>
#include <QImage>
#include <QImageReader>
#include <stdexcept>
#include <boost/regex.hpp>
#include <ros/ros.h>
#include <ros/package.h>


static size_t replaceRegex(const boost::regex &ex, std::string &str,
                           const std::string &replace) {
  std::string::const_iterator start = str.begin(), end = str.end();
  boost::match_results<std::string::const_iterator> what;
  boost::match_flag_type flags = boost::match_default;
  size_t count = 0;
  while (boost::regex_search(start, end, what, ex, flags)) {
    str.replace(what.position(), what.length(), replace);
    start = what[0].second;
    count++;
  }
  return count;
}

void TileLoader::MapTile::abortLoading() {
  if (reply_) {
    reply_->abort();
    reply_ = 0;
  }
}

bool TileLoader::MapTile::hasImage() const { return !image_.isNull(); }

TileLoader::TileLoader(const std::string &service, double latitude,
                       double longitude, unsigned int zoom, unsigned int blocks,
                       QObject *parent)
    : QObject(parent), latitude_(latitude), longitude_(longitude), zoom_(zoom),
      blocks_(blocks), object_uri_(service) {
  assert(blocks_ >= 0);
          
    std::string packagePath = ros::package::getPath("rviz_satellite");
    if ( packagePath.empty() ) throw std::runtime_error("package 'rviz_satellite' not found");
    
    cachePath_ = QDir::cleanPath( QString::fromStdString(packagePath) + QDir::separator() + QString("mapscache"));
    QDir dir(cachePath_);
    if (!dir.exists()) {
        dir.mkpath(".");
    }
          
  /// @todo: some kind of error checking of the URL

  //  calculate center tile coordinates
  double x, y;
  latLonToTileCoords(latitude_, longitude_, zoom_, x, y);
  tile_x_ = std::floor(x);
  tile_y_ = std::floor(y);
  //  fractional component
  origin_x_ = x - tile_x_;
  origin_y_ = y - tile_y_;
}

bool TileLoader::insideCentreTile(double lat, double lon) const {
  double x, y;
  latLonToTileCoords(lat, lon, zoom_, x, y);
  return (std::floor(x) == tile_x_ && std::floor(y) == tile_y_);
}

void TileLoader::start() {
  //  discard previous set of tiles and all pending requests
  abort();

  ROS_INFO("loading %d blocks around tile=(%d,%d)", blocks_, tile_x_, tile_y_ );

  qnam_.reset( new QNetworkAccessManager(this) );
  QObject::connect(qnam_.get(), SIGNAL(finished(QNetworkReply *)), this,
                   SLOT(finishedRequest(QNetworkReply *)));

  //  determine what range of tiles we can load
  const int min_x = std::max(0, tile_x_ - blocks_);
  const int min_y = std::max(0, tile_y_ - blocks_);
  const int max_x = std::min(maxTiles(), tile_x_ + blocks_);
  const int max_y = std::min(maxTiles(), tile_y_ + blocks_);

  //  initiate requests
  for (int y = min_y; y <= max_y; y++) {
    for (int x = min_x; x <= max_x; x++) {
        
        // Generate filename
        QString fileName = "x_" + QString::number(x)+ "y_" +QString::number(y) +"z_" +QString::number(zoom_)+ ".jpg";
        QString fullPath = QDir::cleanPath(cachePath_ + QDir::separator() + fileName);

        // Check if tile is already in the cache
        QFile tile(fullPath);
        if (tile.exists()) {
            QImage image(fullPath);
            tiles_.push_back(MapTile(x, y, zoom_, image));
        } else {
            const QUrl uri = uriForTile(x, y);
            //  send request
            const QNetworkRequest request = QNetworkRequest(uri);
            QNetworkReply *rep = qnam_->get(request);
            emit initiatedRequest(request);
            tiles_.push_back(MapTile(x, y, zoom_, rep));
        }
    }
  }

    // check if we are finished already
    bool loaded = true;
    for (MapTile &tile : tiles_) {
        if (!tile.hasImage()) {
            loaded = false;
            ROS_DEBUG_STREAM( "Tile x: " << tile.x() << " y: "<< tile.y() << " has no image new version" );
        }
    }
    if (loaded) {
        emit finishedLoading();
    }
}

double TileLoader::resolution() const {
  return zoomToResolution(latitude_, zoom_);
}

/// @see http://wiki.openstreetmap.org/wiki/Slippy_map_tilenames
/// For explanation of these calculations.
void TileLoader::latLonToTileCoords(double lat, double lon, unsigned int zoom,
                                    double &x, double &y) {
  if (zoom > 19) {
    throw std::invalid_argument("Zoom level " + std::to_string(zoom) +
                                " too high");
  } else if (lat < -85.0511 || lat > 85.0511) {
    throw std::invalid_argument("Latitude " + std::to_string(lat) + " invalid");
  } else if (lon < -180 && lon > 180) {
    throw std::invalid_argument("Longitude " + std::to_string(lon) +
                                " invalid");
  }

  const double rho = M_PI / 180;
  const double lat_rad = lat * rho;

  unsigned int n = (1 << zoom);
  x = n * ((lon + 180) / 360.0);
  y = n * (1 - (std::log(std::tan(lat_rad) + 1 / std::cos(lat_rad)) / M_PI)) /
      2;
  ROS_DEBUG_STREAM( "Center tile coords: " << x << ", " << y );
}

double TileLoader::zoomToResolution(double lat, unsigned int zoom) {
  const double lat_rad = lat * M_PI / 180;
  return 156543.034 * std::cos(lat_rad) / (1 << zoom);
}

void TileLoader::finishedRequest(QNetworkReply *reply) {
  const QNetworkRequest request = reply->request();

  //  find corresponding tile
  MapTile *tile = 0;
  for (MapTile &t : tiles_) {
    if (t.reply() == reply) {
      tile = &t;
    }
  }
  if (!tile) {
    //  removed from list already, ignore this reply
    return;
  }

  if (reply->error() == QNetworkReply::NoError) {
    //  decode an image
    QImageReader reader(reply);
    if (reader.canRead()) {
      QImage image = reader.read();
      tile->setImage(image);
      QString fileName = "x_" + QString::number(tile->x())+ "y_" +QString::number(tile->y()) +"z_" +QString::number(tile->z())+ ".jpg";
      QString fullPath = QDir::cleanPath(cachePath_ + QDir::separator() + fileName);
      image.save(fullPath,"JPEG");
      emit receivedImage(request);
    } else {
      //  probably not an image
      QString err;
      err = "Unable to decode image at " + request.url().toString();
      emit errorOcurred(err);
    }
  } else {
    QString err;
    err = "Failed loading " + request.url().toString();
    err += " with code " + QString::number(reply->error());
    emit errorOcurred(err);
  }

  //  check if all tiles have images
  bool loaded = true;
  for (MapTile &tile : tiles_) {
    if (!tile.hasImage()) {
      loaded = false;
    }
  }
  if (loaded) {
    emit finishedLoading();
  }
}

QUrl TileLoader::uriForTile(int x, int y) const {
  std::string object = object_uri_;
  //  place {x},{y},{z} with appropriate values
  replaceRegex(boost::regex("\\{x\\}", boost::regex::icase), object,
               std::to_string(x));
  replaceRegex(boost::regex("\\{y\\}", boost::regex::icase), object,
               std::to_string(y));
  replaceRegex(boost::regex("\\{z\\}", boost::regex::icase), object,
               std::to_string(zoom_));

  const QString qstr = QString::fromStdString(object);
  return QUrl(qstr);
}

int TileLoader::maxTiles() const { return (1 << zoom_) - 1; }

void TileLoader::abort() {
  tiles_.clear();
  //  destroy network access manager
  qnam_.reset();
}
