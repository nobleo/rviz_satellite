/*
 * TileLoader.h
 *
 *  Copyright (c) 2014 Gaeth Cross. Apache 2 License.
 *
 *  This file is part of rviz_satellite.
 *
 *	Created on: 07/09/2014
 */

#ifndef TILELOADER_H
#define TILELOADER_H

#include <QObject>
#include <QImage>
#include <QNetworkAccessManager>
#include <QString>
#include <QNetworkReply>
#include <vector>
#include <tuple>

class TileLoader : public QObject {
  Q_OBJECT
public:
  class MapTile {
  public:
    MapTile(int x, int y, int z, QNetworkReply *reply = 0)
        : x_(x), y_(y), z_(z), reply_(reply) {}
      
    MapTile(int x, int y, int z, QImage & image)
      : x_(x), y_(y), image_(image) {}

    /// X tile coordinate.
    const int &x() const { return x_; }

    /// Y tile coordinate.
    const int &y() const { return y_; }
      
    /// Z tile zoom value.
    const int &z() const { return z_; }

    /// Network reply.
    QNetworkReply *reply() { return reply_; }

    /// Abort the network request for this tile, if applicable.
    void abortLoading();

    /// Has a tile successfully loaded?
    bool hasImage() const;

    /// Image associated with this tile.
    const QImage &image() const { return image_; }
    void setImage(const QImage &image) { image_ = image; }

  private:
    int x_;
    int y_;
    int z_;
    QNetworkReply *reply_;
    QImage image_;
  };

  explicit TileLoader(const std::string &service, double latitude,
                      double longitude, unsigned int zoom, unsigned int blocks,
                      QObject *parent = 0);

  /// Start loading tiles asynchronously.
  void start();

  /// Meters/pixel of the tiles.
  double resolution() const;

  /// X index of central tile.
  int tileX() const { return tile_x_; }

  /// Y index of central tile.
  int tileY() const { return tile_y_; }

  /// Fraction of a tile to offset the origin (X).
  double originX() const { return origin_x_; }

  /// Fraction of a tile to offset the origin (Y).
  double originY() const { return origin_y_; }

  /// Convert lat/lon to a tile index with mercator projection.
  static void latLonToTileCoords(double lat, double lon, unsigned int zoom,
                                 double &x, double &y);

  /// Convert latitude and zoom level to ground resolution.
  static double zoomToResolution(double lat, unsigned int zoom);

  /// Path to tiles on the server.
  const std::string &objectURI() const { return object_uri_; }

  /// Current set of tiles.
  const std::vector<MapTile> &tiles() const { return tiles_; }

  /// Cancel all current requests.
  void abort();

signals:

  void initiatedRequest(QNetworkRequest request);

  void receivedImage(QNetworkRequest request);

  void finishedLoading();

  void errorOcurred(QString description);

public slots:

private slots:

  void finishedRequest(QNetworkReply *reply);

private:

  /// URI for tile [x,y]
  QUrl uriForTile(int x, int y) const;

  /// Maximum number of tiles for the zoom level
  int maxTiles() const;

  double latitude_;
  double longitude_;
  unsigned int zoom_;
  int blocks_;
  int tile_x_;
  int tile_y_;
  double origin_x_;
  double origin_y_;

  QNetworkAccessManager *qnam_;
  QString cachePath_;

  std::string object_uri_;

  std::vector<MapTile> tiles_;
};

#endif // TILELOADER_H
