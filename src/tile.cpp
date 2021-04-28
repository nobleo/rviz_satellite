#include "tile.hpp"
#include <string>

namespace rviz_satellite {

float zoomSize(double lat, int zoom)
{
  float constexpr METER_PER_PIXEL_ZOOM_0 = 156543.034;
  return METER_PER_PIXEL_ZOOM_0 * 256 * std::cos(angles::from_degrees(lat)) / (1 << zoom);
}

std::string tileURL(const TileId & tile_id)
{
  auto url = tile_id.server_url;
  boost::replace_all(url, "{x}", std::to_string(tile_id.coord.x));
  boost::replace_all(url, "{y}", std::to_string(tile_id.coord.y));
  boost::replace_all(url, "{z}", std::to_string(tile_id.coord.z));
  return url;
}

/**
 * @see https://wiki.openstreetmap.org/wiki/Slippy_map_tilenames for explanation of these calculations.
 */
TileCoordinate fromWGS(const geographic_msgs::msg::GeoPoint & coord, int zoom)
{
  if (zoom > MAX_ZOOM) {
    throw std::invalid_argument("Zoom level " + std::to_string(zoom) + " too high");
  } else if (coord.latitude < -85.0511 || coord.latitude > 85.0511) {
    throw std::invalid_argument("Latitude " + std::to_string(coord.latitude) + " invalid");
  } else if (coord.longitude < -180 || coord.longitude > 180) {
    throw std::invalid_argument("Longitude " + std::to_string(coord.longitude) + " invalid");
  }

  const double lat = angles::from_degrees(coord.latitude);
  TileCoordinate ret;
  int const n = 1 << zoom;
  ret.x = n * ((coord.longitude + 180) / 360.0);
  ret.y = n * (1 - (std::log(std::tan(lat) + 1 / std::cos(lat)) / M_PI)) / 2;
  ret.z = zoom;
  return ret;
}

}  // namespace rviz_satellite

std::ostream & operator<<(std::ostream & os, const rviz_satellite::TileId & tile_id)
{
  os << rviz_satellite::tileURL(tile_id);
  return os;
}
