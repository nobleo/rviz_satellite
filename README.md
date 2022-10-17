# rviz_satellite

Plugin for rviz for displaying satellite maps loaded from the internet.

![Alt text](.screenshot.png?raw=true "Example Image")

In the near future this plugin will not add support for ROS 2.
For an unofficial ROS 2 fork see [blacksoul000/rviz_satellite](https://github.com/blacksoul000/rviz_satellite/tree/dashing).

In order to use rviz_satellite, add this package to your catkin workspace.

## Demo

The package contains a launch file for demonstration purposes.
Use it to verify your installation and to get started:

```
roslaunch rviz_satellite demo.launch
```

The launch file will fake a GPS position in Philadelphia, USA and display [Wikimedia Maps](https://maps.wikimedia.org) nearby.
You can edit the longitude and latitude values in `launch/demo.gps` to change the position.

Check the Usage section below to learn how to use the position of your robot and a satellite map.

See demo file `launch/demo_utm.launch` for an example of using this plugin in the `UTM frame` mode (see below).

## Usage

Add an instance of `AerialMapDisplay` to your rviz config.

The `Topic` field must point to a publisher of `sensor_msgs/NavSatFix`.

Map tiles will be cached to `$HOME/.cache/rviz_satellite`.
At present the cache does not expire automatically - you should delete the files in the folder if you want the images to be reloaded.

Currently, we only support the [OpenStreetMap](http://wiki.openstreetmap.org/wiki/Slippy_map_tilenames) convention for tile URLs.
This e.g. implies that only raster tiles (no vector tiles) are supported.

Transformation of tiles to RViz fixed frame can be done in two ways that are configured using `Map Transform Type` option:

1. Specify a `Map frame`, which is an ENU-oriented frame in which your robot localizes. This mode expects that the frame
   of the subscribed `NavSatFix` messages is consistent with the measured latitude/longitude in this map frame.
   In this mode, the tiles go through an intermediate transform to the map frame.
2. Specify `UTM frame` (and possibly `UTM zone`). In this mode, no map frame is required and the tiles are directly 
   placed on their UTM positions. This mode expects UTM frame is represented in your transform tree. The subscribed
   `NavSatFix` messages are only used to determine the tiles to download, so small inconsistencies between the
   `NavSatFix` frame and the measured latitude/longitude is not a big problem. In this mode, you can also change the
   XY and Z position references from the `NavSatFix` message to a TF frame. This means the point around which the tiles
   are displayed is determined by UTM pose of the specified frame instead of the `NavSatFix` messages.

## Tile servers

You must provide a tile URL (Object URI) from which the satellite images are loaded.
The URL should have the form `http://server.tld/{z}/{x}/{y}.jpg`.
Where the tokens `{z}`, `{x}`, `{y}` represent the zoom level, x coordinate, and y coordinate respectively. If your API requires a pair of latitude and longitude values instead of x and y tile coordinates, use an URL of the form `http://server.tld/{z}/{lat}/{lon}.jpg`, where `{lat}` and `{lon}` represent the latitude and longitude values of the requested location. All these will automatically be substituted by rviz_satellite when making HTTP requests.

rviz_satellite doesn't come with any preconfigured tile URL.
For example, you could use one of the following tile servers:

* OpenStreetMap: https://tile.openstreetmap.org/{z}/{x}/{y}.png
* TomTom: https://api.tomtom.com/map/1/tile/basic/main/{z}/{x}/{y}.png?tileSize=512&key=[TOKEN]
* Mapbox: https://api.mapbox.com/styles/v1/mapbox/satellite-v9/tiles/256/{z}/{x}/{y}?access_token=[TOKEN]
* GoogleMaps: https://maps.googleapis.com/maps/api/staticmap?maptype=satellite&center={lat},{lon}&zoom={z}&size=256x256&key=[TOKEN]


For some of these, you have to request an access token first.
Please refer to the respective terms of service and copyrights.

## Options

- `Topic` is the topic of the GPS measurements.
- `Map Transform Type` selects between the `Map frame` mode and `UTM frame` mode (see section `Usage`).
- `Alpha` is simply the display transparency.
- `Draw Under` will cause the map to be displayed below all other geometry.
- `Zoom` is the zoom level of the map. Recommended values are 16-19, as anything smaller is _very_ low resolution. 22 is the current max.
- `Blocks` number of adjacent blocks to load. rviz_satellite will load the central block, and this many blocks around the center. 8 is the current max.
- `Z Offset` specifies offset of the displayed tiles in the Z coordinate from their default pose (in meters).

### Options available in `Map frame` mode
- `Map Frame` is the map frame rigidly attached to the world with ENU convention.

### Options available in `UTM frame` mode

- `UTM Frame` is the frame that represents UTM coordinate frame.
- `UTM Zone` is the zone used by the `UTM frame`. Value `-1` triggers autodetection of zone and this property is then
  overridden with the autodetected zone.
- `XY Position Reference` specifies how to determine the point around which tiles are centered. It can be either `<NavSatFix Message>`,
  which uses global coordinates from the received fix messages. Or it can be a TF frame name. In such case, the tiles are
  centered around the XY position of the specified frame in UTM coordinates.
  - Please note that selecting the UTM frame for this reference is invalid. Position of the UTM frame in UTM is `(0, 0)`, which is
    an invalid UTM coordinate (supported range is 100 km - 900 km in most zones).
- `Z Position Reference` specifies how to determine the Z coordinate of the displayed tiles. The meaning of the values is similar
  to `XY Position Reference`. `Z Offset` is applied after computing the reference height.

## Support and Contributions

In case of questions or problems, do not hesitate to open an issue.

Contributions are welcomed. Please add a summary of your changes to the [changelog](CHANGELOG.rst) under the section Forthcoming.
