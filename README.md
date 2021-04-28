# rviz_satellite

Rviz plugin for displaying satellite maps at the position of a `sensor_msgs/msg/NavSatFix` message.

![Rviz view with updating aerial map](images/rviz_satellite.png "Rviz view with updating aerial map")

## Usage

Add an instance of `AerialMapDisplay` to your Rviz config and point it to a `sensor_msgs/msg/NavSatFix` topic.

Map tiles will be cached to `$HOME/.cache/rviz_satellite`.
At present the cache does not expire automatically - you should delete the files in the folder if you want the images to be reloaded.

Currently, we only support the [OpenStreetMap](http://wiki.openstreetmap.org/wiki/Slippy_map_tilenames) convention for tile URLs.
This e.g. implies that only raster tiles (no vector tiles) are supported.

## Tile servers

You must provide a tile URL (Object URI) from which the satellite images are loaded.
The URL should have the form `http://server.tld/{z}/{x}/{y}.jpg`.
Where the tokens `{z}`, `{x}`, `{y}` represent the zoom level, x coordinate, and y coordinate respectively.
These will automatically be substituted by rviz_satellite when making HTTP requests.

rviz_satellite doesn't come with any preconfigured tile URL.
For example, you could use one of the following tile servers:

* OpenStreetMap: https://tile.openstreetmap.org/{z}/{x}/{y}.png
* TomTom: https://api.tomtom.com/map/1/tile/basic/main/{z}/{x}/{y}.png?tileSize=512&key=[TOKEN]
* Mapbox: https://api.mapbox.com/styles/v1/mapbox/satellite-v9/tiles/256/{z}/{x}/{y}?access_token=[TOKEN]

For some of these, you have to request an access token first.
Please refer to the respective terms of service and copyrights.

## Options

- `Topic` is the topic of the GPS measurements.
- `Alpha` is simply the display transparency.
- `Draw Under` will cause the map to be displayed below all other geometry.
- `Zoom` is the zoom level of the map. Recommended values are 16-19, as anything smaller is _very_ low resolution. 22 is the current max.
- `Blocks` number of adjacent tiles in addition to the center tile to load, 8 maximum.

## Support and Contributions

In case of questions or problems, do not hesitate to open an issue.

Contributions are welcomed. Please add a summary of your changes to the [changelog](CHANGELOG.rst) under the section Forthcoming.
