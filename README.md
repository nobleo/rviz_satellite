## rviz_satellite

Plugin for rviz for displaying satellite maps loaded from the internet.

![Alt text](.screenshot.png?raw=true "Example Image")

### Usage

In order to use rviz_satellite, add this package to your catkin workspace. Then add an instance of `AerialMapDisplay` to your rviz config.

The `Topic` field must point to a publisher of `sensor_msgs/NavSatFix`. Note that rviz_satellite will not reload tiles until the robot moves outside of the centre tile (if dynamic reloading is enabled).

You must provide an `Object URI` (or URL) from which the satellite images are loaded. rviz_satellite presently only supports the [OpenStreetMap](http://wiki.openstreetmap.org/wiki/Slippy_map_tilenames) convention for tile names.

The URI should have the form:

``http://otile1.mqcdn.com/tiles/1.0.0/sat/{z}/{x}/{y}.jpg``

This is the default URI, which will load data from MapQuest. The tiles are free, and go up to zoom level 18. For higher zoom levels, consider using [MapBox](https://www.mapbox.com).

Map tiles will be cached to the `mapscache` directory in the `rviz_satellite` package directory. At present the cache does not expire automatically - you should delete the files in the folder if you want the images to be reloaded.

### Options

- `Topic` is the topic of the GPS measurements.
- `Robot frame` should be a TF from the robot position to the fixed frame.
- `Dynamically reload` will cause imagery to reload as the robot moves out of the center tile. This will only work if the robot frame is specified correctly by TF.
- `Alpha` is simply the display transparency.
- `Draw Under` will cause the map to be displayed below all other geometry.
- `Zoom` is the zoom level of the map. Recommended values are 16-19, as anything smaller is _very_ low resolution. 22 is the current max.
- `Blocks` number of adjacent blocks to load. rviz_satellite will load the central block, and this many blocks around the center. 8 is the current max.
- `Frame Convention` is the convention for X/Y axes of the map. The default is maps XYZ to ENU, which is the default convention for libGeographic and [ROS](www.ros.org/reps/rep-0103.html).

### Questions, Bugs

Contact the author (gareth-cross on github), or open an issue.
