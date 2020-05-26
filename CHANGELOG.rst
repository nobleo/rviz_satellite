^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rviz_satellite
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.0.0 (2020-05-26)
------------------
* Code cleanup (#76, #75)
* Remove the 'resolution' property (#74)
* Fix frame jitter by splitting map and fixed-frame transforms (#56)
* Cleanup cmake (#70)
* Remove NED and NWU frame conversion options

2.0.0 (2020-04-17)
------------------
* Drop Qt4 support

1.3.0 (2020-04-17)
------------------
* Fix setting the fixed frame
* Fix race when reading the blocks property
* Set tile fetching preference to cache
* Remove the robot frame property. Instead use the frame from the NavSatFix topic
* Fix demo.launch
* Fix uninitialized variable

1.2.0 (2019-03-07)
------------------
* Rewrite of the tile loader
* Display an error if the tile server is invalid
* Move map towards the robot's z-position
* Use XDG cache path for caching tiles
* Incorporate orientation of tile frame when rendering map

1.1.0 (2018-12-04)
------------------
* Integrate package version via CMake
* Detect Qt 5 automatically, adding support for ROS Lunar, Melodic, et al
* Remove unused opencv linking

1.0.0 (2018-11-08)
------------------
* Initial release
