^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rviz_satellite
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
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
