^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rviz_satellite
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* added generic utm rotation calculation based on only latitude and longitude
* added property to visualize in utm frame, if activated utm_rotation factor will be calculated to correct position and orientation of tiles
* Add exception handling for the shiftMap error
* Contributors: Florian, junhee.lee

4.2.1 (2025-05-23)
------------------
* List Tim Clephas as maintainer to receive buildfarm emails
* Replace ament_target_dependencies with target_link_libraries
* Contributors: Alejandro Hernández Cordero, Tim Clephas

4.2.0 (2025-05-20)
------------------
* Fixed demo (install rviz file)
* Add libproj-dev as dependency at package.xml
* Add support for sub-region tile servers (local maps)
* fix: time conversion constant in publish_demo_data
* Contributors: Ahmad Kamal Nasir, Christian Geller, David Alejo Teissière, Tim Clephas, nmitropoulos

4.1.0 (2024-08-27)
------------------
* Fix tile map not moving
* tiles: Add support for reading tiles from filesystem. (`#121 <https://github.com/nobleo/rviz_satellite/issues/121>`_)
* Fix Demo
* Contributors: Jack Geissinger, Ramon Wijnands, Tim Clephas, Karl Schulz

4.0.0 (2023-02-20)
------------------
* ROS2 support!
* Contributors: Lee Hicks, Marcel Zeilinger, Tim Clephas, Vitaliy Bondar, ceranit

3.0.2 (2020-10-13)
------------------
* Fix time synchronization of NavSat transform lookup

3.0.1 (2020-08-03)
------------------
* Fix cleanup bug breaking the navsat-tile transforms (#84, #85)

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
