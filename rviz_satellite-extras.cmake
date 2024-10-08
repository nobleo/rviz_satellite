# There is a workaround to find the GeographicLib that needs to be exported to all downstream packages
# Workaround to find GeographicLib according to https://bugs.launchpad.net/ubuntu/+source/geographiclib/+bug/1805173
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};/usr/share/cmake/geographiclib")