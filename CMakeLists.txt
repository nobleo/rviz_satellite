cmake_minimum_required(VERSION 3.0.2)
project(rviz_satellite)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    rviz
    sensor_msgs
    tf2
    tf2_geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    rviz
    sensor_msgs
    tf2
)

find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Gui Network REQUIRED)
set(QT_LIBRARIES Qt5::Core Qt5::Gui Qt5::Network)

# Ubuntu libgeographic-dev package installs into non-standard location
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};/usr/share/cmake/geographiclib")
find_package(GeographicLib REQUIRED)


## BUILD ##

set(${PROJECT_NAME}_SOURCES
  src/aerialmap_display.cpp
  src/ogre_tile.cpp
  src/position_reference_property.cpp
  src/tile_id.cpp
)

set(${PROJECT_NAME}_HEADERS
  src/aerialmap_display.h
  src/detail/tile_downloader.h
  src/position_reference_property.h
)

# invoke MOC and UI/ include Qt headers/ link Qt libraries - alternatively, see CMAKE_AUTOMOC
qt5_wrap_cpp(${PROJECT_NAME}_MOC_FILES ${${PROJECT_NAME}_HEADERS})

set(PROJECT_SOURCE_FILES
  ${${PROJECT_NAME}_SOURCES}
  ${${PROJECT_NAME}_MOC_FILES}
)

include_directories(
  src
  ${catkin_INCLUDE_DIRS}
)

add_library(
  ${PROJECT_NAME}
  ${PROJECT_SOURCE_FILES}
)

# set compiler options and inject version of software (see tile_downloader.h)
# rviz_satellite_VERSION is defined via catkin and is read from the package.xml
target_compile_options(
  ${PROJECT_NAME}
  PUBLIC
  -Wall -Wunused -std=c++11 "-DRVIZ_SATELLITE_VERSION=\"${rviz_satellite_VERSION}\""
)

target_link_libraries(
  ${PROJECT_NAME}
  ${QT_LIBRARIES}
  ${catkin_LIBRARIES}
  ${GeographicLib_LIBRARIES}
)


## INSTALL ##

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
