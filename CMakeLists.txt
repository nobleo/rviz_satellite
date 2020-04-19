cmake_minimum_required(VERSION 2.8.12)
project(rviz_satellite)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rviz
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS sensor_msgs
)

find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Gui Network REQUIRED)
set(QT_LIBRARIES Qt5::Core Qt5::Gui Qt5::Network)


##
## BUILD

set(${PROJECT_NAME}_SOURCES
  src/aerialmap_display.cpp
  src/ogre_tile.cpp
  src/tile_id.cpp
)

set(${PROJECT_NAME}_HEADERS
  src/aerialmap_display.h
  src/detail/tile_downloader.h
)

# invoke MOC and UI/ include Qt headers/ link Qt libraries - alternatively, see CMAKE_AUTOMOC
qt5_wrap_cpp(${PROJECT_NAME}_MOC_FILES ${${PROJECT_NAME}_HEADERS})

set(PROJECT_SOURCE_FILES
  ${${PROJECT_NAME}_SOURCES}
  ${${PROJECT_NAME}_MOC_FILES}
)

link_directories(
  ${catkin_LIBRARY_DIRS}
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

target_include_directories(
  ${PROJECT_NAME}
  PUBLIC
  src
  ${catkin_INCLUDE_DIRS}
)

target_link_libraries(
  ${PROJECT_NAME}
  PUBLIC
  ${QT_LIBRARIES}
  ${catkin_LIBRARIES}
)


##
## INSTALL

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
