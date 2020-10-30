cmake_minimum_required(VERSION 3.5)
project(rviz_satellite)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wunused)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pluginlib REQUIRED)
find_package(OGRE REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

find_package(Qt5 COMPONENTS Core Gui Network Concurrent Widgets REQUIRED)
set(QT_LIBRARIES Qt5::Core Qt5::Gui Qt5::Network Qt5::Concurrent Qt5::Widgets)

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

add_library(
  ${PROJECT_NAME}
  SHARED
  ${PROJECT_SOURCE_FILES}
)

# inject version of software (see tile_downloader.h)
# rviz_satellite_VERSION is defined via ament and is read from the package.xml
target_compile_options(
  ${PROJECT_NAME}
  PUBLIC
  "-DRVIZ_SATELLITE_VERSION=\"${rviz_satellite_VERSION}\"")

target_include_directories(
  ${PROJECT_NAME}
  PUBLIC
  ${OGRE_INCLUDE_DIRS}
  src
)
  
target_link_libraries(
  ${PROJECT_NAME}
  ${QT_LIBRARIES}
  ${Boost_LIBRARIES}
)

ament_export_include_directories(include)
ament_target_dependencies(${PROJECT_NAME} rclcpp tf2_ros sensor_msgs rviz_common rviz_default_plugins)
ament_export_dependencies(rclcpp tf2_ros sensor_msgs rviz_common rviz_default_plugins)
ament_export_libraries(${PROJECT_NAME})

# prevent pluginlib from using boost
target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

##
## INSTALL

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
)

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

ament_package()
