cmake_minimum_required(VERSION 3.5)
project(rviz_satellite)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
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
find_package(angles REQUIRED)

find_package(Qt5 COMPONENTS Network REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/aerialmap_display.cpp
  src/field.cpp
  src/tile.cpp
  src/tile_client.cpp
  src/tile_object.cpp
)

target_compile_options(
  ${PROJECT_NAME}
  PUBLIC
  "-DRVIZ_SATELLITE_VERSION=\"${rviz_satellite_VERSION}\"")

target_include_directories(${PROJECT_NAME} PRIVATE src)
target_link_libraries(${PROJECT_NAME} Qt5::Network)

# prevent pluginlib from using boost
target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
set_property(TARGET ${PROJECT_NAME} PROPERTY AUTOMOC ON)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_field test/test_field.cpp)
  target_link_libraries(test_field ${PROJECT_NAME})
  target_include_directories(test_field PRIVATE src)
endif()

ament_target_dependencies(${PROJECT_NAME} SYSTEM angles rclcpp pluginlib rviz_common rviz_default_plugins sensor_msgs tf2_ros)
ament_export_dependencies(rclcpp pluginlib rviz_common rviz_default_plugins sensor_msgs tf2_ros)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)

install(
  DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/icons"
  DESTINATION "share/${PROJECT_NAME}")

install(TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install demo
install(FILES demo/demo.launch.xml
  DESTINATION share/${PROJECT_NAME}/launch)
install(PROGRAMS demo/publish_demo_data
  DESTINATION lib/${PROJECT_NAME})

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)
ament_package()
