cmake_minimum_required(VERSION 2.8.3)
project(rviz_plugins)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
find_package(Qt4 REQUIRED)
include(${QT_USE_FILE})
find_package(OpenCV REQUIRED)
find_package(OGRE REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rviz
  nav_msgs
)

catkin_package()

set(${PROJECT_NAME}_SOURCES
  src/aerialmap_display.cpp
)

set(${PROJECT_NAME}_HEADERS
  src/aerialmap_display.h
)

# invoke MOC and UI
QT4_WRAP_CPP(${PROJECT_NAME}_MOCSrcs ${${PROJECT_NAME}_HEADERS})

# QT Declarations
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}
  ${OpenCV_INCLUDE_DIR}
  /usr/include/OGRE
  ${Qt4_INCLUDE_DIR}
  src
)

add_definitions("-Wall -Wunused")

set(PROJECT_SOURCE_FILES
  ${${PROJECT_NAME}_SOURCES}
  ${${PROJECT_NAME}_MOCSrcs}
)

add_library(${PROJECT_NAME}
  ${PROJECT_SOURCE_FILES}
)
message(STATUS "Project source files: ${PROJECT_SOURCE_FILES}")

target_link_libraries(${PROJECT_NAME}
  ${QT_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
)
