cmake_minimum_required(VERSION 2.8.3)
project(rviz_satellite)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
find_package(Qt4 COMPONENTS QtCore QtGui QtNetwork REQUIRED)
include(${QT_USE_FILE})
find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)

pkg_check_modules(OGRE_OV OGRE OGRE-Overlay)

# Old versions of OGRE (pre 1.9) included OGRE-Overlay in the main package
# (i.e. there was no OGRE-Overlay component).  So if the above
# pkg_check_modules() failed, try looking for just OGRE.
if(NOT OGRE_OV_FOUND)
  pkg_check_modules(OGRE_OV REQUIRED OGRE)
endif(NOT OGRE_OV_FOUND)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rviz
  nav_msgs
  sensor_msgs
)

catkin_package()

set(${PROJECT_NAME}_SOURCES
  src/aerialmap_display.cpp
  src/tileloader.cpp
)

set(${PROJECT_NAME}_HEADERS
  src/aerialmap_display.h
  src/tileloader.h
)

# invoke MOC and UI
QT4_WRAP_CPP(${PROJECT_NAME}_MOCSrcs ${${PROJECT_NAME}_HEADERS})

# QT Declarations
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}
  ${OpenCV_INCLUDE_DIR}
  ${Qt4_INCLUDE_DIR}
  ${OGRE_OV_INCLUDE_DIRS}
  src
)

add_definitions("-Wall -Wunused -std=c++11")

set(PROJECT_SOURCE_FILES
  ${${PROJECT_NAME}_SOURCES}
  ${${PROJECT_NAME}_MOCSrcs}
)

add_library(${PROJECT_NAME}
  ${PROJECT_SOURCE_FILES}
)

target_link_libraries(${PROJECT_NAME}
  ${QT_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
)
