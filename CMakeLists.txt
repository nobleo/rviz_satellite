cmake_minimum_required(VERSION 2.8.3)
project(rviz_satellite)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rviz
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS sensor_msgs
)

if(rviz_QT_VERSION VERSION_LESS "5")
  message(FATAL_ERROR "Need at least Qt5, found: ${rviz_QT_VERSION}")
endif()

find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Gui Network Concurrent REQUIRED)
set(QT_LIBRARIES Qt5::Core Qt5::Gui Qt5::Network Qt5::Concurrent)


##
## BUILD

set(${PROJECT_NAME}_SOURCES
  src/aerialmap_display.cpp
  src/OgreTile.cpp
  src/TileId.cpp
)

set(${PROJECT_NAME}_HEADERS
  src/aerialmap_display.h
  src/detail/TileDownloader.h
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

link_directories(
  ${catkin_LIBRARY_DIRS}
)

# set compiler options and inject version of software (see TileDownloader.h)
add_definitions("-Wall -Wunused -std=c++11 -DRVIZ_SATELLITE_VERSION=\"\\\"${rviz_satellite_VERSION}\\\"\"")

add_library(
  ${PROJECT_NAME} 
  ${PROJECT_SOURCE_FILES}
)

target_link_libraries(
  ${PROJECT_NAME}
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



get_cmake_property(_variableNames VARIABLES)
list (SORT _variableNames)
foreach (_variableName ${_variableNames})
    message(STATUS "${_variableName}=${${_variableName}}")
endforeach()