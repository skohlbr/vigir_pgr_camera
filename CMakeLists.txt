cmake_minimum_required(VERSION 2.8.3)
project(pgr_camera)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS camera_info_manager image_transport nodelet roscpp sensor_msgs  dynamic_reconfigure cv_bridge nodelet)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS thread system)

generate_dynamic_reconfigure_options(cfg/PGRCamera.cfg)

catkin_package(
   INCLUDE_DIRS include
#   LIBRARIES nodelet_uvc_camera
   CATKIN_DEPENDS roscpp nodelet sensor_msgs cv_bridge nodelet
#  DEPENDS system_lib
)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(camera_nodelet src/camera_nodelet.cpp src/camera.cpp src/camera_config.cpp)
target_link_libraries(camera_nodelet ${Boost_LIBRARIES} ${catkin_LIBRARIES} flycapture)
add_dependencies(camera_nodelet ${PROJECT_NAME}_gencfg  ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(pgr_camera_node src/camera_node.cpp)
target_link_libraries(pgr_camera_node ${catkin_LIBRARIES} camera_nodelet)
#add_dependencies(pgr_camera_node ${PROJECT_NAME}_gencfg  ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(pgr_stereo_node src/stereo_node.cpp src/stereo.cpp)
target_link_libraries(pgr_stereo_node ${Boost_LIBRARIES} ${catkin_LIBRARIES} flycapture)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS pgr_camera_node camera_nodelet pgr_stereo_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#install the launch files.
install(FILES
  nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#install the launch files.
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.launch"
  
)

#install the yaml files.
install(DIRECTORY params
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.yaml"  
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}  
  PATTERN "*.h"
)
