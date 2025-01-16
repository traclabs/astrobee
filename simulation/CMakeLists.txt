# Copyright (c) 2017, United States Government, as represented by the
# Administrator of the National Aeronautics and Space Administration.
#
# All rights reserved.
#
# The Astrobee platform is licensed under the Apache License, Version 2.0
# (the "License"); you may not use this file except in compliance with the
# License. You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations
# under the License.

cmake_minimum_required(VERSION 3.5)
project(astrobee_gazebo)

set(ENABLE_GAZEBO ON)
if (ENABLE_GAZEBO)
## Compile as C++17
add_compile_options(-std=c++17)

set( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wall -O3 -fPIC" )

# Find amend and libraries
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(ff_common REQUIRED)
find_package(ff_util REQUIRED)
find_package(ff_msgs REQUIRED)
find_package(ff_hw_msgs REQUIRED)
find_package(pmc REQUIRED)
find_package(camera REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Find packages
find_package(ros_gz REQUIRED)
find_package(gz_sensors_vendor REQUIRED)
find_package(gz-sensors REQUIRED)
find_package(gz_rendering_vendor REQUIRED)
find_package(gz-rendering REQUIRED)
find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim REQUIRED)
find_package(gz_physics_vendor REQUIRED)
find_package(gz-physics REQUIRED)

# Find jsoncpp
LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/../cmake")
find_package(JsonCpp REQUIRED)

#   visualization_msgs
#   tf2
#   tf2_ros
#   tf2_eigen
#   tf2_msgs
#   
#   sensor_msgs
#   ff_msgs
#   ff_hw_msgs
#   tf2_geometry_msgs
#   ff_util
#   gnc_autocode
#   camera
#   cv_bridge
#   jsonloader
# )


###########
## Build ##
###########

# Specify additional locations of header files
include_directories(
  include
  ${JSONCPP_INCLUDE_DIRS}
)
# Add gazebo to the link directorries
# link_directories(
#   ${ros_gz_LIBRARY_DIRS}
# )

# list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

# Create reusable classes for models and plugins
add_library(astrobee_gazebo SHARED
  src/astrobee_gazebo.cc
)
target_link_libraries(astrobee_gazebo gz-sensors::gz-sensors gz-rendering::gz-rendering gz-sim::gz-sim gz-physics::gz-physics)
ament_target_dependencies(astrobee_gazebo rclcpp ff_common ff_util ros_gz)

# Create a model plugin for the EPS system
add_library(gazebo_model_plugin_eps SHARED
  src/gazebo_model_plugin_eps/gazebo_model_plugin_eps.cc
)
target_link_libraries(gazebo_model_plugin_eps astrobee_gazebo)
ament_target_dependencies(gazebo_model_plugin_eps ff_hw_msgs tf2 tf2_ros )

# Create a model plugin for aerodynamic drag
add_library(gazebo_model_plugin_drag SHARED
  src/gazebo_model_plugin_drag/gazebo_model_plugin_drag.cc
)
target_link_libraries(gazebo_model_plugin_drag astrobee_gazebo)

# Create a model plugin for the ground truth
add_library(gazebo_model_plugin_truth SHARED
  src/gazebo_model_plugin_truth/gazebo_model_plugin_truth.cc
)
target_link_libraries(gazebo_model_plugin_truth astrobee_gazebo)
#ament_target_dependencies(gazebo_model_plugin_truth rclcpp tf2 tf2_ros ros_gz )

# Create a model plugin for the propulsion system
#add_library(gazebo_model_plugin_pmc SHARED
#  src/gazebo_model_plugin_pmc/gazebo_model_plugin_pmc.cc
#)
#target_link_libraries(gazebo_model_plugin_pmc astrobee_gazebo)
#ament_target_dependencies(gazebo_model_plugin_pmc ff_hw_msgs tf2 tf2_ros )
#ament_target_dependencies(gazebo_model_plugin_pmc rclcpp ff_msgs ff_hw_msgs ros_gz  pmc)
#ament_export_libraries(gazebo_model_plugin_pmc)

# Create a model plugin for the perching arm
add_library(gazebo_model_plugin_perching_arm SHARED
  src/gazebo_model_plugin_perching_arm/gazebo_model_plugin_perching_arm.cc
)
target_link_libraries(gazebo_model_plugin_perching_arm astrobee_gazebo)
ament_target_dependencies(gazebo_model_plugin_perching_arm ff_hw_msgs ) #tf2 tf2_ros )
#ament_target_dependencies(gazebo_model_plugin_perching_arm rclcpp ff_msgs ff_hw_msgs )

# Create a model plugin for the front flashlight
add_library(gazebo_model_plugin_flashlight SHARED
  src/gazebo_model_plugin_flashlight/gazebo_model_plugin_flashlight.cc
)
target_link_libraries(gazebo_model_plugin_flashlight astrobee_gazebo)
ament_target_dependencies(gazebo_model_plugin_flashlight ff_hw_msgs visualization_msgs )

# Create a model plugin for the laser
#add_library(gazebo_model_plugin_laser SHARED
#  src/gazebo_model_plugin_laser/gazebo_model_plugin_laser.cc
#)
#target_link_libraries(gazebo_model_plugin_laser astrobee_gazebo)
#ament_target_dependencies(gazebo_model_plugin_laser rclcpp ff_hw_msgs visualization_msgs ros_gz )
#ament_export_libraries(gazebo_model_plugin_laser)

# Create a model plugin to publish heartbeats
add_library(gazebo_model_plugin_heartbeat SHARED
  src/gazebo_model_plugin_heartbeat/gazebo_model_plugin_heartbeat.cc
)
target_link_libraries(gazebo_model_plugin_heartbeat astrobee_gazebo)

# Create a model plugin for the signal lights
#add_library(gazebo_model_plugin_signal_lights SHARED
#  src/gazebo_model_plugin_signal_lights/gazebo_model_plugin_signal_lights.cc
#)
#target_link_libraries(gazebo_model_plugin_signal_lights astrobee_gazebo)
#ament_target_dependencies(gazebo_model_plugin_signal_lights rclcpp ros_gz )
#ament_export_libraries(gazebo_model_plugin_signal_lights)

# Create a sensor plugin for the IMU
#add_library(gazebo_sensor_plugin_imu SHARED
#   src/gazebo_sensor_plugin_imu/gazebo_sensor_plugin_imu.cc
#)
#target_link_libraries(gazebo_sensor_plugin_imu astrobee_gazebo)
#ament_target_dependencies(gazebo_sensor_plugin_imu rclcpp ros_gz )
#ament_export_libraries(gazebo_sensor_plugin_imu)
#target_include_directories( 
#  gazebo_sensor_plugin_imu PRIVATE include 
#)

# Create a sensor plugin for the handrail detection algorithm
#add_library(gazebo_sensor_plugin_handrail_detect SHARED
#  src/gazebo_sensor_plugin_handrail_detect/gazebo_sensor_plugin_handrail_detect.cc
#)
#target_link_libraries(gazebo_sensor_plugin_handrail_detect astrobee_gazebo)
#ament_target_dependencies(gazebo_sensor_plugin_handrail_detect rclcpp ros_gz  geometry_msgs tf2_ros camera)
#ament_export_libraries(gazebo_sensor_plugin_handrail_detect)

# Create a sensor plugin for the depth camera
#add_library(gazebo_sensor_plugin_depth_cam SHARED
#   src/gazebo_sensor_plugin_depth_cam/gazebo_sensor_plugin_depth_cam.cc
#)
#target_link_libraries(gazebo_sensor_plugin_depth_cam astrobee_gazebo)
#ament_target_dependencies(gazebo_sensor_plugin_depth_cam rclcpp ros_gz  sensor_msgs)
#ament_export_libraries(gazebo_sensor_plugin_depth_cam)

# Create a sensor plugin for the nav cam
add_library(gazebo_sensor_plugin_nav_cam SHARED
  src/gazebo_sensor_plugin_nav_cam/gazebo_sensor_plugin_nav_cam.cc
)
target_link_libraries(gazebo_sensor_plugin_nav_cam astrobee_gazebo)
ament_target_dependencies(gazebo_sensor_plugin_nav_cam rclcpp tf2 tf2_ros sensor_msgs geometry_msgs ros_gz )

# Create a sensor plugin for the dock cam
#add_library(gazebo_sensor_plugin_dock_cam SHARED
#  src/gazebo_sensor_plugin_dock_cam/gazebo_sensor_plugin_dock_cam.cc
#)
#target_link_libraries(gazebo_sensor_plugin_dock_cam astrobee_gazebo)
#ament_target_dependencies(gazebo_sensor_plugin_dock_cam rclcpp tf2 tf2_ros sensor_msgs ros_gz )
#ament_export_libraries(gazebo_sensor_plugin_dock_cam)

# Create a model plugin for sparse mapping
#add_library(gazebo_sensor_plugin_sparse_map SHARED
#  src/gazebo_sensor_plugin_sparse_map/gazebo_sensor_plugin_sparse_map.cc
#)
#target_link_libraries(gazebo_sensor_plugin_sparse_map astrobee_gazebo)
#ament_target_dependencies(gazebo_sensor_plugin_sparse_map rclcpp ros_gz  camera)
#ament_export_libraries(gazebo_sensor_plugin_sparse_map)

# Create a model plugin for AR localization
#add_library(gazebo_sensor_plugin_ar_tags SHARED
#  src/gazebo_sensor_plugin_ar_tags/gazebo_sensor_plugin_ar_tags.cc
#)
#target_link_libraries(gazebo_sensor_plugin_ar_tags astrobee_gazebo)
#ament_target_dependencies(gazebo_sensor_plugin_ar_tags rclcpp ros_gz  camera)
#ament_export_libraries(gazebo_sensor_plugin_ar_tags)

# Create a model plugin for optical flow
#add_library(gazebo_sensor_plugin_optical_flow SHARED
#  src/gazebo_sensor_plugin_optical_flow/gazebo_sensor_plugin_optical_flow.cc
#)
#target_link_libraries(gazebo_sensor_plugin_optical_flow astrobee_gazebo)
#ament_target_dependencies(gazebo_sensor_plugin_optical_flow rclcpp ros_gz  sensor_msgs camera)
#ament_export_libraries(gazebo_sensor_plugin_optical_flow)

# Create a world plugin for speed
add_library(gazebo_world_plugin_speed SHARED
  src/gazebo_world_plugin_speed/gazebo_world_plugin_speed.cc
)
target_link_libraries(gazebo_world_plugin_speed gz-sim::gz-sim)

# Create a system plugin for the client gui
#add_library(gazebo_system_plugin_client SHARED
#  src/gazebo_system_plugin_client/gazebo_system_plugin_client.cc
#)
#target_link_libraries(gazebo_system_plugin_client astrobee_gazebo)
#ament_target_dependencies(gazebo_system_plugin_client rclcpp ros_gz )
#ament_export_libraries(gazebo_system_plugin_client)

#############
## Install ##
#############

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.sh.in")


ament_export_dependencies(ament_cmake rclcpp)

ament_export_include_directories(include)


## Install
install(TARGETS
  astrobee_gazebo
  gazebo_model_plugin_eps
  gazebo_model_plugin_drag
  gazebo_model_plugin_truth
  #gazebo_model_plugin_pmc
  gazebo_model_plugin_perching_arm
  gazebo_model_plugin_flashlight
#  gazebo_model_plugin_laser
  gazebo_model_plugin_heartbeat
#  gazebo_model_plugin_signal_lights
#  gazebo_sensor_plugin_imu
#  gazebo_sensor_plugin_depth_cam
  gazebo_sensor_plugin_nav_cam
#  gazebo_sensor_plugin_dock_cam
#  gazebo_sensor_plugin_ar_tags
#  gazebo_sensor_plugin_optical_flow
#  gazebo_sensor_plugin_handrail_detect
#  gazebo_sensor_plugin_sparse_map
  gazebo_world_plugin_speed
#  gazebo_system_plugin_client
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

# Mark launch files for installation
install(DIRECTORY launch worlds config
  DESTINATION share/${PROJECT_NAME}
  PATTERN ".svn" EXCLUDE)


# pluginlib_export_plugin_description_file(astrobee_gazebo gazebo_astrobee_plugins.xml)


ament_package()

endif (ENABLE_GAZEBO)
