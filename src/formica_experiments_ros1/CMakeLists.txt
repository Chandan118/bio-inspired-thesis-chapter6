cmake_minimum_required(VERSION 3.0.2)
project(formica_experiments)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf
  tf2_ros
  cv_bridge
  image_transport
  message_generation
)

## System dependencies are automatically found via CMake
find_package(OpenCV REQUIRED)

## Generate Python modules
catkin_python_setup()

## Generate messages/services/actions (if needed)
# add_message_files(
#   FILES
# )

# add_service_files(
#   FILES
# )

# add_action_files(
#   FILES
# )

## Generate added messages/services/actions with any dependencies
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

## Generate dynamic reconfigure parameters
# generate_dynamic_reconfigure_params(
#   FILES
#   cfg/DynamicParameters.cfg
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

################################################
## Declare ROS messages, services, actions ##
################################################

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS
    roscpp
    rospy
    std_msgs
    sensor_msgs
    geometry_msgs
    nav_msgs
    tf
    tf2_ros
    cv_bridge
    image_transport
)

###########
## Build ##
###########
## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## Declare C++ executables
# add_executable(exp1_node src/exp1_node.cpp)
# target_link_libraries(exp1_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
# add_dependencies(exp1_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Install Python scripts
catkin_install_python(PROGRAMS
  scripts/exp1_sensor_calibration.py
  scripts/exp2_power_profiling.py
  scripts/exp3_slam_mapping.py
  scripts/exp4_maze_navigation.py
  scripts/exp5_obstacle_fault.py
  scripts/exp6_cnn_detection.py
  scripts/exp7_pheromone_trail.py
  scripts/hardware_checker.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

## Install config files
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

#############
## Testing ##
#############
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  # add_rostest_gtest(test_name test/test_name.test test/test_name.cpp)
  # target_link_libraries(test_name ${catkin_LIBRARIES})
endif()
