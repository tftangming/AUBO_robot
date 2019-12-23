cmake_minimum_required(VERSION 2.8.3)
project(aubo_demo)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

if("$ENV{ROS_DISTRO}" STREQUAL "kinetic")
    find_package(catkin REQUIRED COMPONENTS
      roscpp
      rospy
      std_msgs
      moveit_core
      moveit_visual_tools
      moveit_ros_planning
      moveit_ros_planning_interface
      pluginlib
      cmake_modules
      geometric_shapes
      pcl_ros
      pcl_conversions
      rosbag
      tf
      tf2_ros
      #tf2_eigen
      tf2_geometry_msgs
    )
    Message(STATUS "<<<<<<<<<<<<<<<<<<<<<<<<<Note: Find Package for kinetic")
endif()

if("$ENV{ROS_DISTRO}" STREQUAL "indigo")
    find_package(catkin REQUIRED COMPONENTS
      roscpp
      rospy
      std_msgs
      moveit_core
      moveit_ros_planning
      moveit_ros_planning_interface
      pluginlib
      geometric_shapes
      pcl_ros
      pcl_conversions
      rosbag
      tf
      tf2_ros
      #tf2_eigen
      tf2_geometry_msgs
    )
    Message(STATUS "<<<<<<<<<<<<<<<<<<<<<<<<<Note: Find Package for  for indigo")
endif()





###################################
## catkin specific configuration ##
###################################

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES aubo_demo
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)
catkin_package(
  CATKIN_DEPENDS 
    moveit_core
    moveit_ros_planning_interface
    interactive_markers
)
find_package(Boost REQUIRED system filesystem date_time thread)
find_package(Eigen REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})


if("$ENV{ROS_DISTRO}" STREQUAL "kinetic")
    ## Declare a C++ executable
    ## With catkin_make MoveGroupInterface_To_Kinetic package is built within a single CMake context
    ## The recommended prefix ensures that target names across packages don't collide
    add_executable(MoveGroupInterface_To_Kinetic src/MoveGroupInterface_To_Kinetic.cpp)

    ## Specify libraries to link a library or executable target against
    target_link_libraries(MoveGroupInterface_To_Kinetic ${catkin_LIBRARIES} ${Boost_LIBRARIES})

    add_executable(down src/down.cpp)
    target_link_libraries(down ${catkin_LIBRARIES} ${Boost_LIBRARIES})
    install(TARGETS down DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
    add_executable(reset src/reset.cpp)
    target_link_libraries(reset ${catkin_LIBRARIES} ${Boost_LIBRARIES})
    install(TARGETS reset DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
    add_executable(up src/up.cpp)
    target_link_libraries(up ${catkin_LIBRARIES} ${Boost_LIBRARIES})
    install(TARGETS up DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
    add_executable(left src/left.cpp)
    target_link_libraries(left ${catkin_LIBRARIES} ${Boost_LIBRARIES})
    install(TARGETS left DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
    add_executable(right src/right.cpp)
    target_link_libraries(right ${catkin_LIBRARIES} ${Boost_LIBRARIES})
    install(TARGETS right DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
    add_executable(feed src/feed.cpp)
    target_link_libraries(feed ${catkin_LIBRARIES} ${Boost_LIBRARIES})
    install(TARGETS feed DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
    add_executable(back src/back.cpp)
    target_link_libraries(back ${catkin_LIBRARIES} ${Boost_LIBRARIES})
    install(TARGETS back DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
    add_executable(rotate src/rotate.cpp)
    target_link_libraries(rotate ${catkin_LIBRARIES} ${Boost_LIBRARIES})
    install(TARGETS rotate DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
    add_executable(start src/start.cpp)
    target_link_libraries(start ${catkin_LIBRARIES} ${Boost_LIBRARIES})
    install(TARGETS start DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
    add_executable(stop src/stop.cpp)
    target_link_libraries(stop ${catkin_LIBRARIES} ${Boost_LIBRARIES})
    install(TARGETS stop DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
    add_executable(test1 src/test1.cpp)
    target_link_libraries(test1 ${catkin_LIBRARIES} ${Boost_LIBRARIES})
    install(TARGETS test1 DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endif()

if("$ENV{ROS_DISTRO}" STREQUAL "indigo")
    ## Declare a C++ executable
    ## With catkin_make MoveGroupInterface_To_Kinetic package is built within a single CMake context
    ## The recommended prefix ensures that target names across packages don't collide
    add_executable(MoveGroupInterface_To_Indigo src/MoveGroupInterface_To_Indigo.cpp)

    ## Specify libraries to link a library or executable target against
    target_link_libraries(MoveGroupInterface_To_Indigo ${catkin_LIBRARIES} ${Boost_LIBRARIES})

    Message(STATUS "<<<<<<<<<<<<<<<<<<<<<<<<<Note:build aubo_demo node for indigo")
endif()





