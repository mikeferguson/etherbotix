cmake_minimum_required(VERSION 2.8.3)
project(etherbotix_python)

find_package(catkin REQUIRED)
catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    actionlib
    control_msgs
    diagnostic_msgs
    geometry_msgs
    nav_msgs
    rospy
    sensor_msgs
    std_msgs
    std_srvs
    tf
    trajectory_msgs
)
