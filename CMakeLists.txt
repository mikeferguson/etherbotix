cmake_minimum_required(VERSION 3.5)
project(etherbotix)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(Boost REQUIRED system)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(robot_controllers_interface REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
)

add_library(etherbotix SHARED
  src/etherbotix.cpp
  src/etherbotix_motor.cpp
)
target_link_libraries(etherbotix ${Boost_LIBRARIES})
ament_target_dependencies(etherbotix
  rclcpp
  rclcpp_components
  robot_controllers_interface
  sensor_msgs
)

rclcpp_components_register_node(etherbotix
  PLUGIN "etherbotix::Etherbotix"
  EXECUTABLE etherbotix_driver
)

install(
  TARGETS
    etherbotix
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_cpplint)
  ament_cpplint(FILTERS "-whitespace/braces" "-whitespace/newline")
endif()

ament_package()
