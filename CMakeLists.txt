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
find_package(PythonLibs REQUIRED)
find_package(Boost REQUIRED python3 system thread)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(robot_controllers_interface REQUIRED)
find_package(sensor_msgs REQUIRED)

ament_python_install_package(${PROJECT_NAME})

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_PATH}
)

add_library(etherbotix SHARED
  src/etherbotix.cpp
  src/etherbotix_motor.cpp
  src/etherbotix_ros.cpp
)
target_link_libraries(etherbotix ${Boost_LIBRARIES})
ament_target_dependencies(etherbotix
  rclcpp
  rclcpp_components
  robot_controllers_interface
  sensor_msgs
)
rclcpp_components_register_node(etherbotix
  PLUGIN "etherbotix::EtherbotixROS"
  EXECUTABLE etherbotix_driver
)

add_library(etherbotix_py SHARED
  src/etherbotix.cpp
  src/etherbotix_motor.cpp
  src/python.cpp
)
set_target_properties(etherbotix_py PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}
  PREFIX "")
target_link_libraries(etherbotix_py
  ${Boost_LIBRARIES}
  ${PYTHON_LIBRARIES}
)
ament_target_dependencies(etherbotix_py
  rclcpp
  robot_controllers_interface
)

add_executable(read_etherbotix src/read_etherbotix.cpp)
target_link_libraries(read_etherbotix
  etherbotix
  ${Boost_LIBRARIES}
)
ament_target_dependencies(read_etherbotix
  rclcpp
  rclcpp_components
  robot_controllers_interface
  sensor_msgs
)


install(
  TARGETS
    etherbotix
    read_etherbotix
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS etherbotix_py
  DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}"
)

if(BUILD_TESTING)
  find_package(ament_cmake_cpplint)
  ament_cpplint(FILTERS "-whitespace/braces" "-whitespace/newline")
endif()

ament_package()
