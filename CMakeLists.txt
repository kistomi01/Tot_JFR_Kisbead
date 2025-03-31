cmake_minimum_required(VERSION 3.8)
project(coolsquare)

# Default to C++14 (ROS 2 Humble+ uses C++17; update if needed)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)

# Dependencies
set(ament_dependencies
  rclcpp
  geometry_msgs
  turtlesim     
)

# Executable name
add_executable(square_node src/coolsquare.cpp)  
ament_target_dependencies(square_node
  ${ament_dependencies}  
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

# Install executable
install(TARGETS
  square_node  
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()