cmake_minimum_required(VERSION 3.16)
project(nova2_glove_driver)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(manus_ros2_msgs REQUIRED)
find_package(rclcpp REQUIRED)

option(BUILD_SGCORE_DIRECT "Build the experimental direct SGCore-linked Nova2 node" OFF)

if(BUILD_SGCORE_DIRECT)
  message(FATAL_ERROR
    "BUILD_SGCORE_DIRECT is intentionally disabled in this workspace. "
    "Use scripts/nova2_udp_bridge.py with the Windows UDP reader to avoid SGCore ABI/GLIBC conflicts.")
endif()

install(DIRECTORY include/
  DESTINATION include
)

install(PROGRAMS
  scripts/nova2_udp_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)
install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_dependencies(geometry_msgs manus_ros2_msgs rclcpp)
ament_package()
