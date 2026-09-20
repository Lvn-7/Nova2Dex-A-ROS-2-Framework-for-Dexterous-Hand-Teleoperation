cmake_minimum_required(VERSION 3.10)
project(brainco_hand_dds_bridge)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

find_library(UNITREE_SDK2_LIB unitree_sdk2 REQUIRED)
find_library(DDSC_LIB ddsc REQUIRED)
find_library(DDSCXX_LIB ddscxx REQUIRED)

add_executable(revo2_jointstate_to_brainco_dds
  src/revo2_jointstate_to_brainco_dds.cpp
)

target_include_directories(revo2_jointstate_to_brainco_dds PRIVATE
  include
  /usr/local/include/ddscxx
)

ament_target_dependencies(revo2_jointstate_to_brainco_dds
  rclcpp
  sensor_msgs
)

target_link_libraries(revo2_jointstate_to_brainco_dds
  ${UNITREE_SDK2_LIB}
  ${DDSC_LIB}
  ${DDSCXX_LIB}
  pthread
)

install(TARGETS revo2_jointstate_to_brainco_dds
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
