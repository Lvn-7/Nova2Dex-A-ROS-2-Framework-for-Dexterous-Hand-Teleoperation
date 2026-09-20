cmake_minimum_required(VERSION 3.10)
project(inspire_hand_dds_bridge)

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

add_executable(inspire_jointstate_to_dds src/inspire_jointstate_to_dds.cpp)
target_include_directories(inspire_jointstate_to_dds PRIVATE include /usr/local/include/ddscxx)
ament_target_dependencies(inspire_jointstate_to_dds rclcpp sensor_msgs)
target_link_libraries(
  inspire_jointstate_to_dds
  ${UNITREE_SDK2_LIB}
  ${DDSC_LIB}
  ${DDSCXX_LIB}
  pthread
)

if(BUILD_TESTING)
  enable_testing()
  add_executable(inspire_conversion_self_check test/conversion_self_check.cpp)
  target_include_directories(inspire_conversion_self_check PRIVATE include)
  add_test(NAME inspire_conversion_self_check COMMAND inspire_conversion_self_check)
endif()

install(TARGETS inspire_jointstate_to_dds DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

ament_package()
