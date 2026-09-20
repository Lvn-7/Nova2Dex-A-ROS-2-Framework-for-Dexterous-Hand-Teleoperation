cmake_minimum_required(VERSION 3.10)
project(inspire_service)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

if (CMAKE_BUILD_TYPE MATCHES "Debug")
  message("Debug mode")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -g -O0 -fPIC")
else()
  message("Release mode")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O3 -DNDEBUG -fPIC")
endif()

if(POLICY CMP0167)
  cmake_policy(SET CMP0167 NEW)
endif()

find_package(Boost COMPONENTS program_options REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  /usr/local/include/ddscxx
  /usr/local/include/iceoryx/v2.0.2
  include
)

link_libraries(unitree_sdk2 ddsc ddscxx rt pthread)
link_libraries(libboost_program_options.a)

add_executable(inspire_g1 inspire_g1.cpp)
target_link_libraries(inspire_g1 Eigen3::Eigen)

# example
add_executable(hand_example example/hand_example.cpp)
add_executable(hand_finger_test example/hand_finger_test.cpp)
add_executable(hand_all_test example/hand_all_test.cpp)
