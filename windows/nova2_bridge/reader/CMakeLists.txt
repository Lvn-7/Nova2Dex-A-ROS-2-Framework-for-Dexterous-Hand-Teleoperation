cmake_minimum_required(VERSION 3.20)
project(nova2_windows_udp_reader LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT WIN32)
  message(FATAL_ERROR "nova2_windows_udp_reader is intended to build on Windows.")
endif()

set(BRIDGE_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/..")
set(SG_INCLUDE_DIR "${BRIDGE_ROOT}/sdk/include")
set(SG_LIB_DIR "${BRIDGE_ROOT}/sdk/lib/win64/msvc143/release")

find_package(Qt6 REQUIRED COMPONENTS Widgets)

qt_add_executable(nova2_windows_udp_reader WIN32
  src/nova2_windows_udp_reader.cpp
  src/nova2_reader_window.cpp
)
target_include_directories(nova2_windows_udp_reader PRIVATE "${SG_INCLUDE_DIR}")
target_link_directories(nova2_windows_udp_reader PRIVATE "${SG_LIB_DIR}")
target_link_libraries(nova2_windows_udp_reader PRIVATE Qt6::Widgets sgcore sgconnect ws2_32)

if(MSVC)
  # Keep source decoding stable for MSVC even though UI strings are ASCII.
  target_compile_options(nova2_windows_udp_reader PRIVATE /utf-8)
endif()

add_custom_command(TARGET nova2_windows_udp_reader POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy_if_different
    "${SG_LIB_DIR}/sgcore.dll"
    "$<TARGET_FILE_DIR:nova2_windows_udp_reader>/sgcore.dll"
  COMMAND ${CMAKE_COMMAND} -E copy_if_different
    "${SG_LIB_DIR}/sgconnect.dll"
    "$<TARGET_FILE_DIR:nova2_windows_udp_reader>/sgconnect.dll"
)
