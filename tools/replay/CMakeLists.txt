cmake_minimum_required(VERSION 3.16)

project(acs_replay LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(MATPLOTPP_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(MATPLOTPP_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(MATPLOTPP_BUILD_INSTALLER OFF CACHE BOOL "" FORCE)
set(MATPLOTPP_BUILD_PACKAGE OFF CACHE BOOL "" FORCE)
set(MATPLOTPP_WITH_OPENCV OFF CACHE BOOL "" FORCE)
set(MATPLOTPP_BUILD_EXPERIMENTAL_OPENGL_BACKEND OFF CACHE BOOL "" FORCE)

add_subdirectory(matplotplusplus)

add_executable(
  acs_replay
  main.cpp
  arduino_stubs.cpp
  ../../src/flight_computer.cpp
)

target_compile_features(acs_replay PRIVATE cxx_std_17)
target_include_directories(
  acs_replay
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../src
)
target_link_libraries(acs_replay PRIVATE Matplot++::matplot)

if(MSVC)
  target_compile_options(acs_replay PRIVATE /O2 /W4)
else()
  target_compile_options(acs_replay PRIVATE -O3 -Wall -Wextra -Wpedantic)
endif()

set_target_properties(acs_replay PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
