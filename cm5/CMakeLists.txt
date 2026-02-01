cmake_minimum_required(VERSION 3.16)

project(cm5_flight_computer LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(cm5_flight_computer
    src/main.cpp
    src/flight_computer.cpp
)

target_include_directories(cm5_flight_computer PRIVATE
    src
    ../include
)
