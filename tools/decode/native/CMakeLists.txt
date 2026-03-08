cmake_minimum_required(VERSION 3.16)

add_executable(acs_fast_decode fast_decode_sensor_log.cpp)
target_compile_features(acs_fast_decode PRIVATE cxx_std_17)
add_executable(acs_ndrt_rocketry_fast_decode fast_decode_sensor_log.cpp)
target_compile_features(acs_ndrt_rocketry_fast_decode PRIVATE cxx_std_17)

if(MSVC)
  target_compile_options(acs_fast_decode PRIVATE /O2 /W4)
  target_compile_options(acs_ndrt_rocketry_fast_decode PRIVATE /O2 /W4)
else()
  target_compile_options(acs_fast_decode PRIVATE -O3 -Wall -Wextra -Wpedantic)
  target_compile_options(acs_ndrt_rocketry_fast_decode PRIVATE -O3 -Wall -Wextra -Wpedantic)
endif()

set_target_properties(
  acs_fast_decode
  PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

set_target_properties(
  acs_ndrt_rocketry_fast_decode
  PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)
