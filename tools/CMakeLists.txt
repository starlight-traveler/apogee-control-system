cmake_minimum_required(VERSION 3.16)
project(acs_ndrt_rocketry_tools LANGUAGES CXX)

option(ACS_TOOLS_BUILD_DECODER "Build native decode utilities" ON)
option(ACS_TOOLS_BUILD_REPLAY "Build hosted replay tooling" ON)
option(ACS_TOOLS_ENABLE_NATIVE_CPP_GUI "Build native C++ telemetry GUI (telemetry/imgui)" ON)

if(ACS_TOOLS_BUILD_DECODER)
  add_subdirectory(decode/native)
endif()

if(ACS_TOOLS_BUILD_REPLAY)
  add_subdirectory(replay)
endif()

if(ACS_TOOLS_ENABLE_NATIVE_CPP_GUI AND EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/../telemetry/imgui/CMakeLists.txt)
  add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../telemetry/imgui ${CMAKE_BINARY_DIR}/telemetry_imgui_build)
  if(TARGET teensy_imgui_telemetry)
    add_custom_target(
      run_native_gui
      COMMAND $<TARGET_FILE:teensy_imgui_telemetry>
      WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/telemetry_imgui_build
      COMMENT "Run native C++ telemetry GUI"
    )
  endif()
endif()

if(TARGET acs_ndrt_rocketry_decoder)
  add_custom_target(
    run_native_decoder_gui
    COMMAND $<TARGET_FILE:acs_ndrt_rocketry_decoder>
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    COMMENT "Run native C++ ACS NDRT Rocketry ImGui decoder (CSV/BIN analyzer)"
  )
endif()

if(TARGET acs_replay)
  add_custom_target(
    run_replay
    COMMAND $<TARGET_FILE:acs_replay>
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    COMMENT "Run hosted apogee replay tool"
  )
endif()

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/gui_dashboard.py)
  add_custom_target(
    run_gui
    COMMAND ${CMAKE_COMMAND} -E env PYTHONUNBUFFERED=1 python3 ${CMAKE_CURRENT_SOURCE_DIR}/gui_dashboard.py --https --host localhost --port 8765
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    COMMENT "Run ACS NDRT Rocketry GUI dashboard on https://localhost:8765"
  )
endif()
