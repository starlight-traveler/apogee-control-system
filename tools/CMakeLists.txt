cmake_minimum_required(VERSION 3.16)
project(acs_ndrt_rocketry_tools LANGUAGES CXX)

option(ACS_TOOLS_BUILD_GUI_HELPERS "Build helper binaries used by tools/gui_dashboard.py" ON)
option(ACS_TOOLS_ENABLE_NATIVE_CPP_GUI "Build native C++ GUI (local-imgui-telemetry)" ON)

if(ACS_TOOLS_BUILD_GUI_HELPERS)
  add_subdirectory(cpp)
endif()

if(ACS_TOOLS_ENABLE_NATIVE_CPP_GUI AND EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/../local-imgui-telemetry/CMakeLists.txt)
  add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../local-imgui-telemetry ${CMAKE_BINARY_DIR}/local_imgui_telemetry_build)
  if(TARGET teensy_imgui_telemetry)
    add_custom_target(
      run_native_gui
      COMMAND $<TARGET_FILE:teensy_imgui_telemetry>
      WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/local_imgui_telemetry_build
      COMMENT "Run native C++ telemetry GUI"
    )
  endif()
  add_custom_target(
    run_native_decoder_gui
    COMMAND $<TARGET_FILE:acs_ndrt_rocketry_decoder>
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/local_imgui_telemetry_build
    COMMENT "Run native C++ ACS NDRT Rocketry ImGui decoder (CSV/BIN analyzer)"
  )
endif()

add_custom_target(
  run_gui
  COMMAND ${CMAKE_COMMAND} -E env PYTHONUNBUFFERED=1 python3 ${CMAKE_CURRENT_SOURCE_DIR}/gui_dashboard.py --https --host localhost --port 8765
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  COMMENT "Run ACS NDRT Rocketry GUI dashboard on https://localhost:8765"
)
