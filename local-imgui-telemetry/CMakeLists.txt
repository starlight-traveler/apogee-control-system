cmake_minimum_required(VERSION 3.20)
project(teensy_imgui_telemetry CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)

option(ACS_SETUP_VOICE_ENV "Bootstrap local Python venv for voice control during build" OFF)
option(ACS_SETUP_VOICE_MODEL "Download/unpack Vosk model during build" OFF)
option(ACS_BUILD_TEENSY_UDP_GUI "Build live UDP telemetry GUI (POSIX socket code)" ON)
set(ACS_VOICE_VENV_DIR "${CMAKE_CURRENT_SOURCE_DIR}/.venv" CACHE PATH "Local Python venv path for voice control")
set(ACS_VOSK_MODEL_NAME "vosk-model-en-us-0.22" CACHE STRING "Vosk model folder name")
set(ACS_VOSK_MODEL_URL "https://alphacephei.com/vosk/models/vosk-model-en-us-0.22.zip" CACHE STRING "Vosk model zip URL")
set(ACS_VOSK_MODEL_DIR "$<TARGET_FILE_DIR:teensy_imgui_telemetry>/models" CACHE STRING "Directory where Vosk models are stored")

find_package(OpenGL REQUIRED)
find_package(glfw3 QUIET)
if(NOT glfw3_FOUND)
  FetchContent_Declare(
    glfw
    GIT_REPOSITORY https://github.com/glfw/glfw.git
    GIT_TAG 3.4
  )
  set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
  set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
  set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(glfw)
endif()

find_package(imgui QUIET CONFIG)
if(imgui_FOUND)
  set(IMGUI_TARGET imgui::imgui)
else()
  FetchContent_Declare(
    imgui
    GIT_REPOSITORY https://github.com/ocornut/imgui.git
    GIT_TAG v1.91.8
  )
  FetchContent_MakeAvailable(imgui)

  add_library(imgui_vendor STATIC
    ${imgui_SOURCE_DIR}/imgui.cpp
    ${imgui_SOURCE_DIR}/imgui_draw.cpp
    ${imgui_SOURCE_DIR}/imgui_tables.cpp
    ${imgui_SOURCE_DIR}/imgui_widgets.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
  )
  target_include_directories(imgui_vendor PUBLIC
    ${imgui_SOURCE_DIR}
    ${imgui_SOURCE_DIR}/backends
  )
  target_link_libraries(imgui_vendor PUBLIC OpenGL::GL glfw)
  set(IMGUI_TARGET imgui_vendor)
endif()

if(WIN32)
  # Current teensy live GUI uses POSIX socket headers in main.cpp.
  if(ACS_BUILD_TEENSY_UDP_GUI)
    message(WARNING "ACS_BUILD_TEENSY_UDP_GUI is ON, but main.cpp currently targets POSIX sockets. Turning it OFF on Windows.")
    set(ACS_BUILD_TEENSY_UDP_GUI OFF CACHE BOOL "Build live UDP telemetry GUI (POSIX socket code)" FORCE)
  endif()
endif()

if(ACS_BUILD_TEENSY_UDP_GUI)
  add_executable(teensy_imgui_telemetry main.cpp)
  target_include_directories(teensy_imgui_telemetry PRIVATE ${CMAKE_SOURCE_DIR}/../include)
  target_link_libraries(teensy_imgui_telemetry PRIVATE OpenGL::GL glfw ${IMGUI_TARGET})
endif()

add_executable(acs_imgui_decoder ../tools/cpp/fast_decode_sensor_log.cpp)
target_compile_definitions(acs_imgui_decoder PRIVATE ACS_ENABLE_IMGUI_DECODER=1)
target_link_libraries(acs_imgui_decoder PRIVATE OpenGL::GL glfw ${IMGUI_TARGET})
set_target_properties(acs_imgui_decoder PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

add_executable(acs_ndrt_rocketry_decoder ../tools/cpp/fast_decode_sensor_log.cpp)
target_compile_definitions(acs_ndrt_rocketry_decoder PRIVATE ACS_ENABLE_IMGUI_DECODER=1)
target_link_libraries(acs_ndrt_rocketry_decoder PRIVATE OpenGL::GL glfw ${IMGUI_TARGET})
set_target_properties(acs_ndrt_rocketry_decoder PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
if(ACS_BUILD_TEENSY_UDP_GUI)
  set_target_properties(teensy_imgui_telemetry PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
endif()

if(ACS_BUILD_TEENSY_UDP_GUI)
  add_custom_command(TARGET teensy_imgui_telemetry POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
      ${CMAKE_CURRENT_SOURCE_DIR}/voice_listener.py
      $<TARGET_FILE_DIR:teensy_imgui_telemetry>/voice_listener.py
  )
endif()

set(ACS_VOSK_MODEL_STAMP "${CMAKE_CURRENT_BINARY_DIR}/.acs_vosk_model_ready")
add_custom_command(
  OUTPUT ${ACS_VOSK_MODEL_STAMP}
  COMMAND ${CMAKE_COMMAND}
    -DACS_VOSK_MODEL_URL=${ACS_VOSK_MODEL_URL}
    -DACS_VOSK_MODEL_DIR=${ACS_VOSK_MODEL_DIR}
    -DACS_VOSK_MODEL_NAME=${ACS_VOSK_MODEL_NAME}
    -DACS_VOSK_MODEL_STAMP=${ACS_VOSK_MODEL_STAMP}
    -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/download_vosk_model.cmake
  COMMENT "Downloading/unpacking Vosk model ${ACS_VOSK_MODEL_NAME}"
  VERBATIM
)
add_custom_target(voice_model DEPENDS ${ACS_VOSK_MODEL_STAMP})
if(ACS_SETUP_VOICE_MODEL AND ACS_BUILD_TEENSY_UDP_GUI)
  add_dependencies(teensy_imgui_telemetry voice_model)
endif()

find_package(Python3 COMPONENTS Interpreter QUIET)
if(Python3_Interpreter_FOUND)
  if(WIN32)
    set(ACS_VOICE_VENV_PYTHON "${ACS_VOICE_VENV_DIR}/Scripts/python.exe")
  else()
    set(ACS_VOICE_VENV_PYTHON "${ACS_VOICE_VENV_DIR}/bin/python")
  endif()
  set(ACS_VOICE_VENV_STAMP "${ACS_VOICE_VENV_DIR}/.acs_voice_ready")

  add_custom_command(
    OUTPUT ${ACS_VOICE_VENV_STAMP}
    COMMAND ${Python3_EXECUTABLE} -m venv ${ACS_VOICE_VENV_DIR}
    COMMAND ${ACS_VOICE_VENV_PYTHON} -m pip install --upgrade pip
    COMMAND ${ACS_VOICE_VENV_PYTHON} -m pip install vosk sounddevice
    COMMAND ${CMAKE_COMMAND} -E touch ${ACS_VOICE_VENV_STAMP}
    COMMENT "Bootstrapping local voice venv at ${ACS_VOICE_VENV_DIR}"
    VERBATIM
  )

  add_custom_target(voice_env DEPENDS ${ACS_VOICE_VENV_STAMP})
  if(ACS_SETUP_VOICE_MODEL)
    add_dependencies(voice_env voice_model)
  endif()
  if(ACS_SETUP_VOICE_ENV AND ACS_BUILD_TEENSY_UDP_GUI)
    add_dependencies(teensy_imgui_telemetry voice_env)
  endif()
else()
  message(WARNING "Python3 interpreter not found; voice_env target unavailable.")
endif()

if(UNIX AND NOT APPLE)
  if(TARGET teensy_imgui_telemetry)
    target_link_libraries(teensy_imgui_telemetry PRIVATE pthread)
  endif()
  target_link_libraries(acs_imgui_decoder PRIVATE pthread)
  target_link_libraries(acs_ndrt_rocketry_decoder PRIVATE pthread)
endif()
