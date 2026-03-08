cmake_minimum_required(VERSION 3.20)
project(teensy_imgui_telemetry CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# CMake 4 rejects very old policy compatibility levels used by some fetched
# third-party projects (notably PortAudio and parts of whisper.cpp's tree).
# Setting the floor here keeps FetchContent subprojects configurable without
# requiring users to pass the variable manually on every configure.
set(CMAKE_POLICY_VERSION_MINIMUM 3.10 CACHE STRING "Minimum policy version for fetched third-party projects" FORCE)

include(FetchContent)

option(ACS_SETUP_VOICE_MODEL "Download a default Whisper model during build" OFF)
option(ACS_BUILD_TEENSY_UDP_GUI "Build live UDP telemetry GUI (POSIX socket code)" ON)
option(ACS_BUILD_IMGUI_DECODER "Build ImGui decoder UI from tools/decode/native" ON)
option(ACS_ENABLE_NATIVE_VOICE "Enable native voice control via whisper.cpp + PortAudio" ON)
option(ACS_FETCH_GLFW "Fetch/build GLFW with FetchContent" ON)
option(ACS_FETCH_IMGUI "Fetch/build Dear ImGui with FetchContent" ON)
option(ACS_FETCH_PORTAUDIO "Fetch/build PortAudio with FetchContent when native voice is enabled" ON)
set(ACS_WHISPER_MODEL_NAME "ggml-base.en.bin" CACHE STRING "Default whisper.cpp model filename")
set(ACS_WHISPER_MODEL_URL "https://huggingface.co/ggerganov/whisper.cpp/resolve/main/ggml-base.en.bin" CACHE STRING "Default whisper.cpp model URL")
set(ACS_WHISPER_MODEL_DIR "${CMAKE_BINARY_DIR}/bin/models" CACHE PATH "Directory where whisper.cpp models are stored" FORCE)

set(ACS_NEEDS_IMGUI_STACK OFF)
if(ACS_BUILD_TEENSY_UDP_GUI OR ACS_BUILD_IMGUI_DECODER)
  set(ACS_NEEDS_IMGUI_STACK ON)
endif()

if(ACS_NEEDS_IMGUI_STACK)
  find_package(OpenGL REQUIRED)

  if(ACS_FETCH_GLFW)
    FetchContent_Declare(
      glfw
      GIT_REPOSITORY https://github.com/glfw/glfw.git
      GIT_TAG 3.4
    )
    set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
    set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
    set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(glfw)
    set(GLFW_TARGET glfw)
  else()
    find_package(glfw3 REQUIRED)
    set(GLFW_TARGET glfw)
  endif()

  if(ACS_FETCH_IMGUI)
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
    target_link_libraries(imgui_vendor PUBLIC OpenGL::GL ${GLFW_TARGET})
    set(IMGUI_TARGET imgui_vendor)
  else()
    find_package(imgui REQUIRED CONFIG)
    set(IMGUI_TARGET imgui::imgui)
  endif()
endif()

set(ACS_NATIVE_VOICE_AVAILABLE OFF)
if(ACS_BUILD_TEENSY_UDP_GUI AND ACS_ENABLE_NATIVE_VOICE)
  FetchContent_Declare(
    whispercpp
    GIT_REPOSITORY https://github.com/ggml-org/whisper.cpp.git
    GIT_TAG master
  )
  set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
  set(WHISPER_BUILD_TESTS OFF CACHE BOOL "" FORCE)
  set(WHISPER_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
  set(WHISPER_BUILD_SERVER OFF CACHE BOOL "" FORCE)
  set(WHISPER_BUILD_TOOLS OFF CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(whispercpp)

  set(PORTAUDIO_TARGET "")
  if(APPLE)
    find_path(PORTAUDIO_INCLUDE_DIR
      NAMES portaudio.h
      HINTS /opt/homebrew /usr/local
      PATH_SUFFIXES include opt/portaudio/include
    )
    find_library(PORTAUDIO_LIBRARY
      NAMES portaudio libportaudio
      HINTS /opt/homebrew /usr/local
      PATH_SUFFIXES lib opt/portaudio/lib
    )
    if(PORTAUDIO_INCLUDE_DIR AND PORTAUDIO_LIBRARY)
      set(PORTAUDIO_TARGET ${PORTAUDIO_LIBRARY})
    endif()
  endif()
  if(NOT PORTAUDIO_TARGET AND ACS_FETCH_PORTAUDIO)
    set(PA_BUILD_SHARED OFF CACHE BOOL "" FORCE)
    set(PA_BUILD_STATIC ON CACHE BOOL "" FORCE)
    set(PA_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
    set(PA_BUILD_TESTS OFF CACHE BOOL "" FORCE)
    set(PA_BUILD_TOOLS OFF CACHE BOOL "" FORCE)
    FetchContent_Declare(
      portaudio
      GIT_REPOSITORY https://github.com/PortAudio/portaudio.git
      GIT_TAG v19.7.0
    )
    FetchContent_MakeAvailable(portaudio)
    if(TARGET portaudio_static)
      set(PORTAUDIO_TARGET portaudio_static)
    elseif(TARGET portaudio)
      set(PORTAUDIO_TARGET portaudio)
    endif()
  elseif(NOT PORTAUDIO_TARGET)
    find_path(PORTAUDIO_INCLUDE_DIR
      NAMES portaudio.h
      PATH_SUFFIXES include
    )
    find_library(PORTAUDIO_LIBRARY
      NAMES portaudio portaudio_static
      PATH_SUFFIXES lib lib64
    )
    if(PORTAUDIO_INCLUDE_DIR AND PORTAUDIO_LIBRARY)
      set(PORTAUDIO_TARGET ${PORTAUDIO_LIBRARY})
    endif()
  endif()
  find_package(CURL REQUIRED)
  if(TARGET whisper AND PORTAUDIO_TARGET AND CURL_FOUND)
    set(ACS_NATIVE_VOICE_AVAILABLE ON)
  else()
    message(WARNING "Native voice support disabled: whisper.cpp, PortAudio, or CURL dependency setup did not complete.")
  endif()
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
  target_include_directories(teensy_imgui_telemetry PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../../include)
  target_link_libraries(teensy_imgui_telemetry PRIVATE OpenGL::GL ${GLFW_TARGET} ${IMGUI_TARGET})
  target_compile_definitions(teensy_imgui_telemetry PRIVATE ACS_ENABLE_NATIVE_VOICE=$<BOOL:${ACS_NATIVE_VOICE_AVAILABLE}>)
  if(APPLE)
    target_compile_definitions(teensy_imgui_telemetry PRIVATE GL_SILENCE_DEPRECATION=1)
  endif()
  if(ACS_NATIVE_VOICE_AVAILABLE)
    if(PORTAUDIO_INCLUDE_DIR)
      target_include_directories(teensy_imgui_telemetry PRIVATE ${PORTAUDIO_INCLUDE_DIR})
    endif()
    target_link_libraries(teensy_imgui_telemetry PRIVATE whisper ${PORTAUDIO_TARGET} CURL::libcurl)
    if(TARGET whisper)
      add_dependencies(teensy_imgui_telemetry whisper)
    endif()
    if(TARGET portaudio_static)
      add_dependencies(teensy_imgui_telemetry portaudio_static)
    elseif(TARGET portaudio)
      add_dependencies(teensy_imgui_telemetry portaudio)
    endif()
  endif()
  set_target_properties(teensy_imgui_telemetry PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
endif()

if(ACS_BUILD_IMGUI_DECODER)
  add_executable(acs_imgui_decoder ../../tools/decode/native/fast_decode_sensor_log.cpp)
  target_compile_definitions(acs_imgui_decoder PRIVATE ACS_ENABLE_IMGUI_DECODER=1)
  target_include_directories(acs_imgui_decoder PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../../src
    ${CMAKE_CURRENT_SOURCE_DIR}/../../tools/replay/include
  )
  target_link_libraries(acs_imgui_decoder PRIVATE OpenGL::GL ${GLFW_TARGET} ${IMGUI_TARGET})
  if(APPLE)
    target_compile_definitions(acs_imgui_decoder PRIVATE GL_SILENCE_DEPRECATION=1)
  endif()
  set_target_properties(acs_imgui_decoder PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

  add_executable(acs_ndrt_rocketry_decoder ../../tools/decode/native/fast_decode_sensor_log.cpp)
  target_compile_definitions(acs_ndrt_rocketry_decoder PRIVATE ACS_ENABLE_IMGUI_DECODER=1)
  target_include_directories(acs_ndrt_rocketry_decoder PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../../src
    ${CMAKE_CURRENT_SOURCE_DIR}/../../tools/replay/include
  )
  target_link_libraries(acs_ndrt_rocketry_decoder PRIVATE OpenGL::GL ${GLFW_TARGET} ${IMGUI_TARGET})
  if(APPLE)
    target_compile_definitions(acs_ndrt_rocketry_decoder PRIVATE GL_SILENCE_DEPRECATION=1)
  endif()
  set_target_properties(acs_ndrt_rocketry_decoder PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
endif()

set(ACS_WHISPER_MODEL_STAMP "${CMAKE_CURRENT_BINARY_DIR}/.acs_whisper_model_ready")
add_custom_command(
  OUTPUT ${ACS_WHISPER_MODEL_STAMP}
  COMMAND ${CMAKE_COMMAND}
    -DACS_WHISPER_MODEL_URL=${ACS_WHISPER_MODEL_URL}
    -DACS_WHISPER_MODEL_DIR=${ACS_WHISPER_MODEL_DIR}
    -DACS_WHISPER_MODEL_NAME=${ACS_WHISPER_MODEL_NAME}
    -DACS_WHISPER_MODEL_STAMP=${ACS_WHISPER_MODEL_STAMP}
    -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/download_whisper_model.cmake
  COMMENT "Downloading whisper.cpp model ${ACS_WHISPER_MODEL_NAME}"
  VERBATIM
)
add_custom_target(voice_model DEPENDS ${ACS_WHISPER_MODEL_STAMP})
if(ACS_SETUP_VOICE_MODEL AND ACS_BUILD_TEENSY_UDP_GUI)
  add_dependencies(teensy_imgui_telemetry voice_model)
endif()

if(UNIX AND NOT APPLE)
  if(TARGET teensy_imgui_telemetry)
    target_link_libraries(teensy_imgui_telemetry PRIVATE pthread)
  endif()
  if(TARGET acs_imgui_decoder)
    target_link_libraries(acs_imgui_decoder PRIVATE pthread)
  endif()
  if(TARGET acs_ndrt_rocketry_decoder)
    target_link_libraries(acs_ndrt_rocketry_decoder PRIVATE pthread)
  endif()
endif()
