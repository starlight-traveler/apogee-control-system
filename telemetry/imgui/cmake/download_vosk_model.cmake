if(NOT DEFINED ACS_VOSK_MODEL_URL OR ACS_VOSK_MODEL_URL STREQUAL "")
  message(FATAL_ERROR "ACS_VOSK_MODEL_URL is required")
endif()
if(NOT DEFINED ACS_VOSK_MODEL_DIR OR ACS_VOSK_MODEL_DIR STREQUAL "")
  message(FATAL_ERROR "ACS_VOSK_MODEL_DIR is required")
endif()
if(NOT DEFINED ACS_VOSK_MODEL_NAME OR ACS_VOSK_MODEL_NAME STREQUAL "")
  message(FATAL_ERROR "ACS_VOSK_MODEL_NAME is required")
endif()
if(NOT DEFINED ACS_VOSK_MODEL_STAMP OR ACS_VOSK_MODEL_STAMP STREQUAL "")
  message(FATAL_ERROR "ACS_VOSK_MODEL_STAMP is required")
endif()

set(model_root "${ACS_VOSK_MODEL_DIR}")
set(model_dir "${ACS_VOSK_MODEL_DIR}/${ACS_VOSK_MODEL_NAME}")
set(model_zip "${CMAKE_CURRENT_BINARY_DIR}/${ACS_VOSK_MODEL_NAME}.zip")
set(model_check_file "${model_dir}/final.mdl")

if(EXISTS "${model_check_file}")
  message(STATUS "Vosk model already present at ${model_dir}")
  file(TOUCH "${ACS_VOSK_MODEL_STAMP}")
  return()
endif()

file(MAKE_DIRECTORY "${model_root}")
message(STATUS "Downloading model from ${ACS_VOSK_MODEL_URL}")
file(DOWNLOAD "${ACS_VOSK_MODEL_URL}" "${model_zip}" SHOW_PROGRESS STATUS dl_status TLS_VERIFY ON)
list(GET dl_status 0 dl_code)
list(GET dl_status 1 dl_msg)
if(NOT dl_code EQUAL 0)
  message(FATAL_ERROR "Model download failed (${dl_code}): ${dl_msg}")
endif()

message(STATUS "Extracting model zip to ${model_root}")
execute_process(
  COMMAND "${CMAKE_COMMAND}" -E tar xvf "${model_zip}"
  WORKING_DIRECTORY "${model_root}"
  RESULT_VARIABLE extract_code
)
if(NOT extract_code EQUAL 0)
  message(FATAL_ERROR "Failed to extract ${model_zip}")
endif()

if(NOT EXISTS "${model_check_file}")
  message(FATAL_ERROR "Model extracted but ${model_check_file} was not found")
endif()

file(TOUCH "${ACS_VOSK_MODEL_STAMP}")
