project(SETTING_FILE_READER)
cmake_minimum_required(VERSION 3.13)

add_library(${PROJECT_NAME} OBJECT
  initialize_file_access.cpp
  c2a_command_database.cpp
  wings_operation_file.cpp
)

include(../../common.cmake)
