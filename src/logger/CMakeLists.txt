project(LOGGER)
cmake_minimum_required(VERSION 3.13)

add_library(${PROJECT_NAME} STATIC
  logger.cpp
  initialize_log.cpp
)

include(../../common.cmake)
