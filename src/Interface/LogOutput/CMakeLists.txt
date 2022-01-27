project(LOG_OUT)
cmake_minimum_required(VERSION 3.13)

add_library(${PROJECT_NAME} STATIC
  ILoggable.h
  Logger.cpp
  Logger.h
  LogUtility.h
)

include(../../../common.cmake)
