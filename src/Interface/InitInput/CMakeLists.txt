project(INI_ACC)
cmake_minimum_required(VERSION 3.13)

if(WIN32)
  add_library(${PROJECT_NAME} STATIC
    IniAccess.cpp
  )
else()
  add_library(${PROJECT_NAME} STATIC
    IniAccessUnix.cpp
  )
endif()

include(../../../common.cmake)
