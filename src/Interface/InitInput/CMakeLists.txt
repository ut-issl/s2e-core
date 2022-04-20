## ini access
### TODO: split CMakeLists.txt
if(WIN32)
  add_library(INI_ACC STATIC IniAccess.cpp)
else()
  add_library(INI_ACC STATIC IniAccessUnix.cpp)
endif()
# Compile option
target_include_directories(INI_ACC PUBLIC ${S2E_DIR}/src)
if(MSVC)
  target_compile_options(INI_ACC PUBLIC "/W4")
  target_compile_options(INI_ACC PUBLIC "/source-charset:utf-8")
else()
  target_compile_options(INI_ACC PUBLIC -Wall -g -m32 -rdynamic)
endif()
