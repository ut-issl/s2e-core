# C++
set_target_properties(${PROJECT_NAME} PROPERTIES LANGUAGE CXX)
set_target_properties(${PROJECT_NAME} PROPERTIES CXX_STANDARD 17)
set_target_properties(${PROJECT_NAME} PROPERTIES CXX_EXTENSIONS FALSE)

# common include
target_include_directories(${PROJECT_NAME} PUBLIC ${S2E_DIR}/src)
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

# Directory path setting
target_compile_definitions(${PROJECT_NAME} PRIVATE "INI_FILE_DIR_FROM_EXE=\"${INI_FILE_DIR_FROM_EXE}\"")
target_compile_definitions(${PROJECT_NAME} PRIVATE "EXT_LIB_DIR_FROM_EXE=\"${EXT_LIB_DIR_FROM_EXE}\"")
target_compile_definitions(${PROJECT_NAME} PRIVATE "CORE_DIR_FROM_EXE=\"${CORE_DIR_FROM_EXE}\"")

# Compile option
if(MSVC)
  target_compile_options(${PROJECT_NAME} PUBLIC "/MP")  # multi process build

  target_compile_options(${PROJECT_NAME} PUBLIC "/W4")
  if(NOT USE_HILS) # /MT option conflicts /clr option in the HILS config.
    target_compile_options(${PROJECT_NAME} PUBLIC "/MT")
  endif()
  target_compile_options(${PROJECT_NAME} PUBLIC "/source-charset:utf-8")
else()
  target_compile_options(${PROJECT_NAME} PUBLIC "-Wall")
  target_compile_options(${PROJECT_NAME} PUBLIC "-Wextra")
  target_compile_options(${PROJECT_NAME} PUBLIC "-Wpedantic")
  target_compile_options(${PROJECT_NAME} PUBLIC "-pthread")

  # link option
  target_link_options(${PROJECT_NAME} PUBLIC "-rdynamic")
  arget_link_options(${PROJECT_NAME} PUBLIC "-lpigpio")
  arget_link_options(${PROJECT_NAME} PUBLIC "-lrt")

  if(NOT BUILD_64BIT)
    # 32bit
    target_compile_options(${PROJECT_NAME} PUBLIC "-m32")
    target_link_options(${PROJECT_NAME} PUBLIC "-m32")
  endif()
  # debug
  target_compile_options(${PROJECT_NAME} PUBLIC "-g")
endif()
