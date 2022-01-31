# C++
set_target_properties(${PROJECT_NAME} PROPERTIES LANGUAGE CXX)
set_target_properties(${PROJECT_NAME} PROPERTIES CXX_STANDARD 14)

# common include
target_include_directories(${PROJECT_NAME} PUBLIC ${S2E_DIR}/src)

# Compile option
if(MSVC)
  target_compile_options(${PROJECT_NAME} PUBLIC "/W4")
  target_compile_options(${PROJECT_NAME} PUBLIC "/source-charset:utf-8")
else()
  target_compile_options(${PROJECT_NAME} PUBLIC "-Wall")
  target_compile_options(${PROJECT_NAME} PUBLIC "-rdynamic")

  # 32bit
  target_compile_options(${PROJECT_NAME} PUBLIC "-m32")
  target_link_options(${PROJECT_NAME} PUBLIC "-m32")

  # debug
  target_compile_options(${PROJECT_NAME} PUBLIC "-g")
endif()
