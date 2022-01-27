# C++
set_target_properties(${PROJECT_NAME} PROPERTIES LANGUAGE CXX)
set_target_properties(${PROJECT_NAME} PROPERTIES CXX_STANDARD 14)

# Compile option
if(MSVC)
  target_compile_options(${PROJECT_NAME} PUBLIC "/W4")
  target_compile_options(${PROJECT_NAME} PUBLIC "/source-charset:utf-8")
else()
  target_compile_options(${PROJECT_NAME} PUBLIC "-Wall")
  target_compile_options(${PROJECT_NAME} PUBLIC "-m32 -rdynamic -Wall -g")
endif()
