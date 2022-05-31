project(geopotential)

cmake_minimum_required(VERSION 3.13)


set(GEOPOTENTIAL_INSTALL_DIR ${EXT_LIB_DIR}/GeoPotential/)

## install table
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/egm96_to360.ascii
  DESTINATION ${GEOPOTENTIAL_INSTALL_DIR}
)
