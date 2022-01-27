message("install cspice to ${CSPICE_INSTALL_DIR}")
message("  source: ${CMAKE_SOURCE_DIR}")

file(MAKE_DIRECTORY "${CSPICE_INSTALL_DIR}")

message("install cspice/include")
file(
  INSTALL ${CMAKE_SOURCE_DIR}/include
  DESTINATION ${CSPICE_INSTALL_DIR}
)

if(WIN32)
  set(CSPICE_LIB_DEST "cspice_msvs")
else()
  # Linux
  set(CSPICE_LIB_DEST "cspice_unix")
endif()

message("install cspice/lib")
file(
  COPY ${CMAKE_SOURCE_DIR}/lib
  DESTINATION ${CSPICE_INSTALL_DIR}/${CSPICE_LIB_DEST}
)

message("install cspice done.")
