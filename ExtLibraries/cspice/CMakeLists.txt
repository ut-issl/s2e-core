cmake_minimum_required(VERSION 3.13)

include(ExternalProject)
include(FetchContent)

set(CSPICE_INSTALL_DIR ${EXT_LIB_DIR}/cspice)
set(CSPICE_URL http://naif.jpl.nasa.gov/pub/naif/toolkit/C/PC_Windows_VisualC_32bit/packages/cspice.zip)
set(GENERIC_KERNEL_URL_BASE https://naif.jpl.nasa.gov/pub/naif/generic_kernels)

# build & install cspice
ExternalProject_Add(cspice
  URL ${CSPICE_URL}
  SOURCE_DIR "cspice"
  CONFIGURE_COMMAND ""
  BUILD_IN_SOURCE true
  BUILD_COMMAND makeall.bat
  INSTALL_COMMAND
    ${CMAKE_COMMAND} -DCSPICE_INSTALL_DIR=${CSPICE_INSTALL_DIR} -P ${CMAKE_CURRENT_LIST_DIR}/install_step.cmake
)

# install generic kernels
function(download_generic_kernel dir kernel)
  message("downloading ${dir}/${kernel}...")
  FetchContent_Declare(${kernel}
    URL ${GENERIC_KERNEL_URL_BASE}/${dir}/${kernel}
    DOWNLOAD_NO_EXTRACT true
    SOURCE_DIR ${CSPICE_INSTALL_DIR}/generic_kernels
  )
  FetchContent_MakeAvailable(${kernel})
endfunction(download_generic_kernel)

download_generic_kernel(lsk/a_old_versions naif0010.tls)
download_generic_kernel(pck de-403-masses.tpc)
download_generic_kernel(pck gm_de431.tpc)
download_generic_kernel(pck pck00010.tpc)
download_generic_kernel(spk/planets de430.bsp)
