project(cspice)

cmake_minimum_required(VERSION 3.13)

include(ExternalProject)
include(FetchContent)

set(CSPICE_INSTALL_DIR ${EXT_LIB_DIR}/cspice)
set(GENERIC_KERNEL_URL_BASE https://naif.jpl.nasa.gov/pub/naif/generic_kernels)

if(WIN32)
  set(CSPICE_URL http://naif.jpl.nasa.gov/pub/naif/toolkit/C/PC_Windows_VisualC_32bit/packages/cspice.zip)
  set(CSPICE_SHA256 "bc566e6a975c888fc5fd89d76554329501446bda88c8bab937c0725faead170f")
  set(CSPICE_BUILD_COMMAND cd src/cspice COMMAND mkprodct.bat)
else()
  # Linux
  set(CSPICE_URL https://naif.jpl.nasa.gov/pub/naif/toolkit/C/PC_Linux_GCC_32bit/packages/cspice.tar.Z)
  set(CSPICE_SHA256 "33d75cd94acf6546e53d7ebc4e7d3d6d42ac27c83cb0d8f04c91a8b50c1149e3")
  set(CSPICE_BUILD_COMMAND "")
endif()

# build & install cspice
ExternalProject_Add(cspice
  URL ${CSPICE_URL}
  URL_HASH SHA256=${CSPICE_SHA256}
  SOURCE_DIR "cspice"
  CONFIGURE_COMMAND ""
  BUILD_IN_SOURCE true
  BUILD_COMMAND "${CSPICE_BUILD_COMMAND}"
  INSTALL_COMMAND
    ${CMAKE_COMMAND} -DCSPICE_INSTALL_DIR=${CSPICE_INSTALL_DIR} -P ${CMAKE_CURRENT_LIST_DIR}/install_step.cmake
)

# install generic kernels
function(download_generic_kernel dir kernel sha256)
  message("downloading ${dir}/${kernel}...")
  FetchContent_Declare(${kernel}
    URL ${GENERIC_KERNEL_URL_BASE}/${dir}/${kernel}
    URL_HASH SHA256=${sha256}
    DOWNLOAD_NO_EXTRACT true
    SOURCE_DIR ${CSPICE_INSTALL_DIR}/generic_kernels
  )
  FetchContent_MakeAvailable(${kernel})
endfunction(download_generic_kernel)

download_generic_kernel(lsk/a_old_versions naif0010.tls "a3826c1f418a9601afdf92be815298aedfd7960b5c00f3a5c469df674e1436b4")
download_generic_kernel(pck de-403-masses.tpc "5cd68fcd3f59ddc21ed8bbad3a341126b8e092a29ac8f0ae585db718af8e7468")
download_generic_kernel(pck gm_de431.tpc "15756c162151853f329d473fa00c38d35e62d83c3e25ddceece6516bbd98738b")
download_generic_kernel(pck pck00010.tpc "59468328349aa730d18bf1f8d7e86efe6e40b75dfb921908f99321b3a7a701d2")
download_generic_kernel(spk/planets de430.bsp "6e1b277c5f07135a84950604b83e56b736be696a7f3560bcddb1d4aeb944fca1")
