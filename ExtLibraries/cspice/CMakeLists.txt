project(cspice)

cmake_minimum_required(VERSION 3.18)

include(ExternalProject)
include(FetchContent)

set(CSPICE_INSTALL_DIR ${EXT_LIB_DIR}/cspice)
set(GENERIC_KERNEL_URL_BASE https://naif.jpl.nasa.gov/pub/naif/generic_kernels)

if(WIN32)
  # Windows
  if(BUILD_64BIT)
    set(CSPICE_URL http://naif.jpl.nasa.gov/pub/naif/toolkit/C/PC_Windows_VisualC_64bit/packages/cspice.zip)
    set(CSPICE_SHA256 "98D60B814B412FA55294AEAAEB7DAB46D849CC87A8B709FFE835D08DE17625DC")
    set(CSPICE_BUILD_COMMAND "")
  else()
    set(CSPICE_URL http://naif.jpl.nasa.gov/pub/naif/toolkit/C/PC_Windows_VisualC_32bit/packages/cspice.zip)
    set(CSPICE_SHA256 "bc566e6a975c888fc5fd89d76554329501446bda88c8bab937c0725faead170f")
    set(CSPICE_BUILD_COMMAND "")
  endif()
elseif(APPLE)
  # APPLE
  if(APPLE_SILICON)  
    # APPLE Silicon
    set(CSPICE_URL https://naif.jpl.nasa.gov/pub/naif/toolkit//C/MacM1_OSX_clang_64bit/packages/cspice.tar.Z)
    set(CSPICE_SHA256 "0deae048443e11ca4d093cac651d9785d4f2594631a183d85a3d58949f4d0aa9")
    set(CSPICE_CONFIGURE_COMMAND "")
    set(CSPICE_BUILD_COMMAND "")
  else()
    # APPLE Intel
    set(CSPICE_URL https://naif.jpl.nasa.gov/pub/naif/toolkit//C/MacIntel_OSX_AppleC_64bit/packages/cspice.tar.Z)
    set(CSPICE_SHA256 "6f4980445fee4d363dbce6f571819f4a248358d2c1bebca47e0743eedfe9935e")
    set(CSPICE_CONFIGURE_COMMAND "")
    set(CSPICE_BUILD_COMMAND "")
  endif()
else()
  # Linux
  if(BUILD_64BIT)
    set(CSPICE_URL https://naif.jpl.nasa.gov/pub/naif/toolkit/C/PC_Linux_GCC_64bit/packages/cspice.tar.Z)
    set(CSPICE_SHA256 "60a95b51a6472f1afe7e40d77ebdee43c12bb5b8823676ccc74692ddfede06ce")
    set(CSPICE_CONFIGURE_COMMAND "")
    set(CSPICE_BUILD_COMMAND "")

    # If we build on aarch64 linux, we cannot use binary package. Let's (super-slow) build
    if(NOT ${CMAKE_SYSTEM_PROCESSOR} STREQUAL "x86_64")
      # super remove no-need compile option
      set(CSPICE_CONFIGURE_COMMAND find . -type f -exec sed -i "s/-m64//g" {} +)

      # build cspice by host C-compiler with original build logic
      set(CSPICE_BUILD_COMMAND "./makeall.csh")
    endif()
  else()
    set(CSPICE_URL https://naif.jpl.nasa.gov/pub/naif/toolkit/C/PC_Linux_GCC_32bit/packages/cspice.tar.Z)
    set(CSPICE_SHA256 "33d75cd94acf6546e53d7ebc4e7d3d6d42ac27c83cb0d8f04c91a8b50c1149e3")
    set(CSPICE_CONFIGURE_COMMAND "")
    set(CSPICE_BUILD_COMMAND "")
  endif()
endif()
# build & install cspice
message("URL" ${CSPICE_URL})
ExternalProject_Add(cspice
  URL ${CSPICE_URL}
  URL_HASH SHA256=${CSPICE_SHA256}
  # DOWNLOAD_EXTRACT_TIMESTAMP false # prepare for CMake 3.24
  SOURCE_DIR "cspice"
  CONFIGURE_COMMAND ${CSPICE_CONFIGURE_COMMAND}
  BUILD_IN_SOURCE true
  BUILD_COMMAND "${CSPICE_BUILD_COMMAND}"
  INSTALL_COMMAND ""
)

# install
install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/cspice/include
  DESTINATION ${CSPICE_INSTALL_DIR}
)

if(WIN32)
  # Windows
  if(BUILD_64BIT)
    set(CSPICE_LIB_DEST "cspice_msvs64")
  else()
    set(CSPICE_LIB_DEST "cspice_msvs")
  endif()
elseif(APPLE)
  # APPLE
  if(APPLE_SILICON)  
    # APPLE Silicon
    set(CSPICE_LIB_DEST "cspice_apple_silicon64")
  else()
    # APPLE Intel
    set(CSPICE_LIB_DEST "cspice_apple_intel64")
  endif()
else()
  # Linux
  if(BUILD_64BIT)
    set(CSPICE_LIB_DEST "cspice_unix64")
  else()
    set(CSPICE_LIB_DEST "cspice_unix")
  endif()
endif()

message("cspice/lib will be install to ${CSPICE_LIB_DEST}")
install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/cspice/lib
  DESTINATION ${CSPICE_INSTALL_DIR}/${CSPICE_LIB_DEST}
)

# install generic kernels
function(download_generic_kernel dir kernel sha256)
  message("downloading ${dir}/${kernel}...")
  FetchContent_Declare(${kernel}
    URL ${GENERIC_KERNEL_URL_BASE}/${dir}/${kernel}
    URL_HASH SHA256=${sha256}
    DOWNLOAD_NO_EXTRACT true
    SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/generic_kernels/${dir}
  )
  FetchContent_MakeAvailable(${kernel})
endfunction(download_generic_kernel)

download_generic_kernel(lsk/a_old_versions naif0010.tls "a3826c1f418a9601afdf92be815298aedfd7960b5c00f3a5c469df674e1436b4")
download_generic_kernel(pck de-403-masses.tpc "5cd68fcd3f59ddc21ed8bbad3a341126b8e092a29ac8f0ae585db718af8e7468")
download_generic_kernel(pck gm_de431.tpc "15756c162151853f329d473fa00c38d35e62d83c3e25ddceece6516bbd98738b")
download_generic_kernel(pck pck00010.tpc "59468328349aa730d18bf1f8d7e86efe6e40b75dfb921908f99321b3a7a701d2")
download_generic_kernel(spk/planets de430.bsp "6e1b277c5f07135a84950604b83e56b736be696a7f3560bcddb1d4aeb944fca1")

# move generic kernel
configure_file(
  ${CMAKE_CURRENT_BINARY_DIR}/generic_kernels/lsk/a_old_versions/naif0010.tls
  ${CMAKE_CURRENT_BINARY_DIR}/generic_kernels/lsk/naif0010.tls
  COPYONLY
)

install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/generic_kernels
  DESTINATION ${CSPICE_INSTALL_DIR}
)
