cmake_minimum_required(VERSION 3.13)

project(nrlmsise00)

include(ExternalProject)
include(FetchContent)

set(NRLMSISE_INSTALL_DIR ${EXT_LIB_DIR}/nrlmsise00)

set(NRLMSISE_URL_BASE https://ccmc.gsfc.nasa.gov/pub/modelweb/atmospheric/msis/nrlmsise00/nrlmsis00_c_version)
set(NRLMSISE_TABLE_URL_BASE ftp://ftp.agi.com/pub/DynamicEarthData)
set(NRLMSISE_TABLE_FILE SpaceWeather-v1.2.txt)

set(NRLMSISE_TMP_DIR "nrlmsise")

# download files
function(download_file base_url file)
  message("downloading ${file}")
  FetchContent_Declare(
    ${file}
    SOURCE_DIR ${NRLMSISE_TMP_DIR}
    URL ${base_url}/${file}
    DOWNLOAD_NO_EXTRACT true
  )
  FetchContent_MakeAvailable(${file})
endfunction()

## download source files
download_file(${NRLMSISE_URL_BASE} makefile)
download_file(${NRLMSISE_URL_BASE} nrlmsise-00.c)
download_file(${NRLMSISE_URL_BASE} nrlmsise-00.h)
download_file(${NRLMSISE_URL_BASE} nrlmsise-00_data.c)
download_file(${NRLMSISE_URL_BASE} nrlmsise-00_test.c)

## download table
download_file(${NRLMSISE_TABLE_URL_BASE} ${NRLMSISE_TABLE_FILE})


# build
add_library(libnrlmsise00 STATIC ${NRLMSISE_TMP_DIR}/nrlmsise-00.c ${NRLMSISE_TMP_DIR}/nrlmsise-00_data.c)


# install
message("install nrlmsise to ${NRLMSISE_INSTALL_DIR}")

## install CMake config
install(EXPORT nrlmsise00-config
  NAMESPACE nrlmsise00::
  DESTINATION ${NRLMSISE_INSTALL_DIR}
)

## install library
install(TARGETS libnrlmsise00
  EXPORT nrlmsise00-config
  ARCHIVE DESTINATION ${NRLMSISE_INSTALL_DIR}/lib
)

## install source
## TODO: .c要るのか？ヘッダだけにしてディレクトリもincludeでよいのでは？
install(FILES ${NRLMSISE_TMP_DIR}/nrlmsise-00.h
  DESTINATION ${NRLMSISE_INSTALL_DIR}/src
)
install(FILES ${NRLMSISE_TMP_DIR}/nrlmsise-00.c
  DESTINATION ${NRLMSISE_INSTALL_DIR}/src
)
install(FILES ${NRLMSISE_TMP_DIR}/nrlmsise-00_data.c
  DESTINATION ${NRLMSISE_INSTALL_DIR}/src
)

## install table
install(FILES nrlmsise/${NRLMSISE_TABLE_FILE}
  DESTINATION ${NRLMSISE_INSTALL_DIR}/table
)
