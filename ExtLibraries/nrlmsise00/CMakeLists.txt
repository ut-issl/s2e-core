project(nrlmsise00)

cmake_minimum_required(VERSION 3.13)

include(ExternalProject)
include(FetchContent)

set(NRLMSISE_INSTALL_DIR ${EXT_LIB_DIR}/nrlmsise00)

set(NRLMSISE_URL_BASE https://ccmc.gsfc.nasa.gov/pub/modelweb/atmospheric/msis/nrlmsise00/nrlmsis00_c_version)
set(NRLMSISE_TABLE_URL_BASE ftp://ftp.agi.com/pub/DynamicEarthData)
set(NRLMSISE_TABLE_FILE SpaceWeather-v1.2.txt)

set(NRLMSISE_TMP_DIR "nrlmsise")

# download files
function(download_file base_url file sha256)
  message("downloading ${file}")
  FetchContent_Declare(
    ${file}
    SOURCE_DIR ${NRLMSISE_TMP_DIR}
    URL ${base_url}/${file}
    URL_HASH SHA256=${sha256}
    DOWNLOAD_NO_EXTRACT true
  )
  FetchContent_MakeAvailable(${file})
endfunction()
function(download_file_without_validate base_url file)
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
download_file(${NRLMSISE_URL_BASE} makefile "6c58c08dc6f83b11407750733a08b45e6a63d39b6d24ea3127e4cec7d6807d34")
download_file(${NRLMSISE_URL_BASE} nrlmsise-00.c "a20d6523420188241963f095ad0b65df44ac0a6fa6db52d2e1df823c366f0285")
download_file(${NRLMSISE_URL_BASE} nrlmsise-00.h "d5b45be690bbc6cf394f69b18c0e641a1769da0a87015704fdabf0e7560794aa")
download_file(${NRLMSISE_URL_BASE} nrlmsise-00_data.c "d0b3022f3c3e7ffdf703cc0e0d02339dfee01c0bf1520b29c68d5f4afd8784d5")
download_file(${NRLMSISE_URL_BASE} nrlmsise-00_test.c "5a95faa996dc265ab85a0f6db641628abe84c9aef504638b4aa465ed76c89113")

## download table
download_file(${NRLMSISE_TABLE_URL_BASE} ${NRLMSISE_TABLE_FILE} "10369764025772abf565902fee36f9cf720b75ba6b8be20159720bf779dd3f1a")


# build
if(WIN32)
  set(NRLMSISE_LIB libnrlmsise00)
else()
  set(NRLMSISE_LIB nrlmsise00) # prevent liblibnrlmsise00.a
endif()
add_library(${NRLMSISE_LIB} STATIC ${NRLMSISE_TMP_DIR}/nrlmsise-00.c ${NRLMSISE_TMP_DIR}/nrlmsise-00_data.c)

# build with 32bit
if(NOT MSVC)
  target_compile_options(${NRLMSISE_LIB} PUBLIC "-m32")
  target_link_options(${NRLMSISE_LIB} PRIVATE "-m32")
endif()

# install
message("install nrlmsise to ${NRLMSISE_INSTALL_DIR}")

## install CMake config
install(EXPORT nrlmsise00-config
  NAMESPACE nrlmsise00::
  DESTINATION ${NRLMSISE_INSTALL_DIR}
)

## install library
install(TARGETS ${NRLMSISE_LIB}
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
