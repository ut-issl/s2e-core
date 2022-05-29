project(nrlmsise00)

cmake_minimum_required(VERSION 3.13)

include(ExternalProject)
include(FetchContent)

set(NRLMSISE_INSTALL_DIR ${EXT_LIB_DIR}/nrlmsise00)

set(NRLMSISE_TABLE_URL_BASE ftp://ftp.agi.com/pub/DynamicEarthData)
set(NRLMSISE_TABLE_FILE SpaceWeather-v1.2.txt)

set(NRLMSISE_TMP_DIR ${NRLMSISE_INSTALL_DIR}/tmp)

# download files
# download source files
FetchContent_Declare(
  nrlmsise
  GIT_REPOSITORY git://git.linta.de/~brodo/nrlmsise-00.git
  GIT_TAG bc9a2feba4344e74201281e563332688a4d09cc3
  SOURCE_DIR ${NRLMSISE_TMP_DIR}
  CONFIGURE_COMMAND ""
  BUILD_COMMAND ""
)
FetchContent_MakeAvailable(nrlmsise)

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
## download table
download_file_without_validate(${NRLMSISE_TABLE_URL_BASE} ${NRLMSISE_TABLE_FILE})

# build
if(WIN32)
  set(NRLMSISE_LIB libnrlmsise00)
else()
  set(NRLMSISE_LIB nrlmsise00) # prevent liblibnrlmsise00.a
endif()
add_library(${NRLMSISE_LIB} STATIC ${NRLMSISE_TMP_DIR}/nrlmsise-00.c ${NRLMSISE_TMP_DIR}/nrlmsise-00_data.c)

# build with 32bit
if(MSVC)
  target_compile_options(${NRLMSISE_LIB} PUBLIC "/MT")
else()
  if(NOT BUILD_64BIT)
    target_compile_options(${NRLMSISE_LIB} PUBLIC "-m32")
    target_link_options(${NRLMSISE_LIB} PRIVATE "-m32")
  endif()
endif()

# install
message("install nrlmsise to ${NRLMSISE_INSTALL_DIR}")

## install CMake config
install(EXPORT nrlmsise00-config
  NAMESPACE nrlmsise00::
  DESTINATION ${NRLMSISE_INSTALL_DIR}
)

## install library
if(BUILD_64BIT)
  set(NRLMSISE_LIB_DIR lib_64bit)
else()
  set(NRLMSISE_LIB_DIR lib)
endif()

install(TARGETS ${NRLMSISE_LIB}
  EXPORT nrlmsise00-config
  ARCHIVE DESTINATION ${NRLMSISE_INSTALL_DIR}/${NRLMSISE_LIB_DIR}
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
install(FILES ${NRLMSISE_TMP_DIR}/${NRLMSISE_TABLE_FILE}
  DESTINATION ${NRLMSISE_INSTALL_DIR}/table
)
