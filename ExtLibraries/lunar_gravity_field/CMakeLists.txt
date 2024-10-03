project(lunar_gravity_field)

cmake_minimum_required(VERSION 3.18)

include(FetchContent)

set(LUNAR_GRAVITY_FIELD_INSTALL_DIR ${SETTINGS_DIR}/environment/gravity_field)

set(LUNAR_GRAVITY_FIELD_URL_BASE https://pds-geosciences.wustl.edu/grail/grail-l-lgrs-5-rdr-v1/grail_1001/shadr)
set(LUNAR_GRAVITY_FIELD_FILE gggrx_1200a_sha.tab)
set(LUNAR_GRAVITY_FIELD_TMP_DIR ${CMAKE_CURRENT_BINARY_DIR}/tmp)

function(download_file_without_validate base_url file)
  message("downloading ${file}")
  FetchContent_Declare(
    ${file}
    SOURCE_DIR ${LUNAR_GRAVITY_FIELD_TMP_DIR}
    URL ${base_url}/${file}
    DOWNLOAD_NO_EXTRACT true
  )
  FetchContent_MakeAvailable(${file})
endfunction()

## download table
download_file_without_validate(${LUNAR_GRAVITY_FIELD_URL_BASE} ${LUNAR_GRAVITY_FIELD_FILE})

## install table
install(FILES ${LUNAR_GRAVITY_FIELD_TMP_DIR}/${LUNAR_GRAVITY_FIELD_FILE}
  DESTINATION ${LUNAR_GRAVITY_FIELD_INSTALL_DIR}
)
