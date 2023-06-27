project(LIBRARY)
cmake_minimum_required(VERSION 3.13)

add_library(${PROJECT_NAME} STATIC
  atmosphere/simple_air_density_model.cpp

  geodesy/geodetic_position.cpp

  initialize/initialize_file_access.cpp

  logger/logger.cpp
  logger/initialize_log.cpp

  randomization/global_randomization.cpp
  randomization/normal_randomization.cpp
  randomization/minimal_standard_linear_congruential_generator.cpp
  randomization/minimal_standard_linear_congruential_generator_with_shuffle.cpp

  math/quaternion.cpp
  math/vector.cpp
  math/s2e_math.cpp

  optics/gaussian_beam_base.cpp

  orbit/orbital_elements.cpp
  orbit/kepler_orbit.cpp
  orbit/relative_orbit_models.cpp

  external/igrf/igrf.cpp
  external/inih/ini.c
  external/inih/cpp/INIReader.cpp
  external/nrlmsise00/wrapper_nrlmsise00.cpp
  external/sgp4/sgp4ext.cpp
  external/sgp4/sgp4io.cpp
  external/sgp4/sgp4unit.cpp

  utilities/endian.cpp
  utilities/slip.cpp
  utilities/quantization.cpp
  utilities/ring_buffer.cpp
)

include(../../common.cmake)
