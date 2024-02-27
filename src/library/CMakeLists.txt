project(LIBRARY)
cmake_minimum_required(VERSION 3.13)

add_library(${PROJECT_NAME} STATIC
  atmosphere/simple_air_density_model.cpp
  atmosphere/harris_priester_model.cpp

  geodesy/geodetic_position.cpp

  gnss/sp3_file_reader.cpp
  gnss/gnss_satellite_number.cpp
  gnss/antex_file_reader.cpp
  gnss/bias_sinex_file_reader.cpp

  initialize/initialize_file_access.cpp
  initialize/c2a_command_database.cpp
  initialize/wings_operation_file.cpp

  logger/logger.cpp
  logger/initialize_log.cpp

  gravity/gravity_potential.cpp

  randomization/global_randomization.cpp
  randomization/normal_randomization.cpp
  randomization/minimal_standard_linear_congruential_generator.cpp
  randomization/minimal_standard_linear_congruential_generator_with_shuffle.cpp

  math/quaternion.cpp
  math/vector.cpp
  math/s2e_math.cpp
  math/interpolation.cpp

  optics/gaussian_beam_base.cpp

  orbit/orbital_elements.cpp
  orbit/kepler_orbit.cpp
  orbit/relative_orbit_models.cpp
  orbit/interpolation_orbit.cpp

  planet_rotation/moon_rotation_utilities.cpp

  time_system/date_time_format.cpp
  time_system/epoch_time.cpp
  time_system/gps_time.cpp

  utilities/endian.cpp
  utilities/slip.cpp
  utilities/quantization.cpp
  utilities/ring_buffer.cpp
)

include(../../common.cmake)
