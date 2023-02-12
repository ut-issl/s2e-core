project(LIBRARY)
cmake_minimum_required(VERSION 3.13)

add_library(${PROJECT_NAME} STATIC
  geodesy/geodetic_position.cpp
  
  igrf/igrf.cpp

  inih/ini.c
  inih/cpp/INIReader.cpp

  math/global_randomization.cpp
  math/normal_randomization.cpp
  math/quantization.cpp
  math/quaternion.cpp
  math/randomize0.cpp
  math/randomize1.cpp
  math/vector.cpp
  math/s2e_math.cpp

  nrlmsise00/wrapper_nrlmsise00.cpp

  optics/gaussian_beam_base.cpp

  orbit/orbital_elements.cpp
  orbit/kepler_orbit.cpp
  orbit/relative_orbit_models.cpp

  sgp4/sgp4ext.cpp
  sgp4/sgp4io.cpp
  sgp4/sgp4unit.cpp

  utilities/endian.cpp
  utilities/slip.cpp
)

include(../../common.cmake)
