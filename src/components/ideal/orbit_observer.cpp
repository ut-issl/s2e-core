/*
 * @file orbit_observer.cpp
 * @brief Ideal component which can observe orbit
 */

#include "orbit_observer.hpp"

OrbitObserver(const int prescaler, ClockGenerator* clock_generator, const Orbit& orbit) : Component(prescaler, clock_generator), orbit_(orbit) {}

void OrbitObserver::MainRoutine(const int time_count) {
  UNUSED(time_count);

  observed_position_i_m_ = orbit_.GetPosition_i_m();
}

std::string OrbitObserver::GetLogHeader() const {
  std::string str_tmp = "";

  std::string head = "orbit_observer_";
  str_tmp += WriteVector(head + "position", "i", "m", 3);

  return str_tmp;
}

std::string OrbitObserver::GetLogValue() const {
  std::string str_tmp = "";

  str_tmp += WriteVector(observed_position_i_m_, 16);

  return str_tmp;
}

OrbitObserver InitializeOrbitObserver(ClockGenerator* clock_generator, const std::string file_name, const Orbit& orbit) {
  // General
  IniAccess ini_file(file_name);

  // CompoBase
  int prescaler = ini_file.ReadInt("COMPONENT_BASE", "prescaler");
  if (prescaler <= 1) prescaler = 1;

  OrbitObserver orbit_observer(prescaler, clock_generator, orbit);

  return orbit_observer;
}
