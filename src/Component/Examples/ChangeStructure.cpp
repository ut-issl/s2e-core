/**
 * @file ChangeStructure.cpp
 * @brief Class to show an example to change satellite structure information
 */

#include "ChangeStructure.hpp"

ChangeStructure::ChangeStructure(ClockGenerator* clock_gen, Structure* structure) : ComponentBase(1, clock_gen), structure_(structure) {}

ChangeStructure::~ChangeStructure() {}

void ChangeStructure::MainRoutine(int count) {
  if (count > 1000) {
    structure_->GetToSetKinematicsParams().SetMass_kg(100.0);
  }
}

std::string ChangeStructure::GetLogHeader() const {
  std::string str_tmp = "";

  return str_tmp;
}

std::string ChangeStructure::GetLogValue() const {
  std::string str_tmp = "";

  return str_tmp;
}
