/**
 * @file example_change_structure.cpp
 * @brief Class to show an example to change satellite structure information
 */

#include "example_change_structure.hpp"

ExampleChangeStructure::ExampleChangeStructure(ClockGenerator* clock_generator, Structure* structure)
    : Component(1, clock_generator), structure_(structure) {}

ExampleChangeStructure::~ExampleChangeStructure() {}

void ExampleChangeStructure::MainRoutine(const int time_count) {
  if (time_count > 1000) {
    // Mass
    structure_->GetToSetKinematicsParameters().SetMass_kg(100.0);
    // Center of gravity
    Vector<3> cg(0.0);
    cg[0] = 0.01;
    cg[1] = -0.01;
    cg[2] = 0.02;
    structure_->GetToSetKinematicsParameters().SetCenterOfGravityVector_b_m(cg);
    // RMM
    Vector<3> rmm(0.0);
    rmm[0] = 0.1;
    rmm[1] = -0.1;
    rmm[2] = 0.2;
    structure_->GetToSetResidualMagneticMoment().SetRmmConstant_b_Am2(rmm);
    // Surface
    structure_->GetToSetSurfaces()[0].SetArea_m2(0.5);
  }
}

std::string ExampleChangeStructure::GetLogHeader() const {
  std::string str_tmp = "";

  return str_tmp;
}

std::string ExampleChangeStructure::GetLogValue() const {
  std::string str_tmp = "";

  return str_tmp;
}
