/**
 * @file ChangeStructure.cpp
 * @brief Class to show an example to change satellite structure information
 */

#include "ChangeStructure.hpp"

ChangeStructure::ChangeStructure(ClockGenerator* clock_gen, Structure* structure) : ComponentBase(1, clock_gen), structure_(structure) {}

ChangeStructure::~ChangeStructure() {}

void ChangeStructure::MainRoutine(int count) {
  if (count > 1000) {
    // Mass
    structure_->GetToSetKinematicsParams().SetMass_kg(100.0);
    // Center of gravity
    Vector<3> cg(0.0);
    cg[0] = 0.01;
    cg[1] = -0.01;
    cg[2] = 0.02;
    structure_->GetToSetKinematicsParams().SetCenterOfGravityVector_b_m(cg);
    // RMM
    Vector<3> rmm(0.0);
    rmm[0] = 0.1;
    rmm[1] = -0.1;
    rmm[2] = 0.2;
    structure_->GetToSetRMMParams().SetRmmConstant_b_Am2(rmm);
    // Surface
    structure_->GetToSetSurfaces()[0].SetArea(0.5);
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
