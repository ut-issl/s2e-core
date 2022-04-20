#pragma once
#include <Component/AOCS/GNSSReceiver.h>
#include <Component/AOCS/RWModel.h>
#include <Component/AOCS/STT.h>
#include <Component/AOCS/SunSensor.h>
#include <Component/Abstract/ExpHils.h>
#include <Component/Abstract/ExpHilsI2cController.h>
#include <Component/Abstract/ExpHilsI2cTarget.h>
#include <Component/CDH/OBC.h>
#include <Component/Power/PCU.h>
#include <Component/Propulsion/SimpleThruster.h>
#include <Dynamics/Dynamics.h>
#include <Interface/HilsInOut/HilsPortManager.h>
#include <Simulation/Spacecraft/Structure/Structure.h>

#include <Component/AOCS/InitGyro.hpp>
#include <Component/AOCS/InitMagSensor.hpp>
#include <Component/AOCS/InitMagTorquer.hpp>
#include <Library/math/Vector.hpp>

#include "../InstalledComponents.hpp"

class OBC;
class PCU;
class Gyro;
class MagSensor;
class STT;
class SunSensor;
class GNSSReceiver;
class MagTorquer;
class RWModel;
class SimpleThruster;

class SampleComponents : public InstalledComponents {
 public:
  SampleComponents(const Dynamics* dynamics, const Structure* structure, const LocalEnvironment* local_env, const GlobalEnvironment* glo_env,
                   const SimulationConfig* config, ClockGenerator* clock_gen, const int sat_id);
  ~SampleComponents();
  libra::Vector<3> GenerateForce_N_b();
  libra::Vector<3> GenerateTorque_Nm_b();
  void LogSetup(Logger& logger);

 private:
  PCU* pcu_;
  OBC* obc_;
  HilsPortManager* hils_port_manager_;
  Gyro* gyro_;
  MagSensor* mag_sensor_;
  STT* stt_;
  SunSensor* sun_sensor_;
  GNSSReceiver* gnss_;
  MagTorquer* mag_torquer_;
  RWModel* rw_;
  SimpleThruster* thruster_;
  // ExpHils* exp_hils_uart_responder_;
  // ExpHils* exp_hils_uart_sender_;
  // ExpHilsI2cController* exp_hils_i2c_controller_;
  // ExpHilsI2cTarget* exp_hils_i2c_target_;

  const SimulationConfig* config_;
  const Dynamics* dynamics_;
  const Structure* structure_;
  const LocalEnvironment* local_env_;
  const GlobalEnvironment* glo_env_;
};
