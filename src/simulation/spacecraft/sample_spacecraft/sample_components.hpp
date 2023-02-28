/**
 * @file sample_components.hpp
 * @brief An example of user side components management installed on a spacecraft
 */

#ifndef S2E_SIMULATION_SPACECRAFT_SAMPLE_SPACECRAFT_SAMPLE_COMPONENTS_HPP_
#define S2E_SIMULATION_SPACECRAFT_SAMPLE_SPACECRAFT_SAMPLE_COMPONENTS_HPP_

#include <components/examples/example_change_structure.hpp>
#include <components/examples/example_i2c_controller_for_hils.hpp>
#include <components/examples/example_i2c_target_for_hils.hpp>
#include <components/examples/example_serial_communication_for_hils.hpp>
#include <components/ideal/initialize_force_generator.hpp>
#include <components/ideal/initialize_torque_generator.hpp>
#include <components/real/aocs/initialize_gnss_receiver.hpp>
#include <components/real/aocs/initialize_gyro_sensor.hpp>
#include <components/real/aocs/initialize_magnetometer.hpp>
#include <components/real/aocs/initialize_magnetorquer.hpp>
#include <components/real/aocs/initialize_reaction_wheel.hpp>
#include <components/real/aocs/initialize_star_sensor.hpp>
#include <components/real/aocs/initialize_sun_sensor.hpp>
#include <components/real/cdh/on_board_computer.hpp>
#include <components/real/communication/initialize_antenna.hpp>
#include <components/real/power/power_control_unit.hpp>
#include <components/real/propulsion/initialize_simple_thruster.hpp>
#include <dynamics/dynamics.hpp>
#include <library/math/vector.hpp>
#include <simulation/hils/hils_port_manager.hpp>
#include <simulation/spacecraft/structure/structure.hpp>

#include "../installed_components.hpp"

class OnBoardComputer;
class PowerControlUnit;
class GyroSensor;
class Magnetometer;
class StarSensor;
class SunSensor;
class GnssReceiver;
class Magnetorquer;
class ReactionWheel;
class SimpleThruster;
class ForceGenerator;
class TorqueGenerator;

/**
 * @class SampleComponents
 * @brief An example of user side components management class
 */
class SampleComponents : public InstalledComponents {
 public:
  /**
   * @fn SampleComponents
   * @brief Constructor
   */
  SampleComponents(const Dynamics* dynamics, Structure* structure, const LocalEnvironment* local_environment,
                   const GlobalEnvironment* global_environment, const SimulationConfig* config, ClockGenerator* clock_gen, const int spacecraft_id);
  /**
   * @fn ~SampleComponents
   * @brief Destructor
   */
  ~SampleComponents();

  // Override functions for InstalledComponents
  /**
   * @fn GenerateForce_b_N
   * @brief Return force generated by components in unit Newton in body fixed frame
   */
  libra::Vector<3> GenerateForce_b_N();
  /**
   * @fn GenerateTorque_b_Nm
   * @brief Return torque generated by components in unit Newton-meter in body fixed frame
   */
  libra::Vector<3> GenerateTorque_b_Nm();
  /**
   * @fn LogSetup
   * @brief Setup the logger for components
   */
  void LogSetup(Logger& logger);

  // Getter
  inline Antenna& GetAntenna() const { return *antenna_; }

 private:
  PowerControlUnit* pcu_;               //!< Power Control Unit
  OnBoardComputer* obc_;                //!< Onboard Computer
  HilsPortManager* hils_port_manager_;  //!< Port manager for HILS test

  // AOCS
  GyroSensor* gyro_;                   //!< GyroSensor sensor
  Magnetometer* mag_sensor_;           //!< Magnetmeter
  StarSensor* stt_;                    //!< Star sensor
  SunSensor* sun_sensor_;              //!< Sun sensor
  GnssReceiver* gnss_;                 //!< GNSS receiver
  Magnetorquer* mag_torquer_;          //!< Magnetorquer
  ReactionWheel* rw_;                  //!< Reaction Wheel
  SimpleThruster* thruster_;           //!< Thruster
  ForceGenerator* force_generator_;    //!< Ideal Force Generator
  TorqueGenerator* torque_generator_;  //!< Ideal Torque Generator

  // CommGs
  Antenna* antenna_;  //!< Antenna

  // Examples
  // ExampleChangeStructure* change_structure_;  //!< Change structure
  /*
  ExampleSerialCommunicationForHils* exp_hils_uart_responder_;  //!< Example of HILS UART responder
  ExampleSerialCommunicationForHils* exp_hils_uart_sender_;     //!< Example of HILS UART sender
  ExampleI2cControllerForHils* exp_hils_i2c_controller_;        //!< Example of HILS I2C controller
  ExampleI2cTargetForHils* exp_hils_i2c_target_;                //!< Example of HILS I2C target
  */

  // States
  const SimulationConfig* config_;             //!< Simulation settings
  const Dynamics* dynamics_;                   //!< Dynamics information of the spacecraft
  Structure* structure_;                       //!< Structure information of the spacecraft
  const LocalEnvironment* local_environment_;  //!< Local environment information around the spacecraft
  const GlobalEnvironment* glo_env_;           //!< Global environment information
};

#endif  // S2E_SIMULATION_SPACECRAFT_SAMPLE_SPACECRAFT_SAMPLE_COMPONENTS_HPP_
