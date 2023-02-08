project(COMPONENT)
cmake_minimum_required(VERSION 3.13)

add_library(${PROJECT_NAME} STATIC
  base_classes/component_base.cpp
  base_classes/obc_communication_base.cpp
  base_classes/i2c_controller_communication_base.cpp
  base_classes/obc_i2c_target_communication_base.cpp
  base_classes/obc_gpio_base.cpp

  aocs/gyro_sensor.cpp
  aocs/initialize_gyro_sensor.cpp
  aocs/magnetometer.cpp
  aocs/initialize_magnetometer.cpp
  aocs/magnetorquer.cpp
  aocs/initialize_magnetorquer.cpp
  aocs/reaction_wheel_ode.cpp
  aocs/reaction_wheel.cpp
  aocs/initialize_reaction_wheel.cpp
  aocs/reaction_wheel_jitter.cpp
  aocs/star_sensor.cpp
  aocs/initialize_star_sensor.cpp
  aocs/sun_sensor.cpp
  aocs/initialize_sun_sensor.cpp
  aocs/gnss_receiver.cpp
  aocs/initialize_gnss_receiver.cpp

  cdh/obc.cpp
  cdh/obc_c2a.cpp

  communication/antenna.cpp
  communication/antenna_radiation_pattern.cpp
  communication/initialize_antenna.cpp
  communication/ground_station_calculator.cpp
  communication/InitGsCalculator.cpp

  examples/example_change_structure.cpp
  examples/example_serial_communication_with_obc.cpp
  examples/example_serial_communication_for_hils.cpp
  examples/example_i2c_controller_for_hils.cpp
  examples/example_i2c_target_for_hils.cpp

  ideal_components/ForceGenerator.cpp
  ideal_components/InitializeForceGenerator.cpp
  ideal_components/TorqueGenerator.cpp
  ideal_components/InitializeTorqueGenerator.cpp

  mission/Telescope.cpp
  mission/InitTelescope.cpp

  power/PCU.cpp
  power/BAT.cpp
  power/InitBat.cpp
  power/PCU_InitialStudy.cpp
  power/InitPcu_InitialStudy.cpp
  power/SAP.cpp
  power/InitSap.cpp
  power/CsvScenarioInterface.cpp

  propulsion/SimpleThruster.cpp
  propulsion/InitSimpleThruster.cpp

)

include(../../common.cmake)
