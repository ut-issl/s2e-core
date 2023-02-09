project(COMPONENT)
cmake_minimum_required(VERSION 3.13)

add_library(${PROJECT_NAME} STATIC
  base/component.cpp
  base/uart_communication_with_obc.cpp
  base/i2c_controller.cpp
  base/obc_i2c_target_communication_base.cpp
  base/gpio_connection_with_obc.cpp

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
  communication/initialize_ground_station_calculator.cpp

  examples/example_change_structure.cpp
  examples/example_serial_communication_with_obc.cpp
  examples/example_serial_communication_for_hils.cpp
  examples/example_i2c_controller_for_hils.cpp
  examples/example_i2c_target_for_hils.cpp

  ideal_components/force_generator.cpp
  ideal_components/initialize_force_generator.cpp
  ideal_components/torque_generator.cpp
  ideal_components/initialize_torque_generator.cpp

  mission/telescope.cpp
  mission/initialize_telescope.cpp

  power/power_control_unit.cpp
  power/battery.cpp
  power/initialize_battery.cpp
  power/pcu_initial_study.cpp
  power/initialize_pcu_initial_study.cpp
  power/solar_array_panel.cpp
  power/initialize_solar_array_panel.cpp
  power/csv_scenario_interface.cpp

  propulsion/simple_thruster.cpp
  propulsion/initialize_simple_thruster.cpp

)

include(../../common.cmake)
