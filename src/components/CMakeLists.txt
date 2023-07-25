project(COMPONENT)
cmake_minimum_required(VERSION 3.13)

set(SOURCE_FILES
base/component.cpp
base/uart_communication_with_obc.cpp
base/i2c_controller.cpp
base/i2c_target_communication_with_obc.cpp
base/gpio_connection_with_obc.cpp

real/aocs/gyro_sensor.cpp
real/aocs/initialize_gyro_sensor.cpp
real/aocs/magnetometer.cpp
real/aocs/initialize_magnetometer.cpp
real/aocs/magnetorquer.cpp
real/aocs/initialize_magnetorquer.cpp
real/aocs/reaction_wheel_ode.cpp
real/aocs/reaction_wheel.cpp
real/aocs/initialize_reaction_wheel.cpp
real/aocs/reaction_wheel_jitter.cpp
real/aocs/star_sensor.cpp
real/aocs/initialize_star_sensor.cpp
real/aocs/sun_sensor.cpp
real/aocs/initialize_sun_sensor.cpp
real/aocs/gnss_receiver.cpp
real/aocs/initialize_gnss_receiver.cpp

real/cdh/on_board_computer.cpp
real/cdh/on_board_computer_with_c2a.cpp

real/communication/antenna.cpp
real/communication/antenna_radiation_pattern.cpp
real/communication/initialize_antenna.cpp
real/communication/ground_station_calculator.cpp
real/communication/initialize_ground_station_calculator.cpp

examples/example_change_structure.cpp
examples/example_serial_communication_with_obc.cpp
examples/example_serial_communication_for_hils.cpp
examples/example_i2c_controller_for_hils.cpp
examples/example_i2c_target_for_hils.cpp

ideal/force_generator.cpp
ideal/initialize_force_generator.cpp
ideal/torque_generator.cpp
ideal/initialize_torque_generator.cpp

real/mission/telescope.cpp
real/mission/initialize_telescope.cpp

real/power/power_control_unit.cpp
real/power/battery.cpp
real/power/initialize_battery.cpp
real/power/pcu_initial_study.cpp
real/power/initialize_pcu_initial_study.cpp
real/power/solar_array_panel.cpp
real/power/initialize_solar_array_panel.cpp
real/power/csv_scenario_interface.cpp

real/propulsion/simple_thruster.cpp
real/propulsion/initialize_simple_thruster.cpp

real/structure/flexible_structure.cpp

ports/gpio_port.cpp
ports/power_port.cpp
ports/uart_port.cpp
ports/i2c_port.cpp
)

if(USE_HILS)
  set(SOURCE_FILES
    ${SOURCE_FILES}
    hils/ports/hils_uart_port.cpp
    hils/ports/hils_i2c_target_port.cpp
  )
endif()

add_library(${PROJECT_NAME} STATIC
  ${SOURCE_FILES}
)

include(../../common.cmake)
