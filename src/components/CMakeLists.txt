project(COMPONENT)
cmake_minimum_required(VERSION 3.13)

set(SOURCE_FILES
base/component.cpp
base/uart_communication_with_obc.cpp
base/i2c_controller.cpp
base/i2c_target_communication_with_obc.cpp
base/gpio_connection_with_obc.cpp

real/aocs/gyro_sensor.cpp
real/aocs/magnetometer.cpp
real/aocs/magnetorquer.cpp
real/aocs/reaction_wheel_ode.cpp
real/aocs/reaction_wheel.cpp
real/aocs/reaction_wheel_jitter.cpp
real/aocs/star_sensor.cpp
real/aocs/sun_sensor.cpp
real/aocs/gnss_receiver.cpp
real/aocs/mtq_magnetometer_interference.cpp

real/cdh/on_board_computer.cpp
real/cdh/on_board_computer_with_c2a.cpp

real/communication/antenna.cpp
real/communication/antenna_radiation_pattern.cpp
real/communication/ground_station_calculator.cpp
real/communication/wings_command_sender_to_c2a.cpp

examples/example_change_structure.cpp
examples/example_serial_communication_with_obc.cpp
examples/example_serial_communication_for_hils.cpp
examples/example_i2c_controller_for_hils.cpp
examples/example_i2c_target_for_hils.cpp

ideal/force_generator.cpp
ideal/torque_generator.cpp
ideal/angular_velocity_observer.cpp
ideal/attitude_observer.cpp
ideal/orbit_observer.cpp

real/mission/telescope.cpp

real/power/power_control_unit.cpp
real/power/battery.cpp
real/power/pcu_initial_study.cpp
real/power/solar_array_panel.cpp
real/power/csv_scenario_interface.cpp

real/propulsion/simple_thruster.cpp

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
