#
# Plot orbit observer
#
# arg[1] : read_file_tag : time tag for default CSV output log file. ex. 220627_142946
#

#
# Import
#
# plots
import matplotlib.pyplot as plt
# numpy
import numpy as np
# local function
from common import find_latest_log_tag
from common import add_log_file_arguments
from common import read_3d_vector_from_csv
from common import read_scalar_from_csv
# arguments
import argparse

# Arguments
aparser = argparse.ArgumentParser()
aparser = add_log_file_arguments(aparser)
aparser.add_argument('--no-gui', action='store_true')
args = aparser.parse_args()

#
# Read Arguments
#
# log file path
path_to_logs = args.logs_dir

read_file_tag = args.file_tag
if read_file_tag == None:
  print("file tag does not found. use latest.")
  read_file_tag = find_latest_log_tag(path_to_logs)

print("log: " + read_file_tag)

#
# CSV file name
#
read_file_name  = path_to_logs + '/' + 'logs_' + read_file_tag + '/' + read_file_tag + '_default.csv'

#
# Data read and edit
#
# Read S2E CSV
time = read_scalar_from_csv(read_file_name, 'elapsed_time[s]')

measured_position_eci_m = read_3d_vector_from_csv(read_file_name, 'orbit_observer_position_i', 'm')
true_position_eci_m = read_3d_vector_from_csv(read_file_name, 'spacecraft_position_i', 'm')

measured_velocity_eci_m_s = read_3d_vector_from_csv(read_file_name, 'orbit_observer_velocity_i', 'm/s')
true_velocity_eci_m_s = read_3d_vector_from_csv(read_file_name, 'spacecraft_velocity_i', 'm/s')

# Statistics
position_error_m = measured_position_eci_m[:, 1:] - true_position_eci_m[:, 1:]
position_average = [0.0, 0.0, 0.0]
position_standard_deviation = [0.0, 0.0, 0.0]
velocity_error_m = measured_velocity_eci_m_s[:, 1:] - true_velocity_eci_m_s[:, 1:]
velocity_average = [0.0, 0.0, 0.0]
velocity_standard_deviation = [0.0, 0.0, 0.0]
for i in range(3):
  position_average[i] = position_error_m[i].mean()
  position_standard_deviation[i] = position_error_m[i].std()
  velocity_average[i] = velocity_error_m[i].mean()
  velocity_standard_deviation[i] = velocity_error_m[i].std()

#
# Plot
#
unit = ' m'
fig, axis = plt.subplots(3, 1, squeeze = False, tight_layout = True, sharex = True)
axis[0, 0].plot(time[0], measured_position_eci_m[0], marker=".", c="red",    label="MEASURED-X")
axis[0, 0].plot(time[0], true_position_eci_m[0], marker=".", c="orange",  label="TRUE-X")
axis[0, 0].text(0.01, 0.99, "Error average:" + format(position_average[0], '+.2e') + unit, verticalalignment = 'top', transform = axis[0, 0].transAxes)
axis[0, 0].text(0.01, 0.79, "Standard deviation:" + format(position_standard_deviation[0], '+.2e') + unit, verticalalignment = 'top', transform = axis[0, 0].transAxes)
axis[0, 0].legend(loc = 'upper right')

axis[1, 0].plot(time[0], measured_position_eci_m[1], marker=".", c="green",  label="MEASURED-Y")
axis[1, 0].plot(time[0], true_position_eci_m[1], marker=".", c="yellow",  label="TRUE-Y")
axis[1, 0].text(0.01, 0.99, "Error average:" + format(position_average[1], '+.2e') + unit, verticalalignment = 'top', transform = axis[1, 0].transAxes)
axis[1, 0].text(0.01, 0.79, "Standard deviation:" + format(position_standard_deviation[1], '+.2e') + unit, verticalalignment = 'top', transform = axis[1, 0].transAxes)
axis[1, 0].legend(loc = 'upper right')

axis[2, 0].plot(time[0], measured_position_eci_m[2], marker=".", c="blue",   label="MEASURED-Z")
axis[2, 0].plot(time[0], true_position_eci_m[2], marker=".", c="purple",  label="TRUE-Z")
axis[2, 0].text(0.01, 0.99, "Error average:" + format(position_average[2], '+.2e') + unit, verticalalignment = 'top', transform = axis[2, 0].transAxes)
axis[2, 0].text(0.01, 0.79, "Standard deviation:" + format(position_standard_deviation[2], '+.2e') + unit, verticalalignment = 'top', transform = axis[2, 0].transAxes)
axis[2, 0].legend(loc = 'upper right')

fig.suptitle("Orbit observer position results @ ECI")
fig.supylabel("Position [m]")
fig.supxlabel("Time [s]")

unit = ' m/s'
fig, axis = plt.subplots(3, 1, squeeze = False, tight_layout = True, sharex = True)
axis[0, 0].plot(time[0], measured_velocity_eci_m_s[0], marker=".", c="red",    label="MEASURED-X")
axis[0, 0].plot(time[0], true_velocity_eci_m_s[0], marker=".", c="orange",  label="TRUE-X")
axis[0, 0].text(0.01, 0.99, "Error average:" + format(velocity_average[0], '+.2e') + unit, verticalalignment = 'top', transform = axis[0, 0].transAxes)
axis[0, 0].text(0.01, 0.79, "Standard deviation:" + format(velocity_standard_deviation[0], '+.2e') + unit, verticalalignment = 'top', transform = axis[0, 0].transAxes)
axis[0, 0].legend(loc = 'upper right')

axis[1, 0].plot(time[0], measured_velocity_eci_m_s[1], marker=".", c="green",  label="MEASURED-Y")
axis[1, 0].plot(time[0], true_velocity_eci_m_s[1], marker=".", c="yellow",  label="TRUE-Y")
axis[1, 0].text(0.01, 0.99, "Error average:" + format(velocity_average[1], '+.2e') + unit, verticalalignment = 'top', transform = axis[1, 0].transAxes)
axis[1, 0].text(0.01, 0.79, "Standard deviation:" + format(velocity_standard_deviation[1], '+.2e') + unit, verticalalignment = 'top', transform = axis[1, 0].transAxes)
axis[1, 0].legend(loc = 'upper right')

axis[2, 0].plot(time[0], measured_velocity_eci_m_s[2], marker=".", c="blue",   label="MEASURED-Z")
axis[2, 0].plot(time[0], true_velocity_eci_m_s[2], marker=".", c="purple",  label="TRUE-Z")
axis[2, 0].text(0.01, 0.99, "Error average:" + format(velocity_average[2], '+.2e') + unit, verticalalignment = 'top', transform = axis[2, 0].transAxes)
axis[2, 0].text(0.01, 0.79, "Standard deviation:" + format(velocity_standard_deviation[2], '+.2e') + unit, verticalalignment = 'top', transform = axis[2, 0].transAxes)
axis[2, 0].legend(loc = 'upper right')

fig.suptitle("Orbit observer velocity results @ ECI")
fig.supylabel("Velocity [m/s]")
fig.supxlabel("Time [s]")

# Data save
if args.no_gui:
  plt.savefig(read_file_tag + "_orbit_observer.png") # save last figure only
else:
  plt.show()
