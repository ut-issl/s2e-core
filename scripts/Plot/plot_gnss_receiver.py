#
# Plot GNSS Receiver TODO: Add plot for velocity, time, geodetic position?
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

measured_position_ecef_m = read_3d_vector_from_csv(read_file_name, 'gnss_receiver1_measured_position_ecef', 'm')
# true_position_ecef_m = read_3d_vector_from_csv(read_file_name, 'spacecraft_position_ecef', 'm')

number_of_visible_satellites = read_scalar_from_csv(read_file_name, 'gnss_receiver1_number_of_visible_satellites')
satellite_visible_flag = read_scalar_from_csv(read_file_name, 'gnss_receiver1_satellite_visible_flag')

# Statistics
#error_m = measured_position_ecef_m[:, 1:] - true_position_ecef_m[:, 1:]
average = [0.0, 0.0, 0.0]
standard_deviation = [0.0, 0.0, 0.0]
#for i in range(3):
#  average[i] = error_m[i].mean()
#  standard_deviation[i] = error_m[i].std()

#
# Plot
#
unit = ' m'
fig, axis = plt.subplots(5, 1, squeeze = False, tight_layout = True, sharex = True)
axis[0, 0].plot(time[0], measured_position_ecef_m[0], marker=".", c="red",    label="MEASURED-X")
#axis[0, 0].plot(time[0], true_position_ecef_m[0], marker=".", c="orange",  label="TRUE-X")
axis[0, 0].text(0.01, 0.99, "Error average:" + format(average[0], '+.2e') + unit, verticalalignment = 'top', transform = axis[0, 0].transAxes)
axis[0, 0].text(0.01, 0.79, "Standard deviation:" + format(standard_deviation[0], '+.2e') + unit, verticalalignment = 'top', transform = axis[0, 0].transAxes)
axis[0, 0].legend(loc = 'upper right')

axis[1, 0].plot(time[0], measured_position_ecef_m[1], marker=".", c="green",  label="MEASURED-Y")
#axis[1, 0].plot(time[0], true_position_ecef_m[1], marker=".", c="yellow",  label="TRUE-Y")
axis[1, 0].text(0.01, 0.99, "Error average:" + format(average[1], '+.2e') + unit, verticalalignment = 'top', transform = axis[1, 0].transAxes)
axis[1, 0].text(0.01, 0.79, "Standard deviation:" + format(standard_deviation[1], '+.2e') + unit, verticalalignment = 'top', transform = axis[1, 0].transAxes)
axis[1, 0].legend(loc = 'upper right')

axis[2, 0].plot(time[0], measured_position_ecef_m[2], marker=".", c="blue",   label="MEASURED-Z")
#axis[2, 0].plot(time[0], true_position_ecef_m[2], marker=".", c="purple",  label="TRUE-Z")
axis[2, 0].text(0.01, 0.99, "Error average:" + format(average[2], '+.2e') + unit, verticalalignment = 'top', transform = axis[2, 0].transAxes)
axis[2, 0].text(0.01, 0.79, "Standard deviation:" + format(standard_deviation[2], '+.2e') + unit, verticalalignment = 'top', transform = axis[2, 0].transAxes)
axis[2, 0].legend(loc = 'upper right')

axis[3, 0].plot(time[0], satellite_visible_flag[0], marker=".", c="black",  label="VISIBLE-FLAG")
axis[3, 0].legend(loc = 'upper right')
axis[3, 0].set_ylim(0, 1.2)

axis[4, 0].plot(time[0], number_of_visible_satellites[0], marker=".", c="black",  label="VISIBLE-SATELLITE-NUMBER")
axis[4, 0].legend(loc = 'upper right')
axis[4, 0].set_ylim(0, max(number_of_visible_satellites[0]) + 2)

fig.suptitle("GNSS Receiver Spacecraft position @ ECEF")
fig.supylabel("Position [m]")
fig.supxlabel("Time [s]")

# Data save
if args.no_gui:
  plt.savefig(read_file_tag + "_gnss_receiver.png") # save last figure only
else:
  plt.show()
