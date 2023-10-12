#
# Plot Magnetorquer
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

angular_velocity_rad_s = read_scalar_from_csv(read_file_name, 'rw1_angular_velocity[rad/s]')
angular_velocity_rpm = read_scalar_from_csv(read_file_name, 'rw1_angular_velocity[rpm]')
angular_acceleration_rad_s2 = read_scalar_from_csv(read_file_name, 'rw1_angular_acceleration[rad/s2]')
target_angular_acceleration_rad_s2 = read_scalar_from_csv(read_file_name, 'rw1_target_angular_acceleration[rad/s2]')

#
# Plot
#
fig, axis = plt.subplots(4, 1, squeeze = False, tight_layout = True, sharex = True)
axis[0, 0].plot(time[0], angular_velocity_rad_s[0], marker=".", c="red", label="rad/s")
axis[0, 0].set(ylabel = "Angular velocity rad/s")
axis[0, 0].legend(loc = 'upper right')

axis[1, 0].plot(time[0], angular_velocity_rpm[0], marker=".", c="green", label="rpm")
axis[1, 0].set(ylabel = "Angular velocity rpm")
axis[1, 0].legend(loc = 'upper right')

axis[2, 0].plot(time[0], angular_acceleration_rad_s2[0], marker=".", c="blue", label="rad/s2")
axis[2, 0].set(ylabel = "Angular acceleration rad/s2")
axis[2, 0].legend(loc = 'upper right')

axis[3, 0].plot(time[0], target_angular_acceleration_rad_s2[0], marker=".", c="blue", label="rad/s2")
axis[3, 0].set(ylabel = "Target angular acceleration rad/s2")
axis[3, 0].legend(loc = 'upper right')

fig.suptitle("Reaction Wheel Output")
fig.supxlabel("Time [s]")

# Data save
if args.no_gui:
  plt.savefig(read_file_tag + "_reaction_wheel.png") # save last figure only
else:
  plt.show()
