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

output_magnetic_moment_b_Am2 = read_3d_vector_from_csv(read_file_name, 'magnetorquer1_output_magnetic_moment_b', 'Am2')
output_torque_b_Nm = read_3d_vector_from_csv(read_file_name, 'magnetorquer1_output_torque_b', 'Nm')

#
# Plot
#
unit = ' m'
fig, axis = plt.subplots(2, 1, squeeze = False, tight_layout = True, sharex = True)
axis[0, 0].plot(time[0], output_magnetic_moment_b_Am2[0], marker=".", c="red",    label="X")
axis[0, 0].plot(time[0], output_magnetic_moment_b_Am2[1], marker=".", c="green",  label="Y")
axis[0, 0].plot(time[0], output_magnetic_moment_b_Am2[2], marker=".", c="blue",   label="Z")
axis[0, 0].set(ylabel = "Magnetic Moment Am2")
axis[0, 0].legend(loc = 'upper right')

axis[1, 0].plot(time[0], output_torque_b_Nm[0], marker=".", c="red",    label="X")
axis[1, 0].plot(time[0], output_torque_b_Nm[1], marker=".", c="green",  label="Y")
axis[1, 0].plot(time[0], output_torque_b_Nm[2], marker=".", c="blue",   label="Z")
axis[1, 0].set(ylabel = "Torque Nm")
axis[1, 0].legend(loc = 'upper right')

fig.suptitle("Magnetorquer Output @ Body frame")
fig.supxlabel("Time [s]")

# Data save
if args.no_gui:
  plt.savefig(read_file_tag + "_magnetorquer.png") # save last figure only
else:
  plt.show()
