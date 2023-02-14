#
# Plot Disturbances
#
# arg[1] : read_file_tag : time tag for default CSV output log file. ex. 220627_142946
#

#
# Import
#
# plots
import matplotlib.pyplot as plt
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

total_torque_b = read_3d_vector_from_csv(read_file_name, 'spacecraft_torque_b', 'Nm')
gg_torque_b = read_3d_vector_from_csv(read_file_name, 'gravity_gradient_torque_b', 'Nm')
srp_torque_b = read_3d_vector_from_csv(read_file_name, 'srp_torque_b', 'Nm')
airdrag_torque_b = read_3d_vector_from_csv(read_file_name, 'air_drag_torque_b', 'Nm')
mag_torque_b = read_3d_vector_from_csv(read_file_name, 'magnetic_disturbance_torque_b', 'Nm')

#
# Plot
#
plt.figure(0)
plt.plot(time[0], gg_torque_b[0], marker=".", c="red",    label="X")
plt.plot(time[0], gg_torque_b[1], marker=".", c="green",  label="Y")
plt.plot(time[0], gg_torque_b[2], marker=".", c="blue",   label="Z")
plt.title("Gravity Gradient torque @ Body frame")
plt.xlabel("Time [s]")
plt.ylabel("Torque [Nm]")
plt.legend()

plt.figure(1)
plt.plot(time[0], srp_torque_b[0], marker=".", c="red",    label="X")
plt.plot(time[0], srp_torque_b[1], marker=".", c="green",  label="Y")
plt.plot(time[0], srp_torque_b[2], marker=".", c="blue",   label="Z")
plt.title("Solar Radiation Pressure torque @ Body frame")
plt.xlabel("Time [s]")
plt.ylabel("Torque [Nm]")
plt.legend()

plt.figure(2)
plt.plot(time[0], airdrag_torque_b[0], marker=".", c="red",    label="X")
plt.plot(time[0], airdrag_torque_b[1], marker=".", c="green",  label="Y")
plt.plot(time[0], airdrag_torque_b[2], marker=".", c="blue",   label="Z")
plt.title("Air drag torque @ Body frame")
plt.xlabel("Time [s]")
plt.ylabel("Torque [Nm]")
plt.legend()

plt.figure(3)
plt.plot(time[0], mag_torque_b[0], marker=".", c="red",    label="X")
plt.plot(time[0], mag_torque_b[1], marker=".", c="green",  label="Y")
plt.plot(time[0], mag_torque_b[2], marker=".", c="blue",   label="Z")
plt.title("Magnetic disturbance torque @ Body frame")
plt.xlabel("Time [s]")
plt.ylabel("Torque [Nm]")
plt.legend()

plt.figure(4)
plt.plot(time[0], total_torque_b[0], marker=".", c="red",    label="X")
plt.plot(time[0], total_torque_b[1], marker=".", c="green",  label="Y")
plt.plot(time[0], total_torque_b[2], marker=".", c="blue",   label="Z")
plt.title("Total torque @ Body frame")
plt.xlabel("Time [s]")
plt.ylabel("Torque [Nm]")
plt.legend()

# Data save
if args.no_gui:
  plt.savefig(read_file_tag + "_disturbance_torque.png") # save last figure only
else:
  plt.show()
