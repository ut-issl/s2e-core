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
from common import read_3d_vector_from_csv
from common import read_scalar_from_csv
# arguments
import argparse

aparser = argparse.ArgumentParser()

aparser.add_argument('--logs-dir', type=str, help='logs directory like "../../data/SampleSat/logs"', default='../../data/SampleSat/logs')
aparser.add_argument('--file-tag', type=str, help='log file tag like 220627_142946')
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
time = read_scalar_from_csv(read_file_name, 'time[sec]')

srp_force_b = read_3d_vector_from_csv(read_file_name, 'srp_force_b', 'N')
airdrag_force_b = read_3d_vector_from_csv(read_file_name, 'airdrag_force_b', 'N')

third_body_acc_i = read_3d_vector_from_csv(read_file_name, 'acc_thirdbody_i', 'm/s2')
geopotential_acc_ecef = read_3d_vector_from_csv(read_file_name, 'a_geop_ecef', 'm/s2')

total_acc_i = read_3d_vector_from_csv(read_file_name, 'sat_acc_i_i', 'm/s^2')

#
# Plot
#
plt.figure(0)
plt.plot(time[0], srp_force_b[0], marker=".", c="red",    label="SRP-X")
plt.plot(time[0], srp_force_b[1], marker=".", c="green",  label="SRP-Y")
plt.plot(time[0], srp_force_b[2], marker=".", c="blue",   label="SRP-Z")
plt.plot(time[0], airdrag_force_b[0], marker=".", c="orange",    label="AIR-X")
plt.plot(time[0], airdrag_force_b[1], marker=".", c="yellow",  label="AIR-Y")
plt.plot(time[0], airdrag_force_b[2], marker=".", c="purple",   label="AIR-Z")
plt.title("Non-gravitational Disturbance Force @ Body frame")
plt.xlabel("Time [s]")
plt.ylabel("Force [N]")
plt.legend()

plt.figure(1)
plt.plot(time[0], third_body_acc_i[0], marker=".", c="red",    label="X")
plt.plot(time[0], third_body_acc_i[1], marker=".", c="green",  label="Y")
plt.plot(time[0], third_body_acc_i[2], marker=".", c="blue",   label="Z")
plt.title("Third body acceleration @ Inertial frame")
plt.xlabel("Time [s]")
plt.ylabel("Acceleration [m/s2]")
plt.legend()

plt.figure(2)
plt.plot(time[0], geopotential_acc_ecef[0], marker=".", c="red",    label="X")
plt.plot(time[0], geopotential_acc_ecef[1], marker=".", c="green",  label="Y")
plt.plot(time[0], geopotential_acc_ecef[2], marker=".", c="blue",   label="Z")
plt.title("Geo-potential acceleration @ ECEF frame")
plt.xlabel("Time [s]")
plt.ylabel("Acceleration [m/s2]")
plt.legend()

plt.figure(3)
plt.plot(time[0], total_acc_i[0], marker=".", c="red",    label="X")
plt.plot(time[0], total_acc_i[1], marker=".", c="green",  label="Y")
plt.plot(time[0], total_acc_i[2], marker=".", c="blue",   label="Z")
plt.title("Total acceleration @ Inertial frame")
plt.xlabel("Time [s]")
plt.ylabel("Acceleration [m/s2]")
plt.legend()

# Data save
if args.no_gui:
  plt.savefig(read_file_tag + "_disturbance_torque.png") # save last figure only
else:
  plt.show()
