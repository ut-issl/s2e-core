#
# Plot Gyro Sensor
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

measured_angular_velocity_c_rad_s = read_3d_vector_from_csv(read_file_name, 'gyro_sensor1_measured_angular_velocity_c', 'rad/s')
true_angular_velocity_b_rad_s = read_3d_vector_from_csv(read_file_name, 'spacecraft_angular_velocity_b', 'rad/s')

#
# Plot
#
plt.figure(0)

plt.subplot(3, 1, 1)
plt.plot(time[0], measured_angular_velocity_c_rad_s[0], marker=".", c="red",    label="GYRO-X")
plt.plot(time[0], true_angular_velocity_b_rad_s[0], marker=".", c="orange",  label="TRUE-X")
plt.legend()

plt.subplot(3, 1, 2)
plt.plot(time[0], measured_angular_velocity_c_rad_s[1], marker=".", c="green",  label="GYRO-Y")
plt.plot(time[0], true_angular_velocity_b_rad_s[1], marker=".", c="yellow",  label="TRUE-Y")
plt.ylabel("Angular Velocity [rad/s]")
plt.legend()

plt.subplot(3, 1, 3)
plt.plot(time[0], measured_angular_velocity_c_rad_s[2], marker=".", c="blue",   label="GYRO-Z")
plt.plot(time[0], true_angular_velocity_b_rad_s[2], marker=".", c="purple",  label="TRUE-Z")
plt.legend()
plt.xlabel("Time [s]")

plt.suptitle("Gyro Sensor Angular Velocity")

# Data save
if args.no_gui:
  plt.savefig(read_file_tag + "_gyro_sensor.png") # save last figure only
else:
  plt.show()
