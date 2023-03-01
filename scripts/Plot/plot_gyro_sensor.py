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

# Statistics
# We assume that the component frame and the body frame is same
error_rad_s = measured_angular_velocity_c_rad_s - true_angular_velocity_b_rad_s
average = [0.0, 0.0, 0.0]
standard_deviation = [0.0, 0.0, 0.0]
for i in range(3):
  average[i] = error_rad_s[i].mean()
  standard_deviation[i] = error_rad_s[i].std()

#
# Plot
#
fig, axis = plt.subplots(3, 1, squeeze = False, tight_layout = True, sharex = True)
axis[0, 0].plot(time[0], measured_angular_velocity_c_rad_s[0], marker=".", c="red",    label="GYRO-X")
axis[0, 0].plot(time[0], true_angular_velocity_b_rad_s[0], marker=".", c="orange",  label="TRUE-X")
axis[0, 0].legend()
axis[0, 0].text(0.01, 0.99, "average:" + format(average[0], '+.2e'), verticalalignment = 'top', transform = axis[0, 0].transAxes)
axis[0, 0].text(0.01, 0.89, "standard deviation:" + format(standard_deviation[0], '+.2e'), verticalalignment = 'top', transform = axis[0, 0].transAxes)

axis[1, 0].plot(time[0], measured_angular_velocity_c_rad_s[1], marker=".", c="green",  label="GYRO-Y")
axis[1, 0].plot(time[0], true_angular_velocity_b_rad_s[1], marker=".", c="yellow",  label="TRUE-Y")
axis[1, 0].legend()
axis[1, 0].text(0.01, 0.99, "average:" + format(average[1], '+.2e'), verticalalignment = 'top', transform = axis[1, 0].transAxes)
axis[1, 0].text(0.01, 0.89, "standard deviation:" + format(standard_deviation[1], '+.2e'), verticalalignment = 'top', transform = axis[1, 0].transAxes)

axis[2, 0].plot(time[0], measured_angular_velocity_c_rad_s[2], marker=".", c="blue",   label="GYRO-Z")
axis[2, 0].plot(time[0], true_angular_velocity_b_rad_s[2], marker=".", c="purple",  label="TRUE-Z")
axis[2, 0].legend()
axis[2, 0].text(0.01, 0.99, "average:" + format(average[2], '+.2e'), verticalalignment = 'top', transform = axis[2, 0].transAxes)
axis[2, 0].text(0.01, 0.89, "standard deviation:" + format(standard_deviation[2], '+.2e'), verticalalignment = 'top', transform = axis[2, 0].transAxes)

fig.suptitle("Gyro Sensor Angular Velocity")
fig.supylabel("Angular Velocity [rad/s]")
fig.supxlabel("Time [s]")

# Data save
if args.no_gui:
  plt.savefig(read_file_tag + "_gyro_sensor.png") # save last figure only
else:
  plt.show()
