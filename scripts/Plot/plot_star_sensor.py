#
# Plot Star Sensor
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
from common import read_quaternion_from_csv
from common import read_scalar_from_csv
from common import calc_error_angle_from_quaternions
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

measured_quaternion_i2c = read_quaternion_from_csv(read_file_name, 'stt1_measured_quaternion_i2c')
true_quaternion_i2b = read_quaternion_from_csv(read_file_name, 'spacecraft_quaternion_i2b')

# Statistics
error_angle_rad = calc_error_angle_from_quaternions(measured_quaternion_i2c, true_quaternion_i2b)
error_average = error_angle_rad.mean()
standard_deviation = error_angle_rad.std()

#
# Plot
#

fig, axis = plt.subplots(4, 1, squeeze = False, tight_layout = True, sharex = True)
axis[0, 0].plot(time[0], measured_quaternion_i2c[0], marker=".", c="red",    label="MEASURED-X")
axis[0, 0].plot(time[0], true_quaternion_i2b[0], marker=".", c="orange",  label="TRUE-X")
axis[0, 0].legend(loc = 'upper right')

axis[1, 0].plot(time[0], measured_quaternion_i2c[1], marker=".", c="green",    label="MEASURED-Y")
axis[1, 0].plot(time[0], true_quaternion_i2b[1], marker=".", c="yellow",  label="TRUE-Y")
axis[1, 0].legend(loc = 'upper right')

axis[2, 0].plot(time[0], measured_quaternion_i2c[2], marker=".", c="blue",    label="MEASURED-Z")
axis[2, 0].plot(time[0], true_quaternion_i2b[2], marker=".", c="purple",  label="TRUE-Z")
axis[2, 0].legend(loc = 'upper right')

axis[3, 0].plot(time[0], measured_quaternion_i2c[3], marker=".", c="black",    label="MEASURED-W")
axis[3, 0].plot(time[0], true_quaternion_i2b[3], marker=".", c="gray",  label="TRUE-W")
axis[3, 0].legend(loc = 'upper right')

fig.suptitle("Star Sensor Quaternion")
fig.supylabel("Quaternion")
fig.supxlabel("Time [s]")

unit = 'rad'
plt.figure(0)
plt.plot(time[0], error_angle_rad, marker=".", c="red")
plt.title("Error angle \n" + "Error average:" + format(error_average, '+.2e') + unit + "\n Standard deviation:" + format(standard_deviation, '+.2e') + unit)
plt.xlabel("Time [s]")
plt.ylabel("Angle [rad]")

# Data save
if args.no_gui:
  plt.savefig(read_file_tag + "_star_sensor.png") # save last figure only
else:
  plt.show()
