#
# Plot Sun Sensor
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
from common import normalize_csv_read_vector
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

measured_sun_direction_c = read_3d_vector_from_csv(read_file_name, 'sun_sensor1_measured_sun_direction_c', '-')
true_sun_direction_b = np.transpose(normalize_csv_read_vector(read_3d_vector_from_csv(read_file_name, 'sun_position_from_spacecraft_b', 'm')))

sun_detection_flag = read_scalar_from_csv(read_file_name, 'sun_sensor1_sun_detected_flag[-]')

#
# Plot
#
fig, axis = plt.subplots(4, 1, squeeze = False, tight_layout = True, sharex = True)
axis[0, 0].plot(time[0], measured_sun_direction_c[0], marker=".", c="red",    label="MEASURED-X")
axis[0, 0].plot(time[0], true_sun_direction_b[0], marker=".", c="orange",  label="TRUE-X")
axis[0, 0].legend(loc = 'upper right')

axis[1, 0].plot(time[0], measured_sun_direction_c[1], marker=".", c="green",  label="MEASURED-Y")
axis[1, 0].plot(time[0], true_sun_direction_b[1], marker=".", c="yellow",  label="TRUE-Y")
axis[1, 0].legend(loc = 'upper right')

axis[2, 0].plot(time[0], measured_sun_direction_c[2], marker=".", c="blue",   label="MEASURED-Z")
axis[2, 0].plot(time[0], true_sun_direction_b[2], marker=".", c="purple",  label="TRUE-Z")
axis[2, 0].legend(loc = 'upper right')

axis[3, 0].plot(time[0], sun_detection_flag[0], marker=".", c="black",  label="SUN-FLAG")
axis[3, 0].legend(loc = 'upper right')
axis[3, 0].set_ylim(0, 1.2)

fig.suptitle("Sun Sensor Sun direction at the body frame")
fig.supylabel("Sun direction")
fig.supxlabel("Time [s]")

# Data save
if args.no_gui:
  plt.savefig(read_file_tag + "_sun_sensor.png") # save last figure only
else:
  plt.show()
