#
# Plot Euler Angle of Cantilever Observer
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

euler_angle_cantilever_rad = read_3d_vector_from_csv(read_file_name, 'euler_angular_cantilever_c', 'rad')

#
# Plot
#
unit = ' rad'

fig, axis = plt.subplots(3, 1, squeeze = False, tight_layout = True, sharex = True)
axis[0, 0].plot(time[0][0:1200], euler_angle_cantilever_rad[0][0:1200], c="orange",  label="TRUE-X")
axis[0, 0].legend(loc = 'upper right')

axis[1, 0].plot(time[0][0:1200], euler_angle_cantilever_rad[1][0:1200], c="green",  label="TRUE-Y")
axis[1, 0].legend(loc = 'upper right')

axis[2, 0].plot(time[0][0:1200], euler_angle_cantilever_rad[2][0:1200], c="blue",   label="TRUE-Z")
axis[2, 0].legend(loc = 'upper right')

fig.suptitle("Euler Angle Observer")
fig.supylabel("Euler Angle [" + unit + "]")
fig.supxlabel("Time [s]")

# Data save
if args.no_gui:
  plt.savefig(read_file_tag + "_angular_velocity_observer.png") # save last figure only
else:
  plt.show()
