#
# Plot GNSS Receiver Pseudorange
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

plt.figure(0)
for gps_idx in range(32):
  gps_str = 'GPS' + str(gps_idx)
  pseudorange = read_scalar_from_csv(read_file_name, gps_str + '_pseudorange_1[m]')
  plt.scatter(time[0][1:], pseudorange[0][1:], marker=".", label=gps_str)

plt.title("GPS Psuedorange")
plt.xlabel("Time [s]")
plt.ylabel("Psuedorange [m]")
plt.legend(fontsize=7, loc="upper right")


plt.figure(1)
spacecraft_position_ecef_m = read_3d_vector_from_csv(read_file_name, 'spacecraft_position_ecef', 'm')[:, :-1]
for gps_idx in range(32):
  gps_str = 'GPS' + str(gps_idx)
  pseudorange = read_scalar_from_csv(read_file_name, gps_str + '_pseudorange_1[m]')[:, 1:]
  gps_position = read_3d_vector_from_csv(read_file_name, gps_str + '_position_ecef', 'm')[:, 1:]
  geometric_range = np.linalg.norm(gps_position - spacecraft_position_ecef_m, axis=0)
  pseudorange_error = pseudorange - geometric_range
  plt.scatter(time[0][:-1], pseudorange_error, marker=".", label=gps_str)

plt.title("GPS Pseudorange Error")
plt.xlabel("Time [s]")
plt.ylabel("Pseudorange Error [m]")
plt.legend(fontsize=7, loc="upper right")

# Data save
if args.no_gui:
  plt.savefig(read_file_tag + "_gnss_receiver_pseudorange.png") # save last figure only
else:
  plt.show()
