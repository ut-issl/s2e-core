#
# Plot Ground station calculator
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
sc0_visible_flag = read_scalar_from_csv(read_file_name, 'ground_station0_sc0_visible_flag')
max_bitrate_Mbps = read_scalar_from_csv(read_file_name, 'gs_calculator_max_bitrate[Mbps]')
receive_margin_dB = read_scalar_from_csv(read_file_name, 'gs_calculator_receive_margin[dB]')

#
# Plot
#
unit = ' Nm'

fig, axis = plt.subplots(3, 1, squeeze = False, tight_layout = True, sharex = True)
axis[0, 0].plot(time[0], sc0_visible_flag[0], marker=".", c="red",    label="SC0-VSIBLE")
axis[0, 0].legend(loc = 'upper right')
axis[0, 0].set(ylabel = 'Visible Flag')

axis[1, 0].plot(time[0], max_bitrate_Mbps[0], marker=".", c="green",  label="Mbps")
axis[1, 0].legend(loc = 'upper right')
axis[1, 0].set(ylabel = 'Max bitrate [Mbps]')

axis[2, 0].plot(time[0], receive_margin_dB[0], marker=".", c="blue",   label="dB")
axis[2, 0].legend(loc = 'upper right')
axis[2, 0].set(ylabel = 'Receive margin [dB]')

fig.suptitle("Ground Station")
fig.supxlabel("Time [s]")

# Data save
if args.no_gui:
  plt.savefig(read_file_tag + "_ground_station_calculator.png") # save last figure only
else:
  plt.show()
