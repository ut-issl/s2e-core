#
# Plot GNSS Receiver Carrier Phase
#
# arg[1] : read_file_tag : time tag for default CSV output log file. ex. 220627_142946
#

# Import
import matplotlib.pyplot as plt
import numpy as np
import argparse
import os
import csv
from common import find_latest_log_tag
from common import add_log_file_arguments
from common import read_3d_vector_from_csv
from common import read_scalar_from_csv

# Arguments
aparser = argparse.ArgumentParser()
aparser = add_log_file_arguments(aparser)
aparser.add_argument('--no-gui', action='store_true')
args = aparser.parse_args()

# Read Arguments
path_to_logs = args.logs_dir
read_file_tag = args.file_tag
if read_file_tag is None:
    print("file tag does not found. use latest.")
    read_file_tag = find_latest_log_tag(path_to_logs)

print("log: " + read_file_tag)

# CSV file name
read_file_name = os.path.join(path_to_logs, f'logs_{read_file_tag}', f'{read_file_tag}_default.csv')

# Read time data
time = read_scalar_from_csv(read_file_name, 'elapsed_time[s]')

# Plotting function for carrier phases
def plot_carrier_phase(fig_idx, phase_suffix, title):
    plotted = False
    plt.figure(fig_idx)
    for gps_idx in range(32):
        gps_str = f'GPS{gps_idx}'
        column_name = f"{gps_str}_carrier_phase_{phase_suffix}[cycle]"
        try:
            carrier_phase = read_scalar_from_csv(read_file_name, column_name)
            plt.scatter(time[0][1:], carrier_phase[0][1:], marker=".", label=gps_str)
            plotted = True
        except:
            continue
    if plotted:
        plt.title(title)
        plt.xlabel("Time [s]")
        plt.ylabel(f"Carrier Phase {phase_suffix} [cycle]")
        plt.legend(fontsize=7, loc="upper right")
    else:
        plt.close(fig_idx)  # データが無ければ図を閉じる

# Plot each carrier phase if data exists
plot_carrier_phase(0, 1, "GPS Carrier Phase (L1)")
plot_carrier_phase(1, 2, "GPS Carrier Phase (L2)")
plot_carrier_phase(2, 5, "GPS Carrier Phase (L5)")

# Show or save
if args.no_gui:
    plt.savefig(read_file_tag + "_gnss_receiver_carrier_phase.png")  # saves only the last figure
else:
    plt.show()
