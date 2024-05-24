#
# Plot Ground Position in the image sensor
#
# arg[1] : read_file_tag : time tag for default CSV output log file. ex. 220627_142946
#

#
# Import
#
# plots
import matplotlib.pyplot as plt
import pandas as pd
import argparse
# local function
from common import find_latest_log_tag
from common import add_log_file_arguments
from common import read_scalar_from_csv

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
if read_file_tag is None:
    print("file tag does not found. use latest.")
    read_file_tag = find_latest_log_tag(path_to_logs)

print("log: " + read_file_tag)

#
# CSV file name
#
read_file_name = path_to_logs + '/' + 'logs_' + read_file_tag + '/' + read_file_tag + '_default.csv'

#
# Data read and edit
#
# Read S2E CSV
x_center_data = read_scalar_from_csv(read_file_name, 'telescope_ground_position_center_x[pix]')
y_center_data = read_scalar_from_csv(read_file_name, 'telescope_ground_position_center_y[pix]')
x_ymax_data = read_scalar_from_csv(read_file_name, 'telescope_ground_position_ymax_x[pix]')
y_ymax_data = read_scalar_from_csv(read_file_name, 'telescope_ground_position_ymax_y[pix]')
x_ymin_data = read_scalar_from_csv(read_file_name, 'telescope_ground_position_ymin_x[pix]')
y_ymin_data = read_scalar_from_csv(read_file_name, 'telescope_ground_position_ymin_y[pix]')
telescope_flag = read_scalar_from_csv(read_file_name, 'telescope_flag')

# Combine data into a DataFrame
data = pd.DataFrame({
    'x_center': x_center_data,
    'y_center': y_center_data,
    'x_ymax': x_ymax_data,
    'y_ymax': y_ymax_data,
    'x_ymin': x_ymin_data,
    'y_ymin': y_ymin_data,
    'telescope_flag': telescope_flag
})

# Define a function to plot data for a specific flag
def plot_for_flag(flag, data, no_gui):
    filtered_data = data[data['telescope_flag'] == flag]
    
    plt.figure(figsize=(10, 7))
    plt.scatter(filtered_data['x_center'], filtered_data['y_center'], s=2, alpha=1.0, label='Center')
    plt.scatter(filtered_data['x_ymax'], filtered_data['y_ymax'], s=2, alpha=1.0, label='Y Max')
    plt.scatter(filtered_data['x_ymin'], filtered_data['y_ymin'], s=2, alpha=1.0, label='Y Min')
    plt.title(f"Scatter plot of ground position in the image sensor (Flag {flag})")
    plt.xlabel("X [pix]")
    plt.ylabel("Y [pix]")
    plt.legend()
    plt.grid(True)
    
    # Data save
    if no_gui:
        plt.savefig(f"{read_file_tag}_ground_position_flag_{flag}.png")
    else:
        plt.show()

# Plot for each flag
for flag in [1, 2, 3]:
    plot_for_flag(flag, data, args.no_gui)
