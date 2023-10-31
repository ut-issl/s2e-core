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
read_file_name = path_to_logs + '/' + 'logs_' + \
    read_file_tag + '/' + read_file_tag + '_default.csv'

#
# Data read and edit
#
# Read S2E CSV
x_data = read_scalar_from_csv(
    read_file_name, 'telescope_ground_position_x[pix]')
y_data = read_scalar_from_csv(
    read_file_name, 'telescope_ground_position_y[pix]')
#
# Plot
#
plt.figure(figsize=(10, 7))
plt.scatter(x_data, y_data, s=2, alpha=1.0)
plt.title("Scatter plot of ground position in the image sensor")
plt.xlabel("X [pix]")
plt.ylabel("Y [pix]")
plt.grid(True)

# Data save
if args.no_gui:
    # save last figure only
    plt.savefig(read_file_tag + "_ground_position.png")
else:
    plt.show()
