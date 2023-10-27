#
# Plot Ground Position Argument
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
x_data = read_scalar_from_csv(read_file_name, 'telescope_ground_position_angle_z[rad]')
y_data = read_scalar_from_csv(read_file_name, 'telescope_ground_position_angle_y[rad]')
#
# Plot
#
plt.figure(figsize=(10, 7))
plt.scatter(x_data, y_data, s=2, alpha=0.5)
plt.title("Scatter plot of telescope ground position angles")
plt.xlabel("telescope_ground_position_angle_z[rad]")
plt.ylabel("telescope_ground_position_angle_y[rad]")
plt.xlim(-0.3,0.05)
plt.ylim(-0.0025,0.0025)
plt.grid(True)

# Data save
if args.no_gui:
  plt.savefig(read_file_tag + "_ground_position_argument.png") # save last figure only
else:
  plt.show()
