#
# Plot Force Generator
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

ordered_force_b_N = read_3d_vector_from_csv(read_file_name, 'ideal_force_generator_ordered_force_b', 'N')
generated_force_b_N = read_3d_vector_from_csv(read_file_name, 'ideal_force_generator_generated_force_b', 'N')
generated_force_i_N = read_3d_vector_from_csv(read_file_name, 'ideal_force_generator_generated_force_i', 'N')
generated_force_rtn_N = read_3d_vector_from_csv(read_file_name, 'ideal_force_generator_generated_force_rtn', 'N')

# Statistics
# We assume that the component frame and the body frame is same
error_rad_s = generated_force_b_N - ordered_force_b_N
average = [0.0, 0.0, 0.0]
standard_deviation = [0.0, 0.0, 0.0]
for i in range(3):
  average[i] = error_rad_s[i].mean()
  standard_deviation[i] = error_rad_s[i].std()

#
# Plot
#

plt.figure(0)
plt.plot(time[0], generated_force_i_N[0], marker=".", c="red",    label="X")
plt.plot(time[0], generated_force_i_N[1], marker=".", c="green",  label="Y")
plt.plot(time[0], generated_force_i_N[2], marker=".", c="blue",   label="Z")
plt.title("Generated Force @ Inertial frame")
plt.xlabel("Time [s]")
plt.ylabel("Force [N]")
plt.legend()

plt.figure(1)
plt.plot(time[0], generated_force_rtn_N[0], marker=".", c="red",    label="X")
plt.plot(time[0], generated_force_rtn_N[1], marker=".", c="green",  label="Y")
plt.plot(time[0], generated_force_rtn_N[2], marker=".", c="blue",   label="Z")
plt.title("Generated Force @ RTN frame")
plt.xlabel("Time [s]")
plt.ylabel("Force [N]")
plt.legend()

unit = ' N'

fig, axis = plt.subplots(3, 1, squeeze = False, tight_layout = True, sharex = True)
axis[0, 0].plot(time[0], ordered_force_b_N[0], marker=".", c="red",    label="ORDERED-X")
axis[0, 0].plot(time[0], generated_force_b_N[0], marker=".", c="orange",  label="GENERATED-X")
axis[0, 0].legend(loc = 'upper right')
axis[0, 0].text(0.01, 0.99, "Error average:" + format(average[0], '+.2e') + unit, verticalalignment = 'top', transform = axis[0, 0].transAxes)
axis[0, 0].text(0.01, 0.89, "Standard deviation:" + format(standard_deviation[0], '+.2e') + unit, verticalalignment = 'top', transform = axis[0, 0].transAxes)

axis[1, 0].plot(time[0], ordered_force_b_N[1], marker=".", c="green",  label="ORDERED-Y")
axis[1, 0].plot(time[0], generated_force_b_N[1], marker=".", c="yellow",  label="GENERATED-Y")
axis[1, 0].legend(loc = 'upper right')
axis[1, 0].text(0.01, 0.99, "Error average:" + format(average[1], '+.2e') + unit, verticalalignment = 'top', transform = axis[1, 0].transAxes)
axis[1, 0].text(0.01, 0.89, "Standard deviation:" + format(standard_deviation[1], '+.2e') + unit, verticalalignment = 'top', transform = axis[1, 0].transAxes)

axis[2, 0].plot(time[0], ordered_force_b_N[2], marker=".", c="blue",   label="ORDERED-Z")
axis[2, 0].plot(time[0], generated_force_b_N[2], marker=".", c="purple",  label="GENERATED-Z")
axis[2, 0].legend(loc = 'upper right')
axis[2, 0].text(0.01, 0.99, "Error average:" + format(average[2], '+.2e') + unit, verticalalignment = 'top', transform = axis[2, 0].transAxes)
axis[2, 0].text(0.01, 0.89, "Standard deviation:" + format(standard_deviation[2], '+.2e') + unit, verticalalignment = 'top', transform = axis[2, 0].transAxes)

fig.suptitle("Force Generator Output @ Body frame")
fig.supylabel("Force" + unit)
fig.supxlabel("Time [s]")

# Data save
if args.no_gui:
  plt.savefig(read_file_tag + "_force_generator.png") # save last figure only
else:
  plt.show()
