#
# Plot geomaginetic field at the spacecraft position
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

geomagnetic_field_at_spacecraft_position_i_nT = read_3d_vector_from_csv(read_file_name, 'geomagnetic_field_at_spacecraft_position_i', 'nT')
geomagnetic_field_at_spacecraft_position_b_nT = read_3d_vector_from_csv(read_file_name, 'geomagnetic_field_at_spacecraft_position_b', 'nT')

#
# Plot
#
plt.figure(0)
plt.plot(time[0], geomagnetic_field_at_spacecraft_position_i_nT[0], marker=".", c="red",    label="X")
plt.plot(time[0], geomagnetic_field_at_spacecraft_position_i_nT[1], marker=".", c="green",  label="Y")
plt.plot(time[0], geomagnetic_field_at_spacecraft_position_i_nT[2], marker=".", c="blue",   label="Z")
plt.title("Geomagnetic Field at the spacecraft position @ the inertial frame")
plt.xlabel("Time [s]")
plt.ylabel("Magnetic Field [nT]")
plt.legend()

plt.figure(1)
plt.plot(time[0], geomagnetic_field_at_spacecraft_position_b_nT[0], marker=".", c="red",    label="X")
plt.plot(time[0], geomagnetic_field_at_spacecraft_position_b_nT[1], marker=".", c="green",  label="Y")
plt.plot(time[0], geomagnetic_field_at_spacecraft_position_b_nT[2], marker=".", c="blue",   label="Z")
plt.title("Geomagnetic Field at the spacecraft position @ the body frame")
plt.xlabel("Time [s]")
plt.ylabel("Magnetic Field [nT]")
plt.legend()

# Data save
if args.no_gui:
  plt.savefig(read_file_tag + "_geomagnetic_field.png") # save last figure only
else:
  plt.show()
