#
# Plot solar radiation environment at the spacecraft position
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

solar_radiation_pressure_at_spacecraft_position_N_m2 = read_scalar_from_csv(read_file_name, 'solar_radiation_pressure_at_spacecraft_position[N/m2]')
shadow_coefficient_at_spacecraft_position = read_scalar_from_csv(read_file_name, 'shadow_coefficient_at_spacecraft_position')

#
# Plot
#
fig = plt.figure()
axis1 = fig.add_subplot(111)
axis1.plot(time[0], solar_radiation_pressure_at_spacecraft_position_N_m2[0], marker=".", c="red", label="Pressure N/m2")
axis1.set_ylabel("Solar Radiation Pressure [N/m2]")
axis1.legend()

axis2 = axis1.twinx()
axis2.plot(time[0], shadow_coefficient_at_spacecraft_position[0], marker=".", c="blue", label="Shadow Coefficient")
axis2.set_ylabel("Shadow Coefficient")
axis2.legend()
axis2.set_ylim(0, 2)

plt.title("Solar radiation pressure environment")
plt.xlabel("Time [s]")

# Data save
if args.no_gui:
  plt.savefig(read_file_tag + "_solar_radiation_environment.png") # save last figure only
else:
  plt.show()
