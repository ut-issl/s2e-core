#
# Plot celestial body orbit in the inertial frame
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

sun_position_i_m = read_3d_vector_from_csv(read_file_name, 'sun_position_i', 'm')
sun_velocity_i_m_s = read_3d_vector_from_csv(read_file_name, 'sun_velocity_i', 'm/s')
moon_position_i_m = read_3d_vector_from_csv(read_file_name, 'moon_position_i', 'm')
moon_velocity_i_m_s = read_3d_vector_from_csv(read_file_name, 'moon_velocity_i', 'm/s')

#
# Plot
#
plt.figure(0)
plt.plot(time[0], sun_position_i_m[0], marker=".", c="red",    label="X")
plt.plot(time[0], sun_position_i_m[1], marker=".", c="green",  label="Y")
plt.plot(time[0], sun_position_i_m[2], marker=".", c="blue",   label="Z")
plt.title("Sun Position @ the inertial frame")
plt.xlabel("Time [s]")
plt.ylabel("Position [m]")
plt.legend()

plt.figure(1)
plt.plot(time[0], sun_velocity_i_m_s[0], marker=".", c="red",    label="X")
plt.plot(time[0], sun_velocity_i_m_s[1], marker=".", c="green",  label="Y")
plt.plot(time[0], sun_velocity_i_m_s[2], marker=".", c="blue",   label="Z")
plt.title("Sun Velocity @ the inertial frame")
plt.xlabel("Time [s]")
plt.ylabel("Velocity [m]")
plt.legend()

plt.figure(2)
plt.plot(time[0], moon_position_i_m[0], marker=".", c="red",    label="X")
plt.plot(time[0], moon_position_i_m[1], marker=".", c="green",  label="Y")
plt.plot(time[0], moon_position_i_m[2], marker=".", c="blue",   label="Z")
plt.title("Moon Position @ the inertial frame")
plt.xlabel("Time [s]")
plt.ylabel("Position [m]")
plt.legend()

plt.figure(3)
plt.plot(time[0], moon_velocity_i_m_s[0], marker=".", c="red",    label="X")
plt.plot(time[0], moon_velocity_i_m_s[1], marker=".", c="green",  label="Y")
plt.plot(time[0], moon_velocity_i_m_s[2], marker=".", c="blue",   label="Z")
plt.title("Moon Velocity @ the inertial frame")
plt.xlabel("Time [s]")
plt.ylabel("Velocity [m]")
plt.legend()


# Data save
if args.no_gui:
  plt.savefig(read_file_tag + "_disturbance_torque.png") # save last figure only
else:
  plt.show()
