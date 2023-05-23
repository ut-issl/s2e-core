#
# Plot beta angle
#
# arg[1] : read_file_tag : time tag for default CSV output log file. ex. 220627_142946
#

#
# Import
#
import numpy as np
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
read_file_name = path_to_logs + '/' + 'logs_' + read_file_tag + '/' + read_file_tag + '_default.csv'

#
# Data read and edit
#
# Read S2E CSV
time = read_scalar_from_csv(read_file_name, 'elapsed_time[s]')

sun_position_i_m = read_3d_vector_from_csv(read_file_name, 'sun_position_i', 'm').transpose()
sat_position_i_m = read_3d_vector_from_csv(read_file_name, 'spacecraft_position_i', 'm').transpose()
sat_velocity_i_m_s = read_3d_vector_from_csv(read_file_name, 'spacecraft_velocity_i', 'm/s').transpose()

beta_angle_deg = []
for i in range(len(sat_position_i_m)):
    sat_specific_relative_angular_momentum_i_m2_s = np.cross(sat_position_i_m[i], sat_velocity_i_m_s[i])
    beta_angle_deg.append(90 - np.degrees(np.arctan2(np.linalg.norm(np.cross(sun_position_i_m[i], sat_specific_relative_angular_momentum_i_m2_s)), np.dot(sun_position_i_m[i], sat_specific_relative_angular_momentum_i_m2_s))))

#
# Plot
#
plt.plot(time[0], beta_angle_deg, marker=".", c="red")
plt.title("Beta Angle")
plt.xlabel("Time [s]")
plt.ylabel("Angle [deg]")
plt.grid()

# Data save
if args.no_gui:
    plt.savefig(read_file_tag + "_beta_angle.png")  # save last figure only
else:
    plt.show()
