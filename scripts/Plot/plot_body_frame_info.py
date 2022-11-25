#
# Plot Sun and Earth direction in Spacecraft body frame
#
# arg[1] : read_file_tag : time tag for default CSV output log file. ex. 220627_142946
#

#
# Import
#
# plots
import numpy as np
import matplotlib.pyplot as plt
# csv read
import pandas
# local function
from common import find_latest_log_tag
from common import normalize_csv_read_vector
# arguments
import argparse

aparser = argparse.ArgumentParser()

aparser.add_argument('--logs-dir', type=str, help='logs directory like "../../data/SampleSat/logs"', default='../../data/SampleSat/logs')
aparser.add_argument('--file-tag', type=str, help='log file tag like 220627_142946')
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
# Read S2E CSV for Sun
csv_data = pandas.read_csv(read_file_name, sep=',', usecols=['SUN_pos_b(X)[m]', 'SUN_pos_b(Y)[m]', 'SUN_pos_b(Z)[m]'])
sun_position_b = np.transpose(np.array([csv_data['SUN_pos_b(X)[m]'].to_numpy(), 
                                        csv_data['SUN_pos_b(Y)[m]'].to_numpy(),
                                        csv_data['SUN_pos_b(Z)[m]'].to_numpy()]))
sun_direction_b = normalize_csv_read_vector(sun_position_b)

# Read S2E CSV for Earth
csv_data = pandas.read_csv(read_file_name, sep=',', usecols=['EARTH_pos_b(X)[m]', 'EARTH_pos_b(Y)[m]', 'EARTH_pos_b(Z)[m]'])
earth_position_b = np.transpose(np.array([csv_data['EARTH_pos_b(X)[m]'].to_numpy(), 
                                          csv_data['EARTH_pos_b(Y)[m]'].to_numpy(),
                                          csv_data['EARTH_pos_b(Z)[m]'].to_numpy()]))
earth_direction_b = normalize_csv_read_vector(earth_position_b)

# Read S2E CSV for Moon
csv_data = pandas.read_csv(read_file_name, sep=',', usecols=['MOON_pos_b(X)[m]', 'MOON_pos_b(Y)[m]', 'MOON_pos_b(Z)[m]'])
moon_position_b = np.transpose(np.array([csv_data['MOON_pos_b(X)[m]'].to_numpy(), 
                                          csv_data['MOON_pos_b(Y)[m]'].to_numpy(),
                                          csv_data['MOON_pos_b(Z)[m]'].to_numpy()]))
moon_direction_b = normalize_csv_read_vector(moon_position_b)

# Read S2E CSV for velocity vector
csv_data = pandas.read_csv(read_file_name, sep=',', usecols=['sat_velocity_b(X)[m/s]', 'sat_velocity_b(Y)[m/s]', 'sat_velocity_b(Z)[m/s]'])
velocity_vector_b = np.transpose(np.array([csv_data['sat_velocity_b(X)[m/s]'].to_numpy(), 
                                           csv_data['sat_velocity_b(Y)[m/s]'].to_numpy(),
                                           csv_data['sat_velocity_b(Z)[m/s]'].to_numpy()]))
velocity_direction_b = normalize_csv_read_vector(velocity_vector_b)

#
# Plot
#
# Base projection
fig = plt.figure()
ax = plt.axes(projection="3d")

# Plot Spacecraft
ax.plot(0,0,0, marker="*", c="black", markersize=10, label="Sat")
ax.quiver(0, 0, 0, 0.5, 0, 0, color='r', label="X") # X-axis
ax.quiver(0, 0, 0, 0, 0.5, 0, color='g', label="Y") # Y-axis
ax.quiver(0, 0, 0, 0, 0, 0.5, color='b', label="Z") # Z-axis

# Plot Celestial bodies
ax.plot(sun_direction_b[0], sun_direction_b[1], sun_direction_b[2], marker="o", c="red", label="Sun")
ax.plot(earth_direction_b[0], earth_direction_b[1], earth_direction_b[2], marker="o", c="blue", label="Earth")
ax.plot(moon_direction_b[0], moon_direction_b[1], moon_direction_b[2], marker="o", c="yellow", label="Moon")

# Plot Velocity
ax.plot(velocity_direction_b[0], velocity_direction_b[1], velocity_direction_b[2], marker="o", color='black', label="SC Velocity")

# Plot setting
ax.set_xlim([-2.0,2.0])
ax.set_ylim([-2.0,2.0])
ax.set_zlim([-2.0,2.0])
ax.legend()
plt.title('Body frame information')
plt.show()

# Data save
if args.no_gui:
  plt.savefig(read_file_tag + "_body_frame.png")
else:
  plt.show()
