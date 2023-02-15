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
# local function
from common import find_latest_log_tag
from common import add_log_file_arguments
from common import normalize_csv_read_vector
from common import read_3d_vector_from_csv
from common import add_stl_model

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
# Read S2E CSV for Sun
sun_position_b = read_3d_vector_from_csv(read_file_name, 'sun_position_from_spacecraft_b', 'm')
sun_direction_b = normalize_csv_read_vector(np.transpose(sun_position_b))

# Read S2E CSV for Earth
earth_position_b = read_3d_vector_from_csv(read_file_name, 'earth_position_from_spacecraft_b', 'm')
earth_direction_b = normalize_csv_read_vector(np.transpose(earth_position_b))

# Read S2E CSV for Moon
moon_position_b = read_3d_vector_from_csv(read_file_name, 'moon_position_from_spacecraft_b', 'm')
moon_direction_b = normalize_csv_read_vector(np.transpose(moon_position_b))

# Read S2E CSV for velocity vector
velocity_vector_b = read_3d_vector_from_csv(read_file_name, 'spacecraft_velocity_b', 'm/s')
velocity_direction_b = normalize_csv_read_vector(np.transpose(velocity_vector_b))

#
# Plot
#
# Base projection
fig = plt.figure()
ax = plt.axes(projection="3d")

# Plot Spacecraft
ax.quiver(0, 0, 0, 0.5, 0, 0, color='r', label="X") # X-axis
ax.quiver(0, 0, 0, 0, 0.5, 0, color='g', label="Y") # Y-axis
ax.quiver(0, 0, 0, 0, 0, 0.5, color='b', label="Z") # Z-axis

# ax.plot(0,0,0, marker="*", c="black", markersize=10, label="Sat") # Please use this when you don't have mesh data
add_stl_model(ax, "./data/sample_6u_cubesat.stl") # Add Spacecraft mesh

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
