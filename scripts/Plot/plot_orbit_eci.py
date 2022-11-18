#
# Plot Spacecraft orbit in ECI frame with Sun information
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
# Read S2E CSV for position
csv_data = pandas.read_csv(read_file_name, sep=',', usecols=['sat_position_i(X)[m]', 'sat_position_i(Y)[m]', 'sat_position_i(Z)[m]', 'shadow coefficient'])
sc_position_i = np.array([csv_data['sat_position_i(X)[m]'].to_numpy(), 
                          csv_data['sat_position_i(Y)[m]'].to_numpy(),
                          csv_data['sat_position_i(Z)[m]'].to_numpy()])
shadow_coeff = csv_data['shadow coefficient'].to_numpy()

# Read S2E CSV for Sun
csv_data = pandas.read_csv(read_file_name, sep=',', usecols=['SUN_pos_i(X)[m]', 'SUN_pos_i(Y)[m]', 'SUN_pos_i(Z)[m]'])
sun_position_i = np.transpose(np.array([csv_data['SUN_pos_i(X)[m]'].to_numpy(), 
                                        csv_data['SUN_pos_i(Y)[m]'].to_numpy(),
                                        csv_data['SUN_pos_i(Z)[m]'].to_numpy()]))
sun_direction_i = normalize_csv_read_vector(sun_position_i)

#
# Plot
#
# Base projection
fig = plt.figure()
#ax = plt.axes( projection="3d")
ax = fig.add_subplot(111, projection='3d')

# Plot Origin
length_axis_m = 5e6
ax.plot(0,0,0, marker="*", c="black", markersize=10, label="Origin")
ax.quiver(0, 0, 0, length_axis_m, 0, 0, color='r', label="X") # X-axis
ax.quiver(0, 0, 0, 0, length_axis_m, 0, color='g', label="Y") # Y-axis
ax.quiver(0, 0, 0, 0, 0, length_axis_m, color='b', label="Z") # Z-axis

# Plot Sun direction
sun_direction_i = sun_direction_i * length_axis_m
ax.quiver(0, 0, 0, sun_direction_i[0], sun_direction_i[1], sun_direction_i[2], color='y', label="sun") # Z-axis

# Plot spacecraft orbit with eclipse information
def eclipse_color(shadow_coeff):
  if shadow_coeff >= 0.9: return 'orange'
  else: return 'black'

for i in range(sc_position_i.shape[1]):
  ax.plot(sc_position_i[0][i], sc_position_i[1][i], sc_position_i[2][i], marker="o", color=eclipse_color(shadow_coeff[i]))

# Plot setting
ax.legend()
plt.title('Spacecraft Orbit @ ECI')
plt.show()

# Data save
if args.no_gui:
  plt.savefig(read_file_tag + "_sc_orbit_eci.png")
else:
  plt.show()
