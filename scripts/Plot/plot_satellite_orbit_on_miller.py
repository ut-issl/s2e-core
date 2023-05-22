#
# Plot Satellite Position on Miller Projection Map
#
# arg[1] : read_file_tag : time tag for default CSV output log file. ex. 220627_142946
#

#
# Import
#
# plots
import matplotlib.pyplot as plt
# local function
from make_miller_projection_map import make_miller_projection_map
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
# Base Map projection
#
map = make_miller_projection_map()

#
# Data read and edit
#
# Read S2E CSV
sc_lat_deg = read_scalar_from_csv(read_file_name, 'spacecraft_latitude[rad]') * 180/3.14
sc_lon_deg = read_scalar_from_csv(read_file_name, 'spacecraft_longitude[rad]') * 180/3.14
sc_map_lon, sc_map_lat = map(sc_lon_deg, sc_lat_deg)

#
# Plot
#
# Plot SC position with color
for i in range(len(sc_map_lat)):
  map.plot(sc_map_lon[i], sc_map_lat[i], color='blue', marker='o', markersize=3, linestyle='None')

plt.title('Satellite Orbit on Miller Projection Map: logs_' + read_file_tag)
plt.show()

if args.no_gui:
  plt.savefig(read_file_tag + "_orbit_on_miller.png")
else:
  plt.show()
