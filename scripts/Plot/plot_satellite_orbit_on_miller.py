#
# Plot Satellite Position on Miller Projection Map
#
# arg[1] : read_file_tag : time tag for default CSV output log file. ex. 220627_142946
#

#
# Import
#
# plots
import numpy as np
from mpl_toolkits.basemap import Basemap
import matplotlib.pyplot as plt
# csv read
import pandas
# local function
from make_miller_projection_map import make_miller_projection_map
from common import find_latest_log_tag
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
# Base Map projection
#
map = make_miller_projection_map()

#
# Data read and edit
#
# Read S2E CSV
df = pandas.read_csv(read_file_name, sep=',', usecols=['lat[rad]', 'lon[rad]'])
# satellite position data
sc_lat_deg = df['lat[rad]'].to_numpy() * 180/3.14
sc_lon_deg = df['lon[rad]'].to_numpy() * 180/3.14
sc_map_lon, sc_map_lat = map(sc_lon_deg, sc_lat_deg)

#
# Plot
#
# Plot SC position with color
for i in range(len(sc_map_lat)):
  map.plot(sc_map_lon[i], sc_map_lat[i], color='blue', marker='o', markersize=3)

plt.title('Satellite Orbit on Miller Projection Map: logs_' + read_file_tag)
plt.show()

if args.no_gui:
  plt.savefig(read_file_tag + "_orbit_on_miller.png")
else:
  plt.show()
