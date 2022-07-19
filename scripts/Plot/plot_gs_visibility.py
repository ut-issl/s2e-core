#
# Plot Satellite Position with Ground Station Visibility on Miller Projection Map
#
# arg[1] : read_file_tag : time tag for default CSV output log file. ex. 220627_142946
# arg[2] : gs_lat_deg : lattitude of ground station [deg]
# arg[3] : gs_lon_deg : longitude of ground station [deg]
#

#
# Import
#
# plots
import numpy as np
from mpl_toolkits.basemap import Basemap
import matplotlib.pyplot as plt
# csv
import pandas
# local function
from make_miller_projection_map import make_miller_projection_map
# arguments
import sys

#
# User Settings
#
# log file path
path_to_logs = '../../data/SampleSat/logs/'
# CSV file time tag name used when no arguments
# TODO: Read the latest log file when there is no argument
read_file_tag = '220627_142946'
# Ground Station Position used when no arguments
# TODO: Read from the ini file in the logs directory
gs_lat_deg = 26.140837
gs_lon_deg = 127.661483

#
# Read Arguments
#
num_args = len(sys.argv)
if num_args >= 2:
  read_file_tag = sys.argv[1]
if num_args == 4:
  gs_lat_deg = float(sys.argv[2])
  gs_lon_deg = float(sys.argv[3])

#
# CSV file name
#
read_file_name  = path_to_logs + 'logs_' + read_file_tag + '/' + read_file_tag + '_default.csv'

#
# Base Map projection
#
map = make_miller_projection_map()

#
# Data read and edit
#
# Read S2E CSV
df = pandas.read_csv(read_file_name, sep=',', usecols=['lat[rad]', 'lon[rad]', 'is_sc0_visible_from_gs0'])
# satellite position data
sc_lat_deg = df['lat[rad]'].to_numpy() * 180/3.14
sc_lon_deg = df['lon[rad]'].to_numpy() * 180/3.14
sc_map_lon, sc_map_lat = map(sc_lon_deg, sc_lat_deg)

# GS visibility data
gs_visibility = df['is_sc0_visible_from_gs0'].to_numpy()

# Set color
def visibility_color(visibility):
  if visibility == 1: return 'red'
  else: return 'blue'


#
# Plot
#
# Plot ground station
gs_map_lon, gs_map_lat = map(gs_lon_deg, gs_lat_deg)
map.plot(gs_map_lon, gs_map_lat, color='red', marker='*', markersize=12)

# Plot SC position with color
for i in range(len(sc_map_lat)):
  map.plot(sc_map_lon[i], sc_map_lat[i], color=visibility_color(gs_visibility[i]), marker='o', markersize=3)

plt.title('GS Visilibity Analysis: logs_' + read_file_tag)
plt.show()
