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
# arguments
import sys

#
# User Settings
#
path_to_logs = '../../data/SampleSat/logs/'
# CSV file path and name used when no arguments
# TODO: Read the latest log file when there is no argument
read_file_tag = '220627_142946'

#
# Read Arguments
#
num_args = len(sys.argv)
if num_args == 2:
  read_file_tag = sys.argv[1]

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
