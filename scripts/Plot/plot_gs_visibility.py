#
# Plot Satellite Position with Ground Station Visibility on Miller Projection Map
#

#
# User Settings
#
# Ground Station Position
gs_lat_deg = 26.140837
gs_lon_deg = 127.661483
# CSC file name
read_file_name = '../../data/SampleSat/logs/logs_220627_142946/220627_142946_default.csv'


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

plt.title('GS Visilibity Analysis')
plt.show()
