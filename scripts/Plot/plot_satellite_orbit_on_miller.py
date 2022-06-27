#
# Plot Satellite Position on Miller Projection Map
#

#
# User Settings
#
# CSC file path and name
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

plt.title('Satellite Orbit on Miller Projection Map')
plt.show()
