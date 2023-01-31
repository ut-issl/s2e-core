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
# ini file
from configparser import ConfigParser
# local function
from make_miller_projection_map import make_miller_projection_map
from common import find_latest_log_tag
from common import read_scalar_from_csv
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

# Read Gound Station position from the ini file in the logs directory
gs_ini_file_name  = path_to_logs + '/' + 'logs_' + read_file_tag + "/SampleGS.ini"
configur = ConfigParser(comment_prefixes=('#', ';', '//'), inline_comment_prefixes=('#', ';', '//'))
configur.read(gs_ini_file_name)
gs_lat_deg = configur.getfloat('GS0', 'latitude_deg')
gs_lon_deg = configur.getfloat('GS0', 'longitude_deg')

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
gs_visibility = np.transpose(read_scalar_from_csv(read_file_name, 'is_sc0_visible_from_gs0'))

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

if args.no_gui:
  plt.savefig(read_file_tag + "_gs_visibility.png")
else:
  plt.show()
