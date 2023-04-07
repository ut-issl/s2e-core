#
# Plot Spacecraft orbit in ECI frame with Sun information
#
# arg[1] : read_file_tag : time tag for default CSV output log file. ex. 220627_142946
# arg[2] : is_animation : true -> generate animation, false -> generate plot
#

#
# Import
#
# plots
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
# from matplotlib.animation import PillowWriter
# csv read
import pandas
# local function
from common import find_latest_log_tag
from common import add_log_file_arguments
from common import normalize_csv_read_vector
from common import read_3d_vector_from_csv
from common import read_scalar_from_csv
# arguments
import argparse
# math
from numpy.linalg import norm

# Arguments
aparser = argparse.ArgumentParser()
aparser = add_log_file_arguments(aparser)
aparser.add_argument('--is-animation', type=bool, help='True: generate animation, False: generate plot', default=False)
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

is_animation = args.is_animation

#
# CSV file name
#
read_file_name  = path_to_logs + '/' + 'logs_' + read_file_tag + '/' + read_file_tag + '_default.csv'

#
# Data read and edit
#
# Read S2E CSV for position
sc_position_i = read_3d_vector_from_csv(read_file_name, 'spacecraft_position_i', 'm')
max_norm_v = max(norm(sc_position_i, axis=0))
shadow_coeff = np.transpose(read_scalar_from_csv(read_file_name, 'shadow_coefficient_at_spacecraft_position'))

# Read S2E CSV for Sun
sun_position_i = read_3d_vector_from_csv(read_file_name, 'sun_position_i', 'm')
sun_direction_i = normalize_csv_read_vector(np.transpose(sun_position_i))

#
# Plot
#
# Base projection
fig = plt.figure()
ax = plt.axes( projection="3d")

# Plot Origin
length_axis_m = max_norm_v * 1.5
ax.plot(0,0,0, marker="*", c="black", markersize=10, label="Origin")
ax.quiver(0, 0, 0, length_axis_m, 0, 0, color='r', label="X") # X-axis
ax.quiver(0, 0, 0, 0, length_axis_m, 0, color='g', label="Y") # Y-axis
ax.quiver(0, 0, 0, 0, 0, length_axis_m, color='b', label="Z") # Z-axis

# Plot Sun direction
sun_direction_i = sun_direction_i * length_axis_m
ax.quiver(0, 0, 0, sun_direction_i[0][0], sun_direction_i[1][0], sun_direction_i[2][0], color='y', label="sun")

# Check eclipse or not
def eclipse_color(shadow_coeff):
  if shadow_coeff >= 0.9: return 'orange'
  else: return 'black'

# Plot or Animation
if is_animation:
  def animate(i):
    line = ax.plot(sc_position_i[0][i], sc_position_i[1][i], sc_position_i[2][i], marker="o", color=eclipse_color(shadow_coeff[i]))
    return line
  
  ax.plot(sc_position_i[0], sc_position_i[1], sc_position_i[2], color='gray', lw=2, linestyle='--')
  animation = FuncAnimation(fig, animate, frames = len(sc_position_i[0]), interval = 5, blit = True)
else:
  for i in range(sc_position_i.shape[1]):
    ax.plot(sc_position_i[0][i], sc_position_i[1][i], sc_position_i[2][i], marker="o", color=eclipse_color(shadow_coeff[i]))

# Plot setting
ax.set_xlim([-length_axis_m, length_axis_m])
ax.set_ylim([-length_axis_m, length_axis_m])
ax.set_zlim([-length_axis_m, length_axis_m])
ax.legend()
ax.set_xlabel("X [m]")
ax.set_ylabel("Y [m]")
ax.set_zlabel("Z [m]")
plt.title('Spacecraft Orbit @ ECI')
plt.show()

# Data save
if args.no_gui and is_animation==False:
  # if is_animation:
    # animation.save(read_file_tag + "_sc_orbit_eci.gif", PillowWriter(fps=30)) # it takes long time...
  # else:
    plt.savefig(read_file_tag + "_sc_orbit_eci.png")
else:
  plt.show()
