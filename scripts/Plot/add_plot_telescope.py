import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

from common import find_latest_log_tag
from common import add_log_file_arguments
from common import read_3d_vector_from_csv
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

logs = pd.read_csv(read_file_name)

train = pd.read_csv(read_file_name).sample(int(2e3))

star_position = np.array(logs.filter(like='telescope_star_position', axis=1)) #extract position of each star
star_position = star_position[1:, :]

name_lists = np.array(logs.filter(like='telescope_hipparcos_id', axis=1))     #extract hipparcos id of each star
name_lists = name_lists[1, :]

num_of_stars = int(len(star_position[0])/2)
    
def plot_stars_2d(objects, num_of_stars, name_lists):
    """plot location of each star

    Args:
        objects (2darray):  size = (num_of_stars*2, num_of_loc), xy location of each time of each star
        num_of_stars (int): number of stars you want to display
    """
    
    fig, ax = plt.subplots(figsize=(7, 7))
    
    for i in range(0, num_of_stars*2, 2):
        ax.plot(objects[:, i], objects[:, i+1], label=name_lists[int(i/2)])
   
    ax.set_xlabel('X[pix]')
    ax.set_ylabel('Y[pix]')
    ax.legend(loc='best')
    plt.savefig(read_file_tag + "_star_position.png")
    plt.show()
    plt.close()
        
plot_stars_2d(star_position, num_of_stars, name_lists)
