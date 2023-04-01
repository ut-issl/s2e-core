import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

logs = pd.read_csv('230324_132745_default.csv')

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
    plt.show()
    plt.savefig('Plot_each_star.png')
    plt.close()
        
plot_stars_2d(star_position, num_of_stars, name_lists)