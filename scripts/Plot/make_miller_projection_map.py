import numpy as np
from mpl_toolkits.basemap import Basemap
import matplotlib.pyplot as plt

# miller projection
def make_miller_projection_map():
  map = Basemap(projection='mill',lon_0=180)
  # plot coastlines, draw label meridians and parallels.
  map.drawcoastlines()
  map.drawparallels(np.arange(-90,90,30),labels=[1,0,0,0])
  map.drawmeridians(np.arange(map.lonmin,map.lonmax+30,60),labels=[0,0,0,1])
  # fill continents 'coral' (with zorder=0), color wet areas 'aqua'
  map.drawmapboundary(fill_color='aqua')
  map.fillcontinents(color='coral',lake_color='aqua')
  return map
