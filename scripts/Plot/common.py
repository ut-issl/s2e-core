import os
import numpy as np
from numpy.linalg import norm
import pandas
import argparse

# 3D model
from stl import mesh
import mpl_toolkits

def find_latest_log_tag(logs_dir):
  dlist = sorted(os.listdir(logs_dir))
  latest_log = None
  for d in dlist:
    if os.path.isfile(logs_dir + d):
      continue
    if not d.startswith("logs_"):
      continue
    latest_log = d
  return latest_log[len("logs_"):] # remove `logs_` prefix

def normalize_csv_read_vector(vector):
  norm_v = norm(vector, axis=1)
  normalized_vector = vector / norm_v[:, None]
  return np.transpose(normalized_vector)

def read_3d_vector_from_csv(read_file_name, header_name, unit):
  name_x = header_name + "_x" + '[' + unit + ']'
  name_y = header_name + "_y" + '[' + unit + ']'
  name_z = header_name + "_z" + '[' + unit + ']'
  csv_data = pandas.read_csv(read_file_name, sep=',', usecols=[name_x, name_y, name_z])
  vector = np.array([csv_data[name_x].to_numpy(), 
                     csv_data[name_y].to_numpy(),
                     csv_data[name_z].to_numpy()])
  return vector

def read_scalar_from_csv(read_file_name, header_name):
  csv_data = pandas.read_csv(read_file_name, sep=',', usecols=[header_name])
  vector = np.array([csv_data[header_name].to_numpy()])
  return vector

def add_log_file_arguments(aparser):
  aparser.add_argument('--logs-dir', type=str, help='logs directory like "../../data/sample/logs"', default='../../data/sample/logs')
  aparser.add_argument('--file-tag', type=str, help='log file tag like 220627_142946')
  return aparser

def add_stl_model(plot_axis, file_name, alpha=0.7, color='orange'):
  sc_mesh = mesh.Mesh.from_file(file_name)
  sc_size_max = np.max([sc_mesh.x.max(),sc_mesh.y.max(),sc_mesh.z.max()])
  scale = 0.7 / sc_size_max
  plot_axis.add_collection3d(mpl_toolkits.mplot3d.art3d.Poly3DCollection(sc_mesh.vectors * scale, alpha=alpha, color=color))

