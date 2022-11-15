import os
import numpy as np
from numpy.linalg import norm

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
