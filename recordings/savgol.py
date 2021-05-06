from scipy.signal import savgol_filter
import numpy as np

filename = 'forward.npz'
data = np.load(filename)
x = data['pos']
dt = 0.1

d_x = (x[1:, :] - x[:-1, :]) / dt
for i in range(d_x.shape[1]): 
	d_x[:, i] = savgol_filter(d_x[:, i], 21, 5)
d_x = np.vstack([d_x, d_x[-1:, :]]) 
dd_x = (d_x[1:, :] - d_x[:-1, :]) / dt
dd_x = np.vstack([dd_x, dd_x[-1:, :]])

import pdb;pdb.set_trace()