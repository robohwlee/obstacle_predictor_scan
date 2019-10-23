#!/usr/bin/env python
# -*- coding: utf-8 -*-

from copy import copy
import numpy as np
from scipy.signal import convolve2d

from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate
from sensor_msgs.msg import LaserScan


def opticalFlowLK(I1g, I2g, window_size, tau=1e-2): # 7.31 add
    '''
    Lucas-Kanade optical flow
    '''
    kernel_x = np.array([[-1., 1.], [-1., 1.]])
    kernel_y = np.array([[-1., -1.], [1., 1.]])
    kernel_t = np.array([[1., 1.], [1., 1.]])#*.25
    w = window_size / 2 # window_size is odd, all the pixels with offset in between [-w, w] are inside the window
    I1g = I1g / 255. # normalize pixels
    I2g = I2g / 255. # normalize pixels
    # Implement Lucas Kanade
    # for each point, calculate I_x, I_y, I_t
    mode = 'same'
    fx = convolve2d(I1g, kernel_x, boundary='symm', mode=mode)
    fy = convolve2d(I1g, kernel_y, boundary='symm', mode=mode)
    ft = convolve2d(I2g, kernel_t, boundary='symm', mode=mode) + convolve2d(I1g, -kernel_t, boundary='symm', mode=mode)
    uv = np.zeros([I1g.shape[0], I1g.shape[1], 2])
    # within window window_size * window_size
    rep_x_list = []
    rep_y_list = []
    u_list = []
    v_list = []
    for i in range(w, I1g.shape[0]-w):
        for j in range(w, I1g.shape[1]-w):
            Ix = fx[i-w:i+w+1, j-w:j+w+1].flatten()
            Iy = fy[i-w:i+w+1, j-w:j+w+1].flatten()
            It = ft[i-w:i+w+1, j-w:j+w+1].flatten()
            b = -It # Av = b <- Ix*u+Iy*v+It = 0
            A = np.stack([Ix, Iy], axis = 1) # 8.1 add. Ix & Iy are columns.
            # if threshold τ is larger than the smallest eigenvalue of A'A:
            nu = np.matmul(np.linalg.pinv(A), b) # v = (A'A)^-1 * (-A'It)
            if 0.3<np.linalg.norm(nu)<1.1:
                uv[i, j, 0] = nu[0]
                uv[i, j, 1] =-nu[1]
                u_list.append(nu[0])
                v_list.append(nu[1])
                rep_x_list.append(j)
                rep_y_list.append(i)
                #print(i,j)

    rep_x = np.average(rep_x_list)
    rep_y = np.average(rep_y_list)
    rep_u = np.average(u_list)
    rep_v = np.average(v_list)
 
    return uv, [rep_x,rep_y,rep_u,-rep_v]   # [August 8, 19:46] Return changed: (ndarray(nx,ny), ndarray(nx,ny)) -> ndarray(nx,ny,2) 


