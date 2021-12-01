import numpy as np
import sys
import os

# Expecting np.arrays for p0, v0, and c;
def get_x(p0, v0, R, c):
    """
    x calculator from a ball trajectory
    Input: ball initial position, initial velocity, hemisphere radius, center of the sphere
    Output: x as well as time

    """
    p = p0
    v = v0
    t = 0.0
    dt = 0.01
    g = 9.81

    while(np.norm(p - c) > R):
        # x and y velocity are constant, but z velocity changes
        v[2] = v[2] - g*dt

        p = p + v*dt

        t = t + dt

        if (t > 10):
            return "Ball either does not intersect or took too long to intersect."

    # Equations from: Rotz*Roty*Rotx*[0,0,1]^T = - v/norm(v), with thetaz = roll = 0

    # 0: yaw; 1: pitch; 2: roll 
    orientation = np.array([0.0, 0.0, 0.0]).reshape(3,1)

    # pitch 
    orientation[1] = np.arctan2(-v[0], -v[2])

    # yaw
    orientation[0] = np.arctan2(-v[1], -v[2]/np.cos(orientation[1]))

    x = np.vstack((p, orientation))

    return (x,t)

p0 = np.array([0, 0, 0]).reshape(3,1)
v0 = np.array([0, 0, 0]).reshape(3,1)
R = 1
c = np.array([0, 0, 2]).reshape(6,1)

print(get_x(p0, v0, R, c))
