import numpy as np

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
    g = np.array([0.0, 0.0, 9.81]).reshape(3,1)
    while True:
        # x and y velocity are constant, but z velocity changes
        v = v - g*dt

        p = p + v*dt

        t = t + dt

        if (np.linalg.norm(p - c) < R):
            break

        if (t > 1000):
            exit()

    # Equations from: Rotz*Roty*Rotx*[0,0,1]^T = - v/norm(v), with thetaz = roll = 0

    # 0: yaw; 1: pitch; 2: roll
    orientation = np.array([0.0, 0.0, 0.0]).reshape(3,1)
    # pitch
    orientation[1] = np.arctan2(-v[0], -v[2])

    # yaw
    orientation[0] = np.arctan2(-v[1], -v[2]/np.cos(orientation[1]))

    x = np.vstack((p, orientation))

    return (x,t)
