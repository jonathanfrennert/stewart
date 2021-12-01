import numpy as np
import sys
import os

def transformation_matrix(x, y, z, roll, pitch, yaw):
    """
    Transformation matrix
    INPUT: SCALARS
    OUTPUT: 4X4 NUMPY ARRAY
    """
    T = np.array([[np.cos(yaw) * np.cos(pitch), -np.sin(yaw)*np.cos(roll) + np.cos(yaw)*np.sin(pitch)*np.sin(roll),  np.sin(yaw)*np.sin(roll)+np.cos(yaw)*np.sin(pitch)*np.cos(roll), x],
                  [np.sin(yaw)*np.cos(pitch),  np.cos(yaw)*np.cos(roll) + np.sin(yaw)*np.sin(pitch)*np.sin(roll), -np.cos(yaw)*np.sin(roll)+np.sin(yaw)*np.sin(pitch)*np.cos(roll), y],
                  [-np.sin(pitch),  np.cos(pitch)*np.sin(roll), np.cos(pitch)*np.cos(yaw), z],
                  [0., 0., 0., 1.]])
    return T

def q_dot(currentPos, currentVel):
    """
    Q dot calculator
    Input: Column vectors
    Output: List
    """

    # Verifying b and p
    base_length = 2
    b = np.array([[-0.101,    0.8, 0.25, 1],
                  [0.101,    0.8, 0.25, 1],
                  [0.743, -0.313, 0.25, 1],
                  [0.642, -0.487, 0.25, 1],
                  [-0.643, -0.486, 0.25, 1],
                  [-0.744, -0.311, 0.25, 1]])

    p = np.array([[-0.642,  0.487, -0.05, 1],
                  [0.642,  0.487, -0.05, 1],
                  [0.743,  0.313, -0.05, 1],
                  [0.101,   -0.8, -0.05, 1],
                  [-0.101,   -0.8, -0.05, 1],
                  [-0.743,  0.313, -0.05, 1]])

    # Current position of the object
    x, y, z, roll, pitch, yaw = currentPos[0][0], currentPos[1][0], currentPos[2][0], currentPos[3][0], currentPos[4][0], currentPos[5][0]

    # Current velocity
    x_dot, y_dot, z_dot, theta_x_dot, theta_y_dot, theta_z_dot =  currentVel[0][0], currentVel[1][0], currentVel[2][0], currentVel[3][0], currentVel[4][0], currentVel[5][0]
    v = np.array([x_dot, y_dot, z_dot]).reshape((-1, 1))
    w = np.array([theta_x_dot, theta_y_dot, theta_z_dot]).reshape((-1, 1))

    T = transformation_matrix(x, y, z, roll, pitch, yaw)
    c = np.array([x, y, z, 1.0]).reshape((-1, 1))

    current_joint_pos = []
    current_joint_vel = []
    for i in range(6):
        # Computation for joint velocities
        length = T @ p[i, :].reshape((-1, 1)) - b[i, :].reshape((-1, 1))
        R_p = T @ p[i, :].reshape((-1, 1)) - c

        l_dot = v + np.cross(w, R_p[:3, :], axis=0)
        joint_pos = np.sqrt(np.sum(length[:3, :]**2)) - base_length
        current_joint_pos.append(joint_pos)

        joint_vel = (1/current_joint_pos[i]) * (l_dot.T @ l_dot)[0][0]
        current_joint_vel.append(joint_vel)

    return current_joint_vel
