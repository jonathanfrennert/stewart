import numpy as np
import os
import sys

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

def ikin(goalPos):
    """
    Ikin method
    INPUT: goalPos as numpy column pls!!!!!
    OUTPUT: List
    """
    # All inputs as numpy array NO ROS MESSAGEs
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


    # Acquiring from numpy input array
    # Goal position as a column vector pls
    x, y, z, roll, pitch, yaw = goalPos[0][0], goalPos[1][0], goalPos[2][0], goalPos[3][0], goalPos[4][0], goalPos[5][0]

    T = transformation_matrix(x, y, z, roll, pitch, yaw)
    goal_joint_pos = []
    for i in range(6):
        length = T @ p[i, :].reshape((-1, 1)) - b[i, :].reshape((-1, 1))
        joint_pos = np.sqrt(np.sum(length[:3, :]**2)) - base_length
        goal_joint_pos.append(joint_pos)
    return goal_joint_pos
