import numpy as np
import random

def gen_positions(h_max, d_min, d_max, num_balls):
    """
    Initial Position Generator 
    Input: Max Height, Minimum distance (float), Maximum distance (float), Number of Balls (int)
    Output: Position of balls

    """
    result = []

    # Loop through number of balls to generate positions
    for i in range(num_balls):
        theta = 2* np.pi * random.random()

        # Make sure to be in valid region
        distance = d_min + (d_max - d_min) * random.random()

        x = distance * np.cos(theta)
        y = distance * np.sin(theta)
        z = random.random() * h_max;
        position = np.array([x, y, z]).reshape(3,1)
        
        result.append(position)

    return tuple(result)