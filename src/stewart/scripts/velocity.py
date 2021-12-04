import numpy as np

def init_velocity(p0, pf, t):
    """
    p0, pf: Column vector of floats
    t: float
    """

    g = 9.81

    # Initial position
    x0 = p0[0, :][0]
    y0 = p0[1, :][0]
    z0 = p0[2, :][0]

    # Final position
    xf = pf[0, :][0]
    yf = pf[1, :][0]
    zf = pf[2, :][0]

    vx0 = (xf - x0)/t
    vy0 = (yf - y0)/t
    vz0 = (zf - z0 + (0.5 * g * t * t))/t

    v0 = np.array([vx0, vy0, vz0])
    return v0.reshape((-1, 1))
