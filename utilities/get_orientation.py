#!/usr/bin/env python3
import numpy as np

def getOrientation(angular_velocity, dt):
    """
    Function to get orientation from angular velocity
    """
    #Initial orientation
    orientation = np.array([0, 0, 0])
    phi, theta, psi = orientation
    p, q, r = angular_velocity

    #Rate of change of roll, pitch and yaw
    dphi = p + q * np.sin(phi) * np.tan(theta) + r * np.cos(phi) * np.tan(theta)
    dtheta = q * np.cos(phi) - r * np.sin(phi)
    dpsi = q * np.sin(phi) / np.cos(theta) + r * np.cos(phi) / np.cos(theta)

    #Updating orientation
    orientation[0] += dphi * dt
    orientation[1] += dtheta * dt
    orientation[2] += dpsi * dt

    return orientation