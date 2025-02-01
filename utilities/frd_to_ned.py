#!/usr/bin/env python3

import numpy as np

def frdToNed(x, y, z):
    """
    Function to convert from FRD to NED
    """
    return np.array([x, -y, -z])