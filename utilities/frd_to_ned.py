#!/usr/bin/env python3

import numpy as np

def frd_to_ned(x, y, z):
    return np.array([x, -y, -z])