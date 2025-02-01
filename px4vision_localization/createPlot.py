#!/usr/bin/env python3

import matplotlib.pyplot as plt

import matplotlib.pyplot as plt
import numpy as np

# Initialize figure and 3D axis
fig_ekf = plt.figure()
ax_ekf = fig_ekf.add_subplot(111, projection='3d')


fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')


# Enable interactive mode
plt.ion()
plt.show()

def update_ekf_plot(state):
    """
    Update the 3D plot with new state points.
    """
    global ax_ekf

    # Extract X, Y, Z coordinates
    x, y, z = state[0], state[1], state[2]

    # Clear the plot and re-plot the trajectory
    ax_ekf.clear()
    ax_ekf.set_xlabel('X')
    ax_ekf.set_ylabel('Y')
    ax_ekf.set_zlabel('Z')
    ax_ekf.set_title('UAV EKF State Trajectory')

    # Keep a history of past states
    if not hasattr(update_ekf_plot, "history"):
        update_ekf_plot.history = {"x": [], "y": [], "z": []}

    update_ekf_plot.history["x"].append(x)
    update_ekf_plot.history["y"].append(y)
    update_ekf_plot.history["z"].append(z)

    ax_ekf.plot(update_ekf_plot.history["x"], update_ekf_plot.history["y"], update_ekf_plot.history["z"], marker="o", markersize=3, color = 'blue', linestyle="-")

    # Refresh the figure
    plt.draw()
    plt.pause(0.1)

def update_vlp_plot(state):
    """
    Update the 3D plot with new state points.
    """
    global ax

    # Extract X, Y, Z coordinates
    x, y, z = state[0], state[1], state[2]

    # Clear the plot and re-plot the trajectory
    ax.clear()
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('UAV VLP State Trajectory')

    # Keep a history of past states
    if not hasattr(update_vlp_plot, "history"):
        update_vlp_plot.history = {"x": [], "y": [], "z": []}

    update_vlp_plot.history["x"].append(x)
    update_vlp_plot.history["y"].append(y)
    update_vlp_plot.history["z"].append(z)

    ax.plot(update_vlp_plot.history["x"], update_vlp_plot.history["y"], update_vlp_plot.history["z"], marker="o", markersize=3, color = 'blue', linestyle="-")

    # Refresh the figure
    plt.draw()
    plt.pause(0.1)
