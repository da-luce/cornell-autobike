# For visualization
import matplotlib as mpl
# mpl.use('TkAgg')

import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.ticker import AutoMinorLocator as minor
from scipy.stats import gaussian_kde
import numpy as np
import constants as cst

# Colors
dark = '#0B0B0B'
light = '#C9C9C9'
red = '#ff3333'
purple = '#9933ff'
orange = '#ff9933'
blue = '#33ccff'

fig, ax = plt.subplots(1, 1)

# Configure figure appearance
ax.set_facecolor(dark)
fig.patch.set_facecolor(dark)
for spine in ['bottom', 'top', 'left', 'right']:
    ax.spines[spine].set_color(light)
ax.xaxis.label.set_color(light)
ax.yaxis.label.set_color(light)
ax.tick_params(axis='x', colors=light)
ax.tick_params(axis='y', colors=light)
ax.grid(linestyle='dashed')
ax.set_aspect('equal', adjustable='box')

"""
major_spacing = 0.5
ax.set_xticks(np.arange(-2,5,major_spacing))
ax.set_xticks(np.arange(-2,5,major_spacing))

minor_spacing = major_spacing/DIFFERENTIALS[0]
minor_locator = minor(minor_spacing)
ax.xaxis.set_minor_locator(minor_locator)

plt.grid(which="both")
"""

# Plot a representation of the bike
def plot_bike(state):

    print("Plotting bike...")

    x, y, vel_x, vel_y, yaw_angle, steer_angle = state

    wheel_width = 0.1
    wheel_length = 0.6

    rear_axel_x = x - cst.DIST_REAR_AXEL * np.cos(yaw_angle)
    rear_axel_y = y - cst.DIST_REAR_AXEL * np.sin(yaw_angle)

    front_axel_x = x + cst.DIST_FRONT_AXEL * np.cos(yaw_angle)
    front_axel_y = y + cst.DIST_FRONT_AXEL * np.sin(yaw_angle)

    # Plot center line
    ax.plot([rear_axel_x, front_axel_x], [rear_axel_y, front_axel_y], marker='o', color="white")

    # Why neg steering?
    rear_wheel = draw_wheel(rear_axel_x, rear_axel_y, wheel_width, wheel_length, red, yaw_angle)
    front_wheel = draw_wheel(front_axel_x, front_axel_y, wheel_width, wheel_length, blue, yaw_angle + steer_angle)

    ax.add_patch(rear_wheel)
    ax.add_patch(front_wheel)

    ax.plot([x, x + vel_x], [y, y + vel_y], marker='>', color=orange)

    ax.set_xlim(-2, 5)
    ax.set_ylim(-2, 5)


# Draw a wheel helper method
def draw_wheel(centerX, centerY, width, length, color, rotation):

    rect = patches.Rectangle((centerX - length / 2, centerY - width / 2), length, width, color=color, alpha=1)
    ts = ax.transData.transform([centerX, centerY])
    transform = mpl.transforms.Affine2D().rotate_around(centerX, centerY, rotation) + ax.transData
    rect.set_transform(transform)

    return rect


# Plot a list of states
def plot_states(states):

    print("Plotting states...")

    x = list(state[0] for state in states)
    y = list(state[1] for state in states)

    xy = np.vstack([x, y])
    z = gaussian_kde(xy)(xy)

    ax.scatter(x, y, c=z, s=100, alpha=1)


# Plot a list of invalid states in red
def plot_invalid(states):

    x = list(state[0] for state in states)
    y = list(state[1] for state in states)

    ax.scatter(x, y, c=red, s=100, alpha=0.05)

# Save the plot locally (work around until figure out window forwarding with
# Docker)
def show_plot():
    # plt.savefig("./state_pred/state_prediction.png", format="png")
    plt.show()
