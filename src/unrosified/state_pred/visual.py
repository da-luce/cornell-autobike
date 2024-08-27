"""
State prediction visualization
"""

# For visualization
import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import patches
from scipy.stats import gaussian_kde

from src.state_pred import constants as cst

# mpl.use('TkAgg')


# Colors
DARK = '#0B0B0B'
LIGHT = '#C9C9C9'
RED = '#ff3333'
PURPLE = '#9933ff'
ORANGE = '#ff9933'
BLUE = '#33ccff'

fig, ax = plt.subplots(1, 1)

# Configure figure appearance
ax.set_facecolor(DARK)
fig.patch.set_facecolor(DARK)
for spine in ['bottom', 'top', 'left', 'right']:
    ax.spines[spine].set_color(LIGHT)
ax.xaxis.label.set_color(LIGHT)
ax.yaxis.label.set_color(LIGHT)
ax.tick_params(axis='x', colors=LIGHT)
ax.tick_params(axis='y', colors=LIGHT)
ax.grid(linestyle='dashed')
ax.set_aspect('equal', adjustable='box')

# major_spacing = 0.5
# ax.set_xticks(np.arange(-2,5,major_spacing))
# ax.set_xticks(np.arange(-2,5,major_spacing))

# minor_spacing = major_spacing/DIFFERENTIALS[0]
# minor_locator = minor(minor_spacing)
# ax.xaxis.set_minor_locator(minor_locator)

# plt.grid(which="both")


def plot_bike(state):
    """
    Plot a representation of the bike
    """

    print("Plotting bike...")

    x, y, vel_x, vel_y, yaw_angle, steer_angle = state

    wheel_width = 0.1
    wheel_length = 0.6

    rear_axel_x = x - cst.DIST_REAR_AXEL * np.cos(yaw_angle)
    rear_axel_y = y - cst.DIST_REAR_AXEL * np.sin(yaw_angle)

    front_axel_x = x + cst.DIST_FRONT_AXEL * np.cos(yaw_angle)
    front_axel_y = y + cst.DIST_FRONT_AXEL * np.sin(yaw_angle)

    # Plot center line
    ax.plot(
        [rear_axel_x, front_axel_x],
        [rear_axel_y, front_axel_y],
        marker='o',
        color="white",
    )

    # Why neg steering?
    rear_wheel = draw_wheel(
        rear_axel_x, rear_axel_y, wheel_width, wheel_length, RED, yaw_angle
    )
    front_wheel = draw_wheel(
        front_axel_x,
        front_axel_y,
        wheel_width,
        wheel_length,
        BLUE,
        yaw_angle + steer_angle,
    )

    ax.add_patch(rear_wheel)
    ax.add_patch(front_wheel)

    ax.plot([x, x + vel_x], [y, y + vel_y], marker='>', color=ORANGE)

    ax.set_xlim(-2, 5)
    ax.set_ylim(-2, 5)


# pylint: disable=too-many-arguments
def draw_wheel(center_x, center_y, width, length, color, rotation):
    """
    Draw a wheel helper method
    """

    rect = patches.Rectangle(
        (center_x - length / 2, center_y - width / 2),
        length,
        width,
        color=color,
        alpha=1,
    )
    ax.transData.transform([center_x, center_y])
    transform = (
        mpl.transforms.Affine2D().rotate_around(center_x, center_y, rotation)
        + ax.transData
    )
    rect.set_transform(transform)

    return rect


def plot_states(states):
    """
    Plot a list of states
    """

    print("Plotting states...")

    x = list(state[0] for state in states)
    y = list(state[1] for state in states)

    xy = np.vstack([x, y])
    z = gaussian_kde(xy)(xy)

    ax.scatter(x, y, c=z, s=100, alpha=1)


def plot_invalid(states):
    """
    Plot a list of invalid states in red
    """

    x = list(state[0] for state in states)
    y = list(state[1] for state in states)

    ax.scatter(x, y, c=RED, s=100, alpha=0.05)


def show_plot():
    """
    Save the plot locally (work around until figure out window forwarding with Docker)
    """
    # plt.savefig("./state_pred/state_prediction.png", format="png")
    plt.show()
