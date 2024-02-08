"""
Generates fake occupancy grids for testing. FIXME: weird "pulsing"?
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from src.grid_pred.noise import perlin, biased_easing


def gen_grid(
    size_y: int = 100,
    size_x: int = 100,
    scale: float = 2,
    time: float = 0,
    seed: float = 0,
) -> np.ndarray[(64, 64), np.int8]:
    """
    Generate a fake occupancy grid.

    :param size_y: Height of the grid.
    :param size_x: Width of the grid.
    :param scale: Scale factor for the Perlin noise.
    :param time: Time parameter for the Perlin noise, to simulate changes over time.
    :param seed: Seed for the Perlin noise generation.
    :return: An array representing the occupancy grid, as uint8.
    """

    # Create a grid of coordinates
    lin_x = np.linspace(0, 5, size_x, endpoint=False)
    lin_y = np.linspace(0, 5, size_y, endpoint=False)
    x, y = np.meshgrid(lin_y, lin_x)

    # Apply Perlin noise function (move upwards in time)
    noise = perlin(x / scale, y / scale + time, seed)

    # Normalize to 0-1
    noise = (noise - noise.min()) / (noise.max() - noise.min())

    # Apply biased easing function
    noise = biased_easing(noise, 5)

    return noise


# Animation function
def update(frame_number):
    global grid
    grid.set_data(
        gen_grid(
            size_y=100,
            size_x=100,
            scale=2,
            time=frame_number * 0.05,
            seed=0,
        )
    )
    return (grid,)


# A simple animation to demonstrate grid generation
if __name__ == "__main__":

    # Set up the figure and axis for animation
    fig, ax = plt.subplots()
    grid = ax.imshow(gen_grid(time=0), cmap="gray", interpolation="nearest")

    # Create 20 frame animation
    ani = FuncAnimation(
        fig,
        update,
        frames=np.arange(0, stop=256),
        interval=16,
        repeat_delay=256,
    )

    plt.show()
