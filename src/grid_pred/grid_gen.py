import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from src.grid_pred.noise import perlin, biased_easing


def gen_grid(
    size_y=100, size_x=100, scale=2, threshold=0.5, steepness=100, time=0, seed=0
):
    """
    Generate a fake occupancy grid.

    :param size: A tuple (width, height) specifying the size of the grid.
    :param scale: Scale factor for the Perlin noise.
    :param threshold: Threshold to convert the noise into a binary grid.
    :return: An array representing the occupancy grid.
    """
    # Create a grid of coordinates
    lin_y = np.linspace(0, 5, size_y, endpoint=False)
    lin_x = np.linspace(0, 5, size_x, endpoint=False)
    y, x = np.meshgrid(lin_y, lin_x)

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
            threshold=0.5,
            steepness=100,
            time=frame_number * 0.1,
            seed=0,
        )
    )
    return (grid,)


# A simple animation to demonstrate grid generation
if __name__ == "__main__":

    # Set up the figure and axis for animation
    fig, ax = plt.subplots()
    grid = ax.imshow(gen_grid(time=0), cmap="gray", interpolation="nearest")

    # Create animation
    ani = FuncAnimation(fig, update, frames=np.arange(0, 20, 1), blit=True)

    plt.show()
