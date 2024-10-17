"""
Generates fake occupancy grids for testing.
"""

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation

from fake_occupancy.noise import biased_easing, perlin


def gen_grid(
    size_y: int = 100,
    size_x: int = 100,
    scale: float = 2,
    time: float = 0,
    seed: float = 0,
) -> np.ndarray:
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

    # Normalize to 0-1, add epsilon to avoid division by zero issues
    noise = (noise - noise.min()) / (noise.max() - noise.min() + 1e-6)

    # Apply biased easing function
    noise = biased_easing(noise, 5)

    return noise


# Animation function
def update(frame_number, img):
    img.set_data(
        gen_grid(
            size_y=100,
            size_x=100,
            scale=2,
            time=frame_number * 0.05,
            seed=0,
        )
    )
    return (img,)


def main():
    """
    Main function to run the animation of the fake occupancy grid generation.
    """

    # Set up the figure and axis for animation
    fig, ax = plt.subplots()

    # Initialize the grid with the first frame
    initial_grid = gen_grid(time=0)

    # Display the initial grid
    img = ax.imshow(initial_grid, cmap="gray", interpolation="nearest")

    # Create animation
    ani = FuncAnimation(
        fig,
        update,
        fargs=(img,),
        frames=np.arange(0, 256),
        interval=16,
        repeat_delay=256,
    )

    # Show the animation
    plt.show()


# Run the main function if this script is executed directly
if __name__ == "__main__":
    main()
