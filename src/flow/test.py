import numpy as np
import matplotlib.pyplot as plt
from perlin_noise import PerlinNoise

import numpy as np
import matplotlib.pyplot as plt


def perlin(x, y, seed=0):
    def lerp(a, b, x):
        "linear interpolation"
        return a + x * (b - a)

    def fade(t):
        "6t^5 - 15t^4 + 10t^3"
        return 6 * t**5 - 15 * t**4 + 10 * t**3

    def gradient(h, x, y):
        "grad converts h to the right gradient vector and return the dot product with (x,y)"
        vectors = np.array([[0, 1], [0, -1], [1, 0], [-1, 0]])
        g = vectors[h % 4]
        return g[:, :, 0] * x + g[:, :, 1] * y

    # permutation table
    np.random.seed(seed)
    p = np.arange(256, dtype=int)
    np.random.shuffle(p)
    p = np.stack([p, p]).flatten()
    # coordinates of the top-left
    xi, yi = x.astype(int), y.astype(int)
    # internal coordinates
    xf, yf = x - xi, y - yi
    # fade factors
    u, v = fade(xf), fade(yf)
    # noise components
    n00 = gradient(p[p[xi] + yi], xf, yf)
    n01 = gradient(p[p[xi] + yi + 1], xf, yf - 1)
    n11 = gradient(p[p[xi + 1] + yi + 1], xf - 1, yf - 1)
    n10 = gradient(p[p[xi + 1] + yi], xf - 1, yf)
    # combine noises
    x1 = lerp(n00, n10, u)
    x2 = lerp(n01, n11, u)  # FIX1: I was using n10 instead of n01
    return lerp(x1, x2, v)  # FIX2: I also had to reverse x1 and x2 here


def sigmoid(x, threshold=0.5, steepness=10):
    """
    Sigmoid function for smooth thresholding.

    :param x: Input value or array.
    :param threshold: Center of the sigmoid function.
    :param steepness: Controls the steepness of the sigmoid curve.
    :return: Sigmoid function output.
    """
    return 1 / (1 + np.exp(-steepness * (x - threshold)))


def generate_occupancy_grid_with_perlin_noise(
    size=(100, 100), scale=2, threshold=0.5, steepness=100
):
    """
    Generate a fake occupancy grid using Perlin noise.

    :param size: A tuple (width, height) specifying the size of the grid.
    :param scale: Scale factor for the Perlin noise.
    :param threshold: Threshold to convert the noise into a binary grid.
    :return: An array representing the occupancy grid.
    """
    # Create a grid of coordinates
    lin_x = np.linspace(0, 5, size[0], endpoint=False)
    lin_y = np.linspace(0, 5, size[1], endpoint=False)
    x, y = np.meshgrid(lin_x, lin_y)

    # Apply Perlin noise function
    noise = perlin(x / scale, y / scale)

    # Function
    noise = pow(noise, 2)

    # Normalize to 0-1
    noise = (noise - noise.min()) / (noise.max() - noise.min())

    # Apply sigmoid function for soft thresholding
    grid = sigmoid(noise, threshold, steepness)

    return grid


# Generate and plot the occupancy grid with Perlin noise
occupancy_grid_perlin = generate_occupancy_grid_with_perlin_noise()
plt.imshow(occupancy_grid_perlin, cmap="gray")
plt.title("Occupancy Grid with Perlin Noise")
plt.show()
