import numpy as np
import cv2
import matplotlib.pyplot as plt
from src.grid_pred.grid_gen import gen_grid

# Occupancy grid
SIZE_X = 64
SIZE_Y = 64

# From OpenCV documentation:
# "The function finds an optical flow for each prev pixel using the [67]
# algorithm so that
# prev(y,x) = next(y + flow(y,x)[1], x + flow(y,x)[0])"
#


# Function to plot optical flow vectors
def draw_flow(img, flow, step=4, scale=10):
    """
    Draw optical flow vectors on the image.
    :param img: Grayscale image to draw on.
    :param flow: Optical flow vectors (shape: height x width x 2).
    :param step: Space between vectors. Lower values mean more density.
    :param scale: Scale factor for visualizing the flow vectors.
    """

    # Create a figure
    plt.figure(figsize=(10, 10))

    i, j = 10, 10  # Example pixel coordinates
    flow_vector = flow[i, j]

    # Display the image
    plt.imshow(img, cmap="gray", interpolation="nearest")
    plt.title("Optical Flow Vectors")
    plt.axis("off")

    # Create a meshgrid for the flow
    h, w = img.shape[:2]
    y, x = np.mgrid[0:h:step, 0:w:step].reshape(2, -1)
    fx, fy = flow[y, x].T * scale

    # Plot the flow vectors
    plt.quiver(
        x,
        y,
        fx,
        fy,
        color="r",
        angles="xy",
        scale_units="xy",
        scale=1,
        headwidth=3,
        headlength=4,
    )

    plt.show()


def draw_dense_quiver_flow(img, flow, step=1, scale=1e6):
    """
    Draw dense optical flow vectors using quiver plot.
    :param img: Grayscale image to draw on.
    :param flow: Optical flow vectors (shape: height x width x 2).
    :param step: Space between vectors. Lower values mean more density.
    :param scale: Scale factor for visualizing the flow vectors.
    """
    # Create a figure for plotting
    fig, ax = plt.subplots(figsize=(6.4, 4.8))
    ax.imshow(img, cmap="gray", interpolation="nearest")
    ax.set_title("Optical Flow Vectors")
    ax.axis("off")

    # Generate a grid of points
    h, w = img.shape
    y, x = np.mgrid[step / 2 : h : step, step / 2 : w : step].reshape(2, -1).astype(int)
    fx, fy = flow[y, x].T

    # Plotting the flow vectors
    ax.quiver(
        x, y, fx, fy, color="r", angles="xy", scale_units="xy", scale=scale, width=0.005
    )

    return fig


def scale_flow_vectors(flow):
    """
    Scale the optical flow vectors so that the smallest non-zero vector magnitude becomes 1.

    :param flow: A NumPy array of shape (height, width, 2) containing the flow vectors.
    :return: A NumPy array of the scaled flow vectors.
    """
    # Calculate magnitudes of the flow vectors
    magnitudes = np.sqrt(flow[..., 0] ** 2 + flow[..., 1] ** 2)

    # Find the smallest non-zero magnitude
    min_nonzero_magnitude = np.min(magnitudes[np.nonzero(magnitudes)])

    # Avoid division by zero in case all vectors are zero
    if min_nonzero_magnitude == 0:
        return flow  # Return original flow if all vectors are zero

    # Calculate scaling factor
    scaling_factor = 1 / min_nonzero_magnitude

    # Scale the flow vectors
    scaled_flow = flow * scaling_factor

    return scaled_flow


if __name__ == "__main__":

    frameA = gen_grid(SIZE_X, SIZE_Y, time=0)
    frameB = gen_grid(SIZE_X, SIZE_Y, time=0.2)
    flow = cv2.calcOpticalFlowFarneback(
        frameA,
        frameB,
        None,
        pyr_scale=0.5,
        levels=3,
        winsize=15,
        iterations=3,
        poly_n=5,
        poly_sigma=1.1,
        flags=cv2.OPTFLOW_FARNEBACK_GAUSSIAN,
    )

    flow.astype("int16").tofile("./temp")

    plt.figure("Frame A")
    plt.imshow(frameA, cmap="gray")

    plt.figure("Frame B")
    plt.imshow(frameB, cmap="gray")

    flows = draw_dense_quiver_flow(frameA, flow)

    plt.show()
