import numpy as np
import cv2
import matplotlib.pyplot as plt
from src.grid_pred.grid_gen import gen_grid

# Occupancy grid
SIZE_X = 64
SIZE_Y = 64

frameA = gen_grid(SIZE_X, SIZE_Y, time=0)
frameB = gen_grid(SIZE_X, SIZE_Y, time=0.2)

plt.imshow(frameA, cmap="gray")
plt.title("Frame A")
plt.show()

plt.imshow(frameB, cmap="gray")
plt.title("Frame B")
plt.show()

# From OpenCV documentation:
# "The function finds an optical flow for each prev pixel using the [67] algorithm so that
# 𝚙𝚛𝚎𝚟(y,x)∼𝚗𝚎𝚡𝚝(y+𝚏𝚕𝚘𝚠(y,x)[1],x+𝚏𝚕𝚘𝚠(y,x)[0])"
#

flow = cv2.calcOpticalFlowFarneback(frameA, frameB, None, 0.5, 3, 15, 3, 5, 1.2, 0)


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


print(flow)

# Assuming `frameA` and `flow` are defined from your previous code:
draw_flow(frameA, flow)
