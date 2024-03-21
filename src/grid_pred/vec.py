"""
Vector field functions
"""

import numpy as np
import cv2


def sigmoid(x, alpha: float = 1.0, beta: float = 0.0):
    """
    Sigmoid function, returns value in (0, 1)
    alpha: exponential scaling factor, larger alpha results in steeper function
    beta: horizontal shift, i.e. where the function returns 0.5
    """

    return 1 / (1 + np.exp(-alpha * (x - beta)))


import numpy as np
import cv2


def blur_field(flow, kernel_size=(129, 129), sigmaX=1000000):
    """
    Blurs or averages the magnitudes of vectors in a vector field while preserving their directions.

    Parameters:
    - flow: A 3D numpy array of shape (height, width, 2) representing the vector field.
    - kernel_size: A tuple specifying the size of the Gaussian kernel.
    - sigmaX: Standard deviation in the X direction for the Gaussian kernel. A value of 0 lets OpenCV determine the optimal value.

    Returns:
    - A new 3D numpy array representing the vector field with blurred magnitudes but preserved directions.
    """

    # Calculate the magnitudes and directions of the vectors in the flow field
    magnitudes = np.sqrt(flow[..., 0] ** 2 + flow[..., 1] ** 2)
    directions = np.arctan2(flow[..., 1], flow[..., 0])

    # Apply Gaussian blur to the magnitudes
    blurred_magnitudes = cv2.GaussianBlur(magnitudes, kernel_size, sigmaX)

    # Reconstruct the flow field with blurred magnitudes but original directions
    blurred_flow = np.zeros_like(flow)
    blurred_flow[..., 0] = blurred_magnitudes * np.cos(directions)
    blurred_flow[..., 1] = blurred_magnitudes * np.sin(directions)

    return blurred_flow


import numpy as np
import cv2


def blur_dir(flow, kernel_size=(69, 69), sigmaX=1000):
    """
    Averages the directions of vectors in a vector field while preserving their magnitudes.

    Parameters:
    - flow: A 3D numpy array of shape (height, width, 2) representing the vector field.
    - kernel_size: A tuple specifying the size of the Gaussian kernel for averaging.
    - sigmaX: Standard deviation in the X direction for the Gaussian kernel.

    Returns:
    - A new 3D numpy array representing the vector field with averaged directions.
    """

    # Ensure kernel size is odd
    kernel_size = (kernel_size[0] | 1, kernel_size[1] | 1)

    # Calculate the magnitudes and directions of the vectors in the flow field
    magnitudes = np.sqrt(flow[..., 0] ** 2 + flow[..., 1] ** 2)
    directions = np.arctan2(flow[..., 1], flow[..., 0])

    # Convert directions to Cartesian coordinates
    x_components = magnitudes * np.cos(directions)
    y_components = magnitudes * np.sin(directions)

    # Apply Gaussian blur to the Cartesian components to average the directions
    blurred_x_components = cv2.GaussianBlur(x_components, kernel_size, sigmaX)
    blurred_y_components = cv2.GaussianBlur(y_components, kernel_size, sigmaX)

    # Calculate the averaged directions from the blurred components
    averaged_directions = np.arctan2(blurred_y_components, blurred_x_components)

    # Reconstruct the flow field with original magnitudes but averaged directions
    averaged_flow = np.zeros_like(flow)
    averaged_flow[..., 0] = magnitudes * np.cos(averaged_directions)
    averaged_flow[..., 1] = magnitudes * np.sin(averaged_directions)

    return averaged_flow


import numpy as np
import cv2


def unify_flow_magnitudes(flow):
    """
    Modify the optical flow field such that all vectors in a contiguous shape have the same magnitude.

    Parameters:
    - flow: A 2D numpy array of shape (height, width, 2) representing the flow field.

    Returns:
    - A modified flow field where each contiguous shape of non-zero vectors has the magnitude of the maximum vector in that shape.
    """
    h, w, _ = flow.shape
    magnitude = np.sqrt(flow[..., 0] ** 2 + flow[..., 1] ** 2)
    angle = np.arctan2(flow[..., 1], flow[..., 0])

    # Threshold the magnitude to create a binary image
    thresholded = magnitude > 0

    # Find connected components
    num_labels, labels = cv2.connectedComponents(thresholded.astype(np.uint8))

    # Create an output flow field
    new_flow = np.zeros_like(flow)

    # Iterate through each label to find the maximum magnitude in each blob
    for label in range(1, num_labels):  # Start from 1 to ignore the background
        mask = labels == label
        max_magnitude = np.max(magnitude[mask])

        # Assign this magnitude to all vectors in the blob while preserving direction
        new_flow[..., 0][mask] = np.cos(angle[mask]) * max_magnitude
        new_flow[..., 1][mask] = np.sin(angle[mask]) * max_magnitude

    return new_flow


def advect_grid_bilinear(image, flow):
    """
    Advect an image based on the provided flow field using bilinear interpolation.

    Parameters:
    - image: A 2D numpy array representing the image.
    - flow: A 3D numpy array of shape (height, width, 2) representing the flow field.

    Returns:
    - A new 2D numpy array representing the advected image.
    """
    height, width = image.shape
    advected_image = np.zeros_like(image, dtype=np.float32)

    for y in range(height):
        for x in range(width):
            dx, dy = flow[y, x]

            # Calculate the end position of the flow vector with sub-pixel accuracy
            end_x = x + dx
            end_y = y + dy

            # Find the coordinates of the four surrounding pixels
            x0, y0 = int(end_x), int(end_y)  # Top-left
            x1, y1 = x0 + 1, y0 + 1  # Bottom-right

            # Calculate the weights for each surrounding pixel
            wx1, wy1 = end_x - x0, end_y - y0  # Weights for bottom-right
            wx0, wy0 = 1 - wx1, 1 - wy1  # Weights for top-left

            # Distribute the source pixel's intensity to the surrounding pixels
            for xi, yi, wxi, wyi in [
                (x0, y0, wx0, wy0),
                (x1, y0, wx1, wy0),
                (x0, y1, wx0, wy1),
                (x1, y1, wx1, wy1),
            ]:
                if 0 <= xi < width and 0 <= yi < height:
                    advected_image[yi, xi] += image[y, x] * wxi * wyi

    # Normalize the image to prevent intensity increase due to overlap
    advected_image /= advected_image.max() / image.max()

    # Return the advected image with the original data type
    return advected_image.astype(image.dtype)
