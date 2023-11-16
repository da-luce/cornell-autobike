import cv2
import numpy as np

# Occupancy grid dimensions
GRID_WIDTH = 8
GRID_HEIGHT = 6

# Lucas-Kanade parameters
lk_params = dict(
    # The size of our window
    winSize=(GRID_WIDTH, GRID_HEIGHT),
    # Level of subsampling?
    maxLevel=2,
    # Termination criteria, after 10 iterations or when the accuracy reaches 0.03
    criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03),
)


def preprocess_grid(grid):
    """
    Example preprocessing step: normalize and convert to the correct format.
    This step can be adjusted according to your specific grid data.
    """
    # Normalize the grid to [0, 255] and convert to 8-bit
    grid_normalized = cv2.normalize(grid, None, 0, 255, cv2.NORM_MINMAX)
    return grid_normalized.astype(np.uint8)


def extract_features(processed_grid, density_threshold):
    """
    Extract points of high density from a preprocessed occupancy grid.

    :param processed_grid: The preprocessed occupancy grid.
    :param density_threshold: Threshold to consider a point as high density.
    :return: 2D array of points (coordinates) to track.
    """
    # Find the points where the occupancy value exceeds the threshold
    high_density_points = np.argwhere(processed_grid > density_threshold)

    # Convert to required format for optical flow (Nx1x2)
    feature_points = high_density_points.reshape(-1, 1, 2).astype(np.float32)

    return feature_points


def optical_flow_predictions(grid_a, grid_b):
    """
    Calculate optical flow between two grids using Lucas-Kanade method.
    Returns a list of feature points and their velocities
    """
    # Preprocess the grids (if necessary)
    grid_a_processed = preprocess_grid(grid_a)
    grid_b_processed = preprocess_grid(grid_b)

    # # Parameters for goodFeaturesToTrack
    # feature_params = dict(maxCorners=100, qualityLevel=0.3, minDistance=2)

    # # Detect good features to track in grid_a
    # p0 = cv2.goodFeaturesToTrack(grid_a_processed, mask=None, **feature_params)

    featurePoints = extract_features(grid_a, density_threshold=1.0)

    # Calculate optical flow
    nextFeaturePoints, st, err = cv2.calcOpticalFlowPyrLK(
        prevImg=grid_a_processed,
        nextImg=grid_b_processed,
        prevPts=featurePoints,
        nextPts=None,
        **lk_params
    )

    if nextFeaturePoints == None:
        print("AAAAHHHH!!!!")

    # Filter out points for which the flow was not found
    good_old = featurePoints[st == 1]
    good_new = nextFeaturePoints[st == 1]

    # Return the new and old positions of the tracked points
    return good_old, good_new - good_old


def next_grid_prediction(grid_a, grid_b):
    """
    Predict the next grid state using optical flow predictions.
    """
    flow = optical_flow_predictions(grid_a, grid_b)

    # Create an empty grid for the predicted state
    grid_c = np.zeros_like(grid_b)

    for i, (new, old) in enumerate(zip(flow, good_old)):
        a, b = new.ravel()
        c, d = old.ravel()

        # Check if the new position is within grid bounds
        if 0 <= a < GRID_WIDTH and 0 <= b < GRID_HEIGHT:
            grid_c[int(b), int(a)] = grid_b[int(d), int(c)]

    return grid_c


def mean_filter(grid, kernel_size=3):
    kernel = np.ones((kernel_size, kernel_size)) / (kernel_size**2)
    smoothed_grid = np.copy(grid).astype(float)

    # Apply mean filter
    for i in range(grid.shape[0] - kernel_size + 1):
        for j in range(grid.shape[1] - kernel_size + 1):
            smoothed_grid[i + kernel_size // 2, j + kernel_size // 2] = np.sum(
                grid[i : i + kernel_size, j : j + kernel_size] * kernel
            )

    return smoothed_grid


# Testing
if __name__ == "__main__":
    grid_a = np.array(
        [
            [0, 0, 0, 1, 1, 1, 0, 0],
            [0, 0, 0, 1, 1, 1, 0, 0],
            [0, 0, 0, 1, 1, 1, 0, 0],
            [1, 1, 0, 0, 0, 0, 1, 1],
            [1, 1, 0, 0, 0, 0, 1, 1],
            [1, 1, 0, 0, 0, 0, 1, 1],
        ]
    )

    grid_b = np.array(
        [
            [0, 0, 1, 1, 1, 0, 0, 0],
            [0, 0, 1, 1, 1, 0, 0, 0],
            [1, 1, 1, 1, 1, 0, 0, 0],
            [1, 1, 0, 0, 0, 0, 1, 1],
            [1, 1, 0, 0, 0, 0, 1, 1],
            [0, 0, 0, 0, 0, 0, 1, 1],
        ]
    )

    print()
    print(optical_flow_predictions(grid_a, grid_b))
