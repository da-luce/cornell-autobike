# Approach

1. Obtain raw occupancy grid
2. Extend occupancy grid with a border of 0 values (to improve edge predictions)

    From observation, the Lucas–Kanade method struggles predicting motion on
    borders.

3. _Depending on grid resolution, update rate, other factors, descale the grid
    before predicting velocities_

    From Wikipedia:

    "One main assumption for this method is that the motion is small (less than 1 pixel between two images for example). If the motion is large and violates this assumption, one technique is to reduce the resolution of images first and then apply the Lucas–Kanade method."

    <https://en.wikipedia.org/wiki/Lucas%E2%80%93Kanade_method>

4. Evaluate velocity
5. Normalize velocities (using some blurring of sort)
6. Binarize extended occupancy grid (based on some threshold)
7. Mask normalized velocities with binarized grid
8. Predict new pixel locations using velocities
9. Trim borders to obtain final occupancy grid prediction