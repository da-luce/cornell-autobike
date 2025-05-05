# Summary
This ROS 2 node implements a real-time lane detection system using image input from a ZED stereo camera. The module captures and processes camera frames to identify road lanes, which is important for the autonomous navigation of the Cornell Autonomous Bicycle. Using CV methods like edge detection and Hough Line Transformation, this module draws detected lanes on the original image and publishes the result for visual & navigation purposes.

# Dependencies
- ROS 2
- Python 3.8+
- OpenCV (cv2)
- cv_bridge
- sensor_msgs
- rospy
- ZED camera

# Usage
To run the lane detection module:
`ros2 run lane_detection lane_detector.py`

This will subscribe to the camera feed and publish the processed image with detected lanes to the topic src/sim/sim/camera.

# Features
Steps Used for Lane Detection:
- Image Acquisition: Subscribe to sensor_msgs/Image from the ZED camera.
- Grayscale Conversion: Convert image to grayscale for easier processing.
- Noise Reduction: Apply Gaussian blur to reduce image noise.
- Edge Detection: Use Canny edge detection to find edges in the image.
- Line Detection: Apply Hough Line Transform to identify lane-like lines.
- Overlay: Draw detected lines on the original image.
- Publish: Convert back to ROS image message and publish to a visualization topic.


# Nodes
Subscribed Topics:
`/camera (sensor_msgs/Image)`
- Raw image feed from the ZED camera

Published Topics:
`/src/sim/sim/camera (sensor_msgs/Image)`
- Processed image with lane overlays.

# Configuration
To adjust input/output topics or modify internal parameters, open lane_detector.py and change these:
## Input image topic
`self.image_sub = rospy.Subscriber("camera", Image, self.image_callback)`

## Output processed image topic
`self.lane_pub = rospy.Publisher("src/sim/sim/camera", Image, queue_size=10)`
