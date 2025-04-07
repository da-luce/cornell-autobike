import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class LaneDetector:
    def __init__(self):
        """ Initialize the ROS node and set up subscribers and publishers """
        rospy.init_node('listener', anonymous=True)
        self.bridge = CvBridge()


        # Subscribe to ZED camera left image
        self.image_sub = rospy.Subscriber("camera", Image, self.image_callback)

        # Publisher for processed lane detection image
        self.lane_pub = rospy.Publisher("src/sim/sim/camera", Image, queue_size=10)
        self.latest_image = None

    def image_callback(self, data):
        """ Process the incoming ZED camera image """
        try:
            # Convert ROS Image to OpenCV
            self.latest_image = self.bridge.imgmsg_to_cv2(data, "bgra8")
        except Exception as e:
            rospy.logerr(f"Error converting image: {e}")

    def process_frame(self, img):
        """ Apply lane detection on the image """
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)  # Convert to grayscale
        blur = cv2.GaussianBlur(gray, (5, 5), 0)  # Apply Gaussian blur
        edges = cv2.Canny(blur, 50, 150)  # Detect edges using Canny

        # Apply Hough Transform to detect lane lines
        lines = cv2.HoughLinesP(edges, 1, np.pi/180, 50, minLineLength=100, maxLineGap=50)

        # Draw detected lanes
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(img, (x1, y1), (x2, y2), (0, 255, 0), 3)

        return img

    def run(self):
        """ Continuously process frames from the ZED camera """
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.latest_image is not None:
                processed_image = self.process_frame(self.latest_image)

                # Convert back to ROS Image message and publish
                lane_img_msg = self.bridge.cv2_to_imgmsg(processed_image, "bgr8")
                self.lane_pub.publish(lane_img_msg)


                cv2.imshow("Lane Detection", processed_image)
                cv2.waitKey(1)

            rate.sleep()

if __name__ == "__main__":
    detector = LaneDetector()
    detector.run()
    cv2.destroyAllWindows()
