import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from std_msgs.msg import Float32

HALF_DISTANCE_BETWEEN_WHEELS = 0.045
WHEEL_RADIUS = 0.025

class MyRobotDriver:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot

        self.__left_motor = self.__robot.getDevice('left wheel motor')
        self.__right_motor = self.__robot.getDevice('right wheel motor')

        self.__left_motor.setPosition(float('inf'))
        self.__left_motor.setVelocity(0)

        self.__right_motor.setPosition(float('inf'))
        self.__right_motor.setVelocity(0)

        # gps initialization here
        gps = self.__robot.getGPS("gps")
        gps.enable(32)

        self.__target_twist = Twist()

        rclpy.init(args=None)
        self.__node = rclpy.create_node('my_robot_driver')

        self.__node.create_subscription(Twist, '/cmd_vel', self.__cmd_vel_callback, 1)
        self.__node.create_subscription(LaserScan, '/lidar', self.__lidar_callback, 1)
        self.__node.create_subscription(Float32, '/angle', self.__angle_callback, 1)

        # PointCloud2 Publisher
        self.__pointcloud_publisher = self.__node.create_publisher(PointCloud2, '/pointcloud', 10)

    def __angle_callback(self, msg):
        """ Callback function to handle received angle data """
        angle = msg.data  # Extract float value from message

        # Convert angle into an angular speed (you may need to define how)
        angular_speed = angle
        forward_speed = 2

        # Compute motor commands based on the kinematic model
        command_motor_left = (forward_speed - angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS
        command_motor_right = (forward_speed + angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS

        self.__left_motor.setVelocity(command_motor_left)
        self.__right_motor.setVelocity(command_motor_right)

    def __lidar_callback(self, scan):
        """ Process LIDAR data and publish PointCloud2 """
        ranges = np.array(scan.ranges)
        angles = np.linspace(scan.angle_min, scan.angle_max, len(ranges))
        gps_value = self.gps.getValues()
        print(gps_value)
        # Convert to Cartesian coordinates
        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)
        z = np.zeros_like(x)  # 2D LiDAR: all points are at Z = 0

        # Publish the PointCloud2 message
        self.__node.get_logger().info(f"Publishing point cloud!")
        pointcloud_msg = self.create_pointcloud2_msg(x, y, z, scan.header.frame_id)
        self.__pointcloud_publisher.publish(pointcloud_msg)

    def create_pointcloud2_msg(self, x, y, z, frame_id="laser_frame"):
        """ Create a PointCloud2 message from x, y, z arrays """
        cloud_msg = PointCloud2()
        cloud_msg.header.stamp = self.__node.get_clock().now().to_msg()
        cloud_msg.header.frame_id = frame_id

        cloud_msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1)
        ]

        cloud_msg.point_step = 12  # Each point has 3 floats (x, y, z) → 3 * 4 bytes = 12 bytes
        cloud_msg.row_step = cloud_msg.point_step * len(x)
        cloud_msg.is_dense = False  # Contains NaNs for invalid measurements
        cloud_msg.is_bigendian = False
        cloud_msg.height = 1
        cloud_msg.width = len(x)

        # Pack point data into binary format
        points = np.array([x, y, z], dtype=np.float32).T
        cloud_msg.data = points.tobytes()

        return cloud_msg

    def __cmd_vel_callback(self, twist):
        self.__target_twist = twist

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)

        self.__node.get_logger().info(f"Stepping!")

        forward_speed = self.__target_twist.linear.x
        angular_speed = self.__target_twist.angular.z

        command_motor_left = (forward_speed - angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS
        command_motor_right = (forward_speed + angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS

        self.__left_motor.setVelocity(command_motor_left)
        self.__right_motor.setVelocity(command_motor_right)
