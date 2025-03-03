import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, LaserScan, NavSatFix, PointCloud2, PointField
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


        self.__target_twist = Twist()

        rclpy.init(args=None)
        self.__node = rclpy.create_node('my_robot_driver')

        self.__node.create_subscription(Twist, '/cmd_vel', self.__cmd_vel_callback, 1)
        self.__node.create_subscription(LaserScan, '/lidar', self.__lidar_callback, 1)
        self.__node.create_subscription(Float32, '/angle', self.__angle_callback, 1)

        # PointCloud2 Publisher
        self.__pointcloud_publisher = self.__node.create_publisher(PointCloud2, '/pointcloud', 10)
        # GPS publisher
        self.__gps_publisher = self.__node.create_publisher(NavSatFix, '/gps', 10)
        # Gyroscope publisher
        self.__gyro_publisher = self.__node.create_publisher(Imu, '/gyro', 10)

        # Initialize GPS
        self.__gps = self.__robot.getDevice("gps")
        self.__gps.enable(32)

        #Initialize the Gyroscope
        self.__gyro = self.__robot.getDevice("gyro")
        self.__gyro.enable(32)

    def __gyro_callback(self):
        gyro_vals = self.__gyro.getValues()
        imu = Imu()

        imu.angular_velocity.x = gyro_vals[0]
        imu.angular_velocity.y = gyro_vals[1]
        imu.angular_velocity.z = gyro_vals[2]
        self.__gyro_publisher.publish(imu)
        self.__node.get_logger().info('Publishing Gyroscope Data: X: %.2f, Y: %.2f, Z: %.2f' %
                               (imu.angular_velocity.x,
                                imu.angular_velocity.y,
                                imu.angular_velocity.z))


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

        # Log the steering action
        self.__node.get_logger().info(f"Steering: Angle={angle:.2f}, Left Motor={command_motor_left:.2f}, Right Motor={command_motor_right:.2f}")

    def __gps_callback(self):
        """ Callback function to publish GPS data """
        gps_value = self.__gps.getValues()

        # Create a NavSatFix message and populate it with GPS data
        gps_msg = NavSatFix()
        gps_msg.latitude = gps_value[0]  # Latitude
        gps_msg.longitude = gps_value[1]  # Longitude
        gps_msg.altitude = gps_value[2]  # Altitude

        # Publish the GPS data
        self.__gps_publisher.publish(gps_msg)

        # Log GPS data
        self.__node.get_logger().info(f"GPS: Latitude={gps_value[0]:.6f}, Longitude={gps_value[1]:.6f}, Altitude={gps_value[2]:.2f}")

    def __lidar_callback(self, scan):
        """ Process LIDAR data and publish PointCloud2 """
        ranges = np.array(scan.ranges)
        angles = np.linspace(scan.angle_min, scan.angle_max, len(ranges))
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
        self.__node.get_logger().info("Publishing GPS data...")
        self.__gps_callback()

        self.__node.get_logger().info("Publishing Gyroscope data...")
        self.__gyro_callback()

        forward_speed = self.__target_twist.linear.x
        angular_speed = self.__target_twist.angular.z

        command_motor_left = (forward_speed - angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS
        command_motor_right = (forward_speed + angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS

        self.__left_motor.setVelocity(command_motor_left)
        self.__right_motor.setVelocity(command_motor_right)
