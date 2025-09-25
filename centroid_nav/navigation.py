from math import atan2, pi
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

class Navigation(Node):
    def __init__(self):
        super().__init__('navigation')

        # Publishers and subscribers
        self.publisher_vel = self.create_publisher(Twist, 'cmd_vel', 10)
        self.twist = Twist()
        self.timer = self.create_timer(0.05, self.control)

        self.subscriber_point = self.create_subscription(Point, 'point', self.callback_point, 10)
        self.subscriber_odom = self.create_subscription(Odometry, 'odom', self.callback_odom, 10)
        self.subscriber_scan = self.create_subscription(LaserScan, "scan", self.callback_scan, 10)

        # Odom data
        self.position = None
        self.orientation = None
        self.curr_direction = 0.0

        # Objective data
        self.distance = 0.0
        self.direction = 0.0
        self.objective = None

        # Lidar data
        self.ranges = []
        self.range_min = 0.0
        self.range_max = 0.0
        self.angle_min = 0.0
        self.angle_max = 0.0
        self.angle_increment = 0.0
        self.cx = 0.0
        #self.cy = 0.0
        self.eW_prev = 0.0
        self.eV_prev = 0.0

    def callback_scan(self, data):
        # Store raw data properties
        self.ranges = list(data.ranges[:])
        self.range_min = data.range_min
        self.range_max = data.range_max
        self.angle_min = data.angle_min
        self.angle_max = data.angle_max
        self.angle_increment = data.angle_increment

        # Create a full array of angles corresponding to the raw ranges
        full_angles = np.arange(self.angle_min, self.angle_max, self.angle_increment)

        # Define the desired angle limits for the front view
        front_angle_start = pi
        front_angle_end = 0

        # Calculate the start and end indices for the front view slice
        # We check if angle_increment is positive to avoid division by zero
        if self.angle_increment > 0:
            start_index = int((front_angle_start - self.angle_min) / self.angle_increment)
            end_index = int((front_angle_end - self.angle_min) / self.angle_increment)

            # Ensure the indices are within the bounds of the list
            num_ranges = len(self.ranges)
            start_index = max(0, start_index)
            end_index = min(num_ranges, end_index)

            # Slice the ranges and angles to get only the frontal data
            front_ranges = self.ranges[start_index:end_index]
            front_angles = full_angles[start_index:end_index]
        else:
            # Fallback to using all data if angle_increment is not valid
            front_ranges = self.ranges
            front_angles = full_angles

        # Clamp range values to the maximum measurable distance
        front_ranges = [min(r, self.range_max) for r in front_ranges]

        # Calculate centroid with the front-facing data
        sda = sum(element * self.angle_increment for element in front_ranges)
        sxda = sum(a * r * self.angle_increment for a, r in zip(front_angles, front_ranges))

        if sda > 0:
            self.cx = sxda / sda
            #self.cy = ((sda**2)/2)/sda
        else:
            self.cx = 0.0
            #self.cy = 0.0

        #Aquí me quedé

    def callback_point(self, data):
        self.objective = data

    def callback_odom(self, data):
        self.position = data.pose.pose.position
        self.orientation = data.pose.pose.orientation

        if self.objective is not None and self.position is not None and self.orientation is not None:
            delta_x = self.objective.x - self.position.x
            delta_y = self.objective.y - self.position.y
            self.distance = (delta_x**2 + delta_y**2)**0.5
            self.direction = atan2(delta_y, delta_x)
            self.curr_direction = 2*atan2(self.orientation.z, self.orientation.w)

        # self.get_logger().info(f'Posición actual: x={position.x}, y={position.y}, z={position.z}')
        # self.get_logger().info(f'Orientación actual: x={orientation.x}, y={orientation.y}, z={orientation.z}, w={orientation.w}')

    def control(self):
        dir_diff = self.direction - self.curr_direction
        if dir_diff > pi:
            dir_diff -= 2 * pi
        elif dir_diff < -pi:
            dir_diff += 2 * pi

        if dir_diff > 0.1 or dir_diff < -0.1:
            self.twist.angular.z = 0.3 * dir_diff
            self.twist.linear.x = 0.0
            # print(f"Dir diff: {dir_diff}")
        else: # Direccion correcta, avanzar!
            # print(f"Dir difffffdafsada: {dir_diff}")
            self.twist.angular.z = 0.0

            if self.distance > 0.05:
                self.twist.linear.x = 0.5 * self.distance
            else:
                self.twist.linear.x = 0.0

        self.publisher_vel.publish(self.twist)

def main(args=None):
    rclpy.init(args=args)
    node = Navigation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()