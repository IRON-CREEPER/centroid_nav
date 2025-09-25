import rclpy  # importa la librería de Ros2
import numpy as np
import math
from geometry_msgs.msg import Twist  # importa un paquete de dimensiones, ángulos. Twist es x y z
from sensor_msgs.msg import LaserScan
from rclpy.node import Node  # importa una entidad que procesa la info de Ros2


class Navigation(Node):
    def __init__(self):
        super().__init__('navigation')
        self.publisher_vel = self.create_publisher(Twist, 'cmd_vel', 10)
        self.twist = Twist()
        self.timer = self.create_timer(0.05, self.control)
        self.ranges = []
        self.range_min = 0.0
        self.range_max = 0.0
        self.angle_min = 0.0
        self.angle_max = 0.0
        self.angle_increment = 0.0
        self.cx = 0.0
        self.cy = 0.0
        self.eW_prev = 0.0
        self.eV_prev = 0.0

        self.subscriber_scan = self.create_subscription(LaserScan, "scan", self.callbackScan, 10)

    def callbackScan(self, data):
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
        front_angle_start = -math.pi
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
            self.cy = ((sda**2)/2)/sda
        else:
            self.cx = 0.0
            self.cy = 0.0

    def control(self):
        eV = 0 - self.cy
        self.twist.linear.x = (-0.3 * eV) + ((eV-self.eV_prev) * -10)
        self.eV_prev = eV

        eW = -math.pi/2 - self.cx # Error set
        self.twist.angular.z = (-20 * eW) + ((eW-self.eW_prev) * 5)
        self.eW_prev = eW

        self.publisher_vel.publish(self.twist)


def main(args=None):
    rclpy.init(args=args)
    listener = Navigation()
    rclpy.spin(listener)
    listener.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()