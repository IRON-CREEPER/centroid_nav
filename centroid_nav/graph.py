import rclpy
from rclpy.node import Node
import matplotlib.pyplot as plt
from sensor_msgs.msg import LaserScan
import numpy as np
import math


class Graph(Node):

    def __init__(self):
        super().__init__('graph')

        self.subscriber_scan = self.create_subscription(LaserScan, "scan", self.callback_scan, 10)

        self.ranges = []
        self.range_min = 0.0
        self.range_max = 0.0
        self.angle_min = 0.0
        self.angle_max = 0.0
        self.angle_increment = 0.0

        plt.ion()
        self.fig, self.ax = plt.subplots()

    def callback_scan(self, data):
        self.ranges = list(data.ranges[:])
        self.range_min = data.range_min
        self.range_max = data.range_max
        self.angle_min = data.angle_min
        self.angle_max = data.angle_max
        self.angle_increment = data.angle_increment

        full_angles = np.arange(self.angle_min, self.angle_max, self.angle_increment)

        front_angle_start = -math.pi
        front_angle_end = math.pi

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

        front_ranges = [min(r, self.range_max) for r in front_ranges]

        if len(front_angles) > len(front_ranges):
            front_angles = front_angles[:len(front_ranges)]

        self.ax.clear()
        self.ax.plot(front_angles, front_ranges, label='LiDAR Scan (Front)')
        self.ax.set_xlabel('Angle (radians)')
        self.ax.set_ylabel('Range (meters)')
        self.ax.set_title('Dynamic LiDAR Scan Data: Range vs. Angle')
        self.ax.grid(True)
        self.ax.legend()
        self.ax.set_ylim(0, 11)
        plt.pause(0.01)


def main(args=None):
    rclpy.init(args=args)
    node = Graph()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()