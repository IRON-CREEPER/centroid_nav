from math import atan2, pi

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry

class FollowPoint(Node):
    def __init__(self):
        super().__init__('follow_point')
        # self.get_logger().info('¡Hola, soy el nuevo nodo (follow_point) y estoy vivo!')

        self.publisher_vel = self.create_publisher(Twist, 'cmd_vel', 10)
        self.twist = Twist()
        self.timer = self.create_timer(0.05, self.control)

        self.subscriber_point = self.create_subscription(Point, 'point', self.callback_point, 10)
        self.subscriber_odom = self.create_subscription(Odometry, 'odom', self.callback_odom, 10)

        self.position = None
        self.orientation = None
        self.curr_direction = 0.0

        self.distance = 0.0
        self.direction = 0.0
        self.objective = None

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
    node = FollowPoint()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()