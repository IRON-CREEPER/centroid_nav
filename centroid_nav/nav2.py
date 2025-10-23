import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

from math import atan2, pi
import numpy as np


class Nav2(Node):
    def __init__(self):
        super().__init__('nav2')

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
        self.range_max = 1.5
        self.angle_min = 0.0
        self.angle_max = 0.0
        self.angle_increment = 0.0
        self.cx = 0.0
        self.cy = 0.0
        self.eW_prev = 0.0
        self.eV_prev = 0.0

        # El __init__ está ahora como el original, sin la máquina de estados
        # ni los parámetros de ajuste del controlador.

    def callback_scan(self, data):
        # Store raw data properties
        self.ranges = list(data.ranges[:])
        self.range_min = data.range_min
        # self.range_max = data.range_max
        self.angle_min = data.angle_min
        self.angle_max = data.angle_max
        self.angle_increment = data.angle_increment

        # Create a full array of angles corresponding to the raw ranges
        full_angles = np.arange(self.angle_min, self.angle_max, self.angle_increment)

        # Define the desired angle limits for the front view 120 degrees
        front_angle_start = -pi / 2 - pi / 3  # -pi
        front_angle_end = -pi / 2 + pi / 3  # 0

        # Calculate the start and end indices for the front view slice
        if self.angle_increment > 0:
            start_index = int((front_angle_start - self.angle_min) / self.angle_increment)
            end_index = int((front_angle_end - self.angle_min) / self.angle_increment)

            num_ranges = len(self.ranges)
            start_index = max(0, start_index)
            end_index = min(num_ranges, end_index)

            front_ranges = self.ranges[start_index:end_index]
            front_angles = full_angles[start_index:end_index]
        else:
            front_ranges = self.ranges
            front_angles = full_angles

        # Clamp range values to the maximum measurable distance
        front_ranges = [min(r, self.range_max) for r in front_ranges]

        # Corrección para asegurar que los arrays tengan la misma longitud
        if len(front_angles) > len(front_ranges):
            front_angles = front_angles[:len(front_ranges)]
        elif len(front_ranges) > len(front_angles):
            front_ranges = front_ranges[:len(front_angles)]

        # Calculate centroid with the front-facing data
        sda = sum(element * self.angle_increment for element in front_ranges)
        sxda = sum(a * r * self.angle_increment for a, r in zip(front_angles, front_ranges))

        if sda > 0:
            self.cx = sxda / sda
            self.cy = ((sda ** 2) / 2) / sda
        else:
            self.cx = 0.0
            self.cy = 0.0

    def callback_point(self, data):
        self.objective = data

    def callback_odom(self, data):
        self.position = data.pose.pose.position
        self.orientation = data.pose.pose.orientation

        if self.objective is not None and self.position is not None and self.orientation is not None:
            delta_x = self.objective.x - self.position.x
            delta_y = self.objective.y - self.position.y
            self.distance = (delta_x ** 2 + delta_y ** 2) ** 0.5
            self.direction = atan2(delta_y, delta_x)
            self.curr_direction = 2 * atan2(self.orientation.z, self.orientation.w)

    def control(self):

        Kp_goal_angular = 0.4  # Ganancia para girar hacia el objetivo
        Kp_goal_linear = 0.5  # Ganancia para avanzar al objetivo
        Kp_avoid_angular = -3.5  # Ganancia P para giro de evasión
        Kd_avoid_angular = -0.5  # Ganancia D para giro de evasión
        Kp_avoid_linear = -1.5  # Ganancia P para velocidad lineal de evasión
        Kd_avoid_linear = -0.1  # Ganancia D para velocidad lineal de evasión
        SAFE_DISTANCE = 1.5  # Distancia para empezar a reaccionar
        DANGER_DISTANCE = 1.0  # Distancia de reacción máxima
        MAX_LINEAR_SPEED = 0.22  # Velocidad lineal máxima (m/s)
        MIN_LINEAR_SPEED = 0.0
        MAX_ANGULAR_SPEED = 1.0  # Velocidad angular máxima (rad/s)

        # --- 1. ESTADO: No listo ---
        if self.position is None or self.orientation is None or self.objective is None:
            self.publisher_vel.publish(Twist())
            return

        # --- 2. ESTADO: Objetivo alcanzado ---
        if self.distance <= 0.1:
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
            print("Llegué al objetivo")
            self.publisher_vel.publish(self.twist)
            return

        # --- 3. ESTADO: Navegando (Controlador Unificado) ---

        # --- A. Calcular errores para AMBOS comportamientos ---

        # Error 1: Ir al Objetivo (Goal-Seeking)
        dir_diff = self.direction - self.curr_direction
        if dir_diff > pi:
            dir_diff -= 2 * pi
        elif dir_diff < -pi:
            dir_diff += 2 * pi

        # Error 2: Evitar Obstáculos (Avoidance)
        eV = 0.0 - self.cy  # Error lineal (distancia al centroide)
        eW = -pi / 2 - self.cx  # Error angular (ángulo del centroide)

        # --- B. Calcular comandos deseados para CADA comportamiento ---

        # Comando 1: Ir al Objetivo
        cmd_angular_goal = Kp_goal_angular * dir_diff
        cmd_linear_goal = min(Kp_goal_linear * self.distance, MAX_LINEAR_SPEED)

        # Comando 2: Evitar Obstáculos (Controlador PD)
        cmd_angular_avoid = (Kp_avoid_angular * eW) + (Kd_avoid_angular * (eW - self.eW_prev))
        cmd_linear_avoid = (Kp_avoid_linear * eV) + (Kd_avoid_linear * (eV - self.eV_prev))

        # --- C. Calcular pesos dinámicos basados en el peligro ---

        raw_danger = (SAFE_DISTANCE - self.cy) / (SAFE_DISTANCE - DANGER_DISTANCE)
        danger_factor = min(max(raw_danger, 0.0), 1.0)

        weight_avoid = danger_factor
        weight_goal = 1.0 - danger_factor

        # --- D. Fusionar los comandos usando los pesos ---

        final_angular = (weight_goal * cmd_angular_goal) + (weight_avoid * cmd_angular_avoid)
        final_linear = (weight_goal * cmd_linear_goal) + (weight_avoid * cmd_linear_avoid)

        # --- E. Aplicar límites globales y publicar ---

        self.twist.angular.z = max(-MAX_ANGULAR_SPEED, min(final_angular, MAX_ANGULAR_SPEED))
        self.twist.linear.x = max(MIN_LINEAR_SPEED, min(final_linear, MAX_LINEAR_SPEED))

        # --- F. Actualizar errores previos para el controlador Derivativo ---
        self.eW_prev = eW
        self.eV_prev = eV

        self.publisher_vel.publish(self.twist)

        print(
            f"Goal_W: {weight_goal:.2f} | Avoid_W: {weight_avoid:.2f} | Lin: {self.twist.linear.x:.2f} | Ang: {self.twist.angular.z:.2f}")


def main(args=None):
    rclpy.init(args=args)
    node = Nav2()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()