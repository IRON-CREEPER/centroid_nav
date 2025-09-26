# ROS 2 Hybrid Navigation

A ROS 2 package for mobile robot navigation. The main node provides a hybrid controller that combines go-to-point navigation with reactive obstacle avoidance using Lidar data.

## Available Nodes

This repository contains three distinct navigation nodes:

-   `navigation.py` **(Main Hybrid Node):** Intelligently switches between navigating to a goal and avoiding obstacles.
-   `follow_point.py` **(Simple Goal-Seeker):** Navigates to a target point without any obstacle avoidance.
-   `centroid_nav.py` **(Simple Avoider):** Purely reactive controller that wanders randomly while avoiding obstacles, without a specific goal.

## How the Hybrid Node (`navigation.py`) Works

The node operates as a simple state machine with two primary behaviors:

1.  **Goal Seeking:** When the path is clear, a proportional controller steers the robot towards coordinates published on the `/point` topic. The robot's speed is proportional to the remaining distance.

2.  **Obstacle Avoidance:** When an obstacle is detected, the node calculates its centroid from the `/scan` data. A PD controller then takes over to steer the robot, keeping the obstacle at a constant offset angle (effectively orbiting or passing it).

A **debouncing mechanism** is used for state transitions, requiring a condition to persist for several consecutive cycles before the robot changes its behavior. This ensures stability and prevents erratic movements from noisy sensor data.

## Dependencies

-   ROS 2 (Humble recommended)
-   Python 3
-   `geometry_msgs`, `nav_msgs`, `sensor_msgs`

## Usage

1.  Clone this repository into your ROS 2 workspace (`~/ros2_ws/src`).
2.  Build the package:
    ```bash
    cd ~/ros2_ws
    colcon build --packages-select centroid_nav
    ```
3.  Source the workspace:
    ```bash
    source ~/ros2_ws/install/setup.bash
    ```
4.  Run the main navigation node:
    ```bash
    ros2 run centroid_nav navigation
    ```
5.  Publish a goal point to start navigation:
    ```bash
    ros2 topic pub --once /point geometry_msgs/msg/Point "{x: 5.0, y: 2.0, z: 0.0}"
    ```

## ROS 2 API

#### Subscribed Topics

| Topic | Message Type | Description |
| :--- | :--- | :--- |
| `/odom` | `nav_msgs/msg/Odometry` | Provides the robot's current position and orientation. |
| `/scan` | `sensor_msgs/msg/LaserScan` | Provides Lidar data for obstacle detection. |
| `/point`| `geometry_msgs/msg/Point` | Receives the (x, y) coordinates for the navigation goal. |

#### Published Topics

| Topic | Message Type | Description |
| :--- | :--- | :--- |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Publishes velocity commands to move the robot. |