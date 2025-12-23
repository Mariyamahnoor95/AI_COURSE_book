---
id: ch16-sensor-fusion
title: Multi-Sensor Fusion
sidebar_label: Multi-Sensor Fusion
sidebar_position: 17
---

# Multi-Sensor Fusion

## Learning Objectives

By the end of this chapter, you will be able to:

- Fuse data from multiple sensors (camera, LiDAR, IMU, GPS, wheel odometry)
- Implement Kalman filters (EKF, UKF) for state estimation
- Use the robot_localization package for sensor fusion in ROS 2
- Build robust localization pipelines with fault detection
- Understand uncertainty propagation and covariance tuning

## Introduction

No single sensor provides complete, reliable information for robot localization. Cameras suffer from motion blur and lighting changes; LiDAR can miss transparent surfaces and struggles in fog; wheel odometry drifts over time due to wheel slippage; GPS fails indoors and near buildings. **Multi-sensor fusion** combines measurements from complementary sensors to produce more accurate, robust state estimates than any individual sensor.

A typical mobile robot sensor suite includes:

- **Wheel odometry**: High-frequency (50-100 Hz), low-drift over short distances, fails on slippery surfaces
- **IMU (Inertial Measurement Unit)**: High-frequency (100-500 Hz), measures acceleration and angular velocity, drifts over time
- **LiDAR**: Medium-frequency (10-20 Hz), accurate range measurements, sensitive to dynamic obstacles
- **Camera (visual odometry)**: Medium-frequency (10-30 Hz), rich feature information, affected by lighting
- **GPS**: Low-frequency (1-10 Hz), absolute position outdoors, 2-5m accuracy (RTK GPS: 2-5cm)

The challenges of sensor fusion include:

1. **Different update rates**: IMU @ 200 Hz, camera @ 30 Hz, GPS @ 5 Hz
2. **Different coordinate frames**: Each sensor has its own reference frame
3. **Measurement noise**: Each sensor has different noise characteristics
4. **Outlier detection**: Sensors can produce incorrect measurements (GPS multipath, LiDAR reflections)

In this chapter, we'll explore Kalman filtering—the foundation of sensor fusion—and implement production-ready multi-sensor localization pipelines using ROS 2.

## Kalman Filter Fundamentals

The **Kalman filter** is an optimal state estimator that recursively combines **predictions** (from a motion model) with **measurements** (from sensors) to estimate the true state of a system.

### The Kalman Filter Algorithm

At each time step, the filter performs two operations:

**1. Prediction Step** (using motion model):

```
x̂_{k|k-1} = F_k * x̂_{k-1|k-1} + B_k * u_k
P_{k|k-1} = F_k * P_{k-1|k-1} * F_k^T + Q_k
```

where:
- **x̂**: state estimate (position, velocity, orientation)
- **F**: state transition matrix (how state evolves)
- **u**: control input (wheel velocities, motor commands)
- **P**: estimation error covariance (uncertainty)
- **Q**: process noise covariance (model uncertainty)

**2. Update Step** (using sensor measurement):

```
K_k = P_{k|k-1} * H_k^T * (H_k * P_{k|k-1} * H_k^T + R_k)^{-1}
x̂_{k|k} = x̂_{k|k-1} + K_k * (z_k - H_k * x̂_{k|k-1})
P_{k|k} = (I - K_k * H_k) * P_{k|k-1}
```

where:
- **z**: sensor measurement (GPS position, LiDAR range)
- **H**: measurement matrix (maps state to measurement space)
- **R**: measurement noise covariance (sensor uncertainty)
- **K**: Kalman gain (optimal weighting of prediction vs measurement)

The **Kalman gain** balances trust between the prediction and measurement:
- If **P** (prediction uncertainty) is high → **K** is large → trust measurement more
- If **R** (measurement uncertainty) is high → **K** is small → trust prediction more

### Extended Kalman Filter (EKF)

For **nonlinear** systems (most robots), we use the **Extended Kalman Filter**, which linearizes the motion and measurement models:

```python
# Nonlinear motion model: x_{k+1} = f(x_k, u_k)
# EKF linearizes: F_k = ∂f/∂x |_{x̂_k}

def ekf_predict(x, P, u, dt, Q):
    """
    EKF prediction step for differential drive robot

    State: x = [x, y, theta, v, omega]^T
    Control: u = [v_cmd, omega_cmd]^T
    """
    # Nonlinear motion model
    x_pred = x.copy()
    x_pred[0] += x[3] * np.cos(x[2]) * dt  # x += v*cos(theta)*dt
    x_pred[1] += x[3] * np.sin(x[2]) * dt  # y += v*sin(theta)*dt
    x_pred[2] += x[4] * dt  # theta += omega*dt
    x_pred[3] = u[0]  # v = v_cmd
    x_pred[4] = u[1]  # omega = omega_cmd

    # Jacobian of motion model
    F = np.eye(5)
    F[0, 2] = -x[3] * np.sin(x[2]) * dt  # ∂x/∂theta
    F[0, 3] = np.cos(x[2]) * dt          # ∂x/∂v
    F[1, 2] = x[3] * np.cos(x[2]) * dt   # ∂y/∂theta
    F[1, 3] = np.sin(x[2]) * dt          # ∂y/∂v
    F[2, 4] = dt                         # ∂theta/∂omega

    # Covariance prediction
    P_pred = F @ P @ F.T + Q

    return x_pred, P_pred
```

### Unscented Kalman Filter (UKF)

For highly nonlinear systems, the **Unscented Kalman Filter** uses **sigma points** to approximate the state distribution, avoiding linearization errors:

```python
def ukf_predict(x, P, f, Q, alpha=0.001, beta=2, kappa=0):
    """
    UKF prediction using unscented transform

    Args:
        x: state mean (n,)
        P: state covariance (n, n)
        f: nonlinear state transition function
        Q: process noise covariance (n, n)
    """
    n = len(x)
    lambda_ = alpha**2 * (n + kappa) - n

    # Generate sigma points
    sigma_points = np.zeros((2*n + 1, n))
    sigma_points[0] = x

    sqrt_P = np.linalg.cholesky((n + lambda_) * P)
    for i in range(n):
        sigma_points[i+1] = x + sqrt_P[i]
        sigma_points[n+i+1] = x - sqrt_P[i]

    # Propagate sigma points through nonlinear function
    sigma_points_prop = np.array([f(sp) for sp in sigma_points])

    # Compute mean and covariance
    weights_m = np.full(2*n+1, 1/(2*(n+lambda_)))
    weights_m[0] = lambda_ / (n + lambda_)
    weights_c = weights_m.copy()
    weights_c[0] += (1 - alpha**2 + beta)

    x_pred = sigma_points_prop.T @ weights_m
    P_pred = np.zeros((n, n))
    for i in range(2*n+1):
        diff = sigma_points_prop[i] - x_pred
        P_pred += weights_c[i] * np.outer(diff, diff)

    P_pred += Q

    return x_pred, P_pred
```

**When to use EKF vs UKF**:
- **EKF**: Mildly nonlinear systems, computational efficiency critical (mobile robots, drones)
- **UKF**: Highly nonlinear systems, better accuracy needed (manipulators, underwater vehicles)

## Robot Localization Package in ROS 2

The **robot_localization** package provides production-ready implementations of EKF and UKF for fusing odometry, IMU, and GPS data in ROS 2.

### ekf_localization_node Configuration

Example configuration for fusing wheel odometry + IMU:

```yaml
# File: config/ekf.yaml

ekf_localization_node:
  ros__parameters:
    frequency: 30  # Hz (state estimation rate)
    sensor_timeout: 0.1  # seconds
    two_d_mode: true  # Ignore z, roll, pitch for 2D robots

    # Frame IDs
    map_frame: 'map'
    odom_frame: 'odom'
    base_link_frame: 'base_link'
    world_frame: 'odom'  # Can be 'odom' or 'map'

    # Odometry source 0 (wheel odometry)
    odom0: '/wheel_odom'
    odom0_config: [
      false, false, false,  # x, y, z (position)
      false, false, false,  # roll, pitch, yaw (orientation)
      true,  true,  false,  # vx, vy, vz (linear velocity)
      false, false, true,   # vroll, vpitch, vyaw (angular velocity)
      false, false, false   # ax, ay, az (linear acceleration)
    ]
    odom0_queue_size: 10
    odom0_differential: false  # Use absolute measurements
    odom0_relative: false

    # IMU source 0
    imu0: '/imu/data'
    imu0_config: [
      false, false, false,  # x, y, z
      true,  true,  true,   # roll, pitch, yaw (orientation)
      false, false, false,  # vx, vy, vz
      true,  true,  true,   # vroll, vpitch, vyaw (angular velocity)
      true,  true,  false   # ax, ay, az (linear acceleration)
    ]
    imu0_differential: false
    imu0_remove_gravitational_acceleration: true

    # Process noise covariance (Q matrix)
    process_noise_covariance: [
      0.05, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
      0.0,  0.05, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
      0.0,  0.0,  0.06, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
      # ... (15x15 matrix)
    ]

    # Initial estimate covariance (P_0 matrix)
    initial_estimate_covariance: [
      1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      # ... (15x15 matrix, larger values = more uncertainty)
    ]
```

### Tuning Covariance Parameters

**Process noise covariance (Q)**: Represents uncertainty in the motion model

- **Large Q**: Trust measurements more (filter tracks sensor noise)
- **Small Q**: Trust model more (filter is smooth but may lag)
- **Tuning**: Start with diagonal values ~0.05, increase for axes with high model error

**Measurement covariance (R)**: Defined in sensor messages (`Imu.angular_velocity_covariance`, `Odometry.pose.covariance`)

- Set based on sensor specs (e.g., IMU gyro noise density: 0.001 rad/s/√Hz)
- If unknown, start with conservative values (e.g., 0.1 for orientation, 0.01 for velocity)

### Launch File

```python
# File: launch/localization.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_localization',
            output='screen',
            parameters=[
                {'use_sim_time': False},
                'config/ekf.yaml'
            ],
            remappings=[
                ('/odometry/filtered', '/odom')
            ]
        )
    ])
```

## Visual-Inertial Odometry (VIO)

Fusing camera visual features with IMU accelerometer/gyroscope measurements provides **visual-inertial odometry** (VIO), which is more robust than pure visual odometry.

### ORB-SLAM3 with IMU

**ORB-SLAM3** is a state-of-the-art SLAM system supporting monocular, stereo, RGB-D, and monocular-inertial modes:

```yaml
# File: config/orb_slam3_imu.yaml

ORB_SLAM3:
  # Camera calibration
  Camera.fx: 615.0
  Camera.fy: 615.0
  Camera.cx: 320.0
  Camera.cy: 240.0

  # IMU parameters
  IMU.NoiseGyro: 0.001  # rad/s/√Hz (gyroscope noise)
  IMU.NoiseAcc: 0.01    # m/s²/√Hz (accelerometer noise)
  IMU.GyroWalk: 0.0001  # rad/s²/√Hz (gyro random walk)
  IMU.AccWalk: 0.001    # m/s³/√Hz (accel random walk)
  IMU.Frequency: 200    # Hz

  # Visualization
  Viewer.KeyFrameSize: 0.05
  Viewer.CameraSize: 0.08
  Viewer.PointSize: 2
```

### Isaac ROS Visual-Inertial SLAM

For GPU-accelerated VIO, use **Isaac ROS visual_slam** with IMU fusion:

```python
#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='isaac_ros_visual_slam',
            executable='isaac_ros_visual_slam',
            name='visual_slam',
            parameters=[{
                'enable_imu': True,
                'enable_slam_visualization': True,
                'num_cameras': 1,  # Monocular
                'rectified_images': True,
                'enable_localization_n_mapping': True
            }],
            remappings=[
                ('visual_slam/image_0', '/camera/image_raw'),
                ('visual_slam/camera_info_0', '/camera/camera_info'),
                ('visual_slam/imu', '/imu/data')
            ]
        )
    ])
```

VIO advantages:
- **No drift in orientation** (IMU provides absolute gravity reference)
- **Robust to rapid motion** (IMU fills gaps when visual tracking fails)
- **Faster initialization** (IMU provides scale estimate for monocular cameras)

## Multi-Sensor Localization Pipeline

For production systems, fuse all available sensors: wheel odometry, IMU, LiDAR SLAM, and GPS.

### Complete Sensor Fusion Architecture

```yaml
# File: config/ekf_complete.yaml

ekf_localization_node:
  ros__parameters:
    frequency: 50

    # Wheel odometry (high frequency, drifts)
    odom0: '/wheel_odom'
    odom0_config: [false, false, false,
                   false, false, false,
                   true, true, false,  # vx, vy
                   false, false, true,  # vyaw
                   false, false, false]

    # IMU (orientation and angular velocity)
    imu0: '/imu/data'
    imu0_config: [false, false, false,
                  true, true, true,  # roll, pitch, yaw
                  false, false, false,
                  true, true, true,  # angular velocities
                  false, false, false]
    imu0_remove_gravitational_acceleration: true

    # LiDAR odometry (accurate but medium frequency)
    odom1: '/lidar_odom'
    odom1_config: [true, true, false,  # x, y (absolute position)
                   false, false, true,  # yaw
                   false, false, false,
                   false, false, false,
                   false, false, false]

    # GPS (outdoor only, low frequency)
    odom2: '/gps/odom'
    odom2_config: [true, true, false,  # x, y (absolute position from GPS)
                   false, false, false,
                   false, false, false,
                   false, false, false,
                   false, false, false]
```

### Weighted Fusion Based on Uncertainty

The robot_localization package automatically weights sensors based on their covariance. To manually adjust trust:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import numpy as np

class AdaptiveCovarianceNode(Node):
    """Adjust sensor covariance based on operating conditions"""

    def __init__(self):
        super().__init__('adaptive_covariance')

        self.wheel_odom_sub = self.create_subscription(
            Odometry, '/wheel_odom_raw', self.odom_callback, 10
        )
        self.wheel_odom_pub = self.create_publisher(
            Odometry, '/wheel_odom', 10
        )

    def odom_callback(self, msg):
        """Increase wheel odometry uncertainty on slippery surfaces"""

        # Detect wheel slip (compare commanded vs actual velocity)
        velocity_error = abs(self.cmd_vel - msg.twist.twist.linear.x)

        if velocity_error > 0.2:  # Slipping
            # Increase uncertainty (less trust in wheel odometry)
            inflation_factor = 10.0
        else:
            inflation_factor = 1.0

        # Modify covariance
        msg.pose.covariance[0] *= inflation_factor  # x uncertainty
        msg.pose.covariance[7] *= inflation_factor  # y uncertainty

        self.wheel_odom_pub.publish(msg)

def main():
    rclpy.init()
    node = AdaptiveCovarianceNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Detecting Sensor Failures

Implement outlier detection and automatic sensor switching:

```python
class SensorHealthMonitor(Node):
    """Monitor sensor health and disable faulty sensors"""

    def __init__(self):
        super().__init__('sensor_health_monitor')

        self.gps_timeout = 2.0  # seconds
        self.last_gps_time = self.get_clock().now()

        self.gps_sub = self.create_subscription(
            Odometry, '/gps/odom', self.gps_callback, 10
        )

        self.create_timer(0.5, self.health_check)

    def gps_callback(self, msg):
        self.last_gps_time = self.get_clock().now()

        # Check for GPS outliers (sudden jumps > 10m)
        if hasattr(self, 'last_gps_pos'):
            dx = msg.pose.pose.position.x - self.last_gps_pos[0]
            dy = msg.pose.pose.position.y - self.last_gps_pos[1]
            distance = np.sqrt(dx**2 + dy**2)

            if distance > 10.0:
                self.get_logger().warn(f'GPS outlier detected: {distance:.1f}m jump')
                return  # Reject measurement

        self.last_gps_pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def health_check(self):
        """Check for sensor timeouts"""
        time_since_gps = (self.get_clock().now() - self.last_gps_time).nanoseconds / 1e9

        if time_since_gps > self.gps_timeout:
            self.get_logger().error('GPS timeout - indoors or signal lost')
            # Disable GPS in robot_localization config dynamically
```

## Best Practices

1. **Calibrate sensor extrinsics**: Measure precise transforms between sensor frames
2. **Tune covariances systematically**: Start conservative, then reduce based on real data
3. **Log raw sensor data**: Replay for offline tuning and debugging
4. **Validate fusion accuracy**: Compare fused estimate to ground truth (motion capture, RTK GPS)
5. **Implement watchdogs**: Detect and handle sensor failures gracefully

## Summary

Key takeaways from this chapter:

- **Multi-sensor fusion** combines complementary sensors (wheel odometry, IMU, LiDAR, camera, GPS) for robust localization
- **Kalman filters** (EKF, UKF) optimally fuse predictions and measurements by balancing uncertainty
- **robot_localization** provides production-ready sensor fusion for ROS 2 with flexible configuration
- **Visual-inertial odometry** (VIO) fuses camera and IMU for drift-free orientation and scale
- **Weighted fusion** based on covariance allows automatic trust adjustment, while outlier detection ensures robustness

These techniques form the foundation of reliable state estimation for Physical AI systems operating in complex, uncertain environments.

## Review Questions

1. Why is multi-sensor fusion necessary for mobile robots? Provide three specific failure modes of individual sensors.
2. Explain the prediction and update steps of the Kalman filter. What is the role of the Kalman gain?
3. What is the key difference between EKF and UKF? When would you choose UKF over EKF?
4. How does the robot_localization package weight different sensors? How can you adjust trust in a specific sensor?
5. What are the advantages of visual-inertial odometry (VIO) over pure visual odometry?
6. Describe how you would detect and handle a GPS outlier measurement in real-time.
7. How would you tune the process noise covariance (Q) and measurement noise covariance (R) for a new robot platform?

## Hands-On Exercises

### Exercise 1: EKF Sensor Fusion

**Objective**: Configure robot_localization to fuse wheel odometry and IMU.

**Steps**:
1. Set up a differential drive robot in Gazebo with wheel encoders and IMU
2. Create ekf.yaml configuration file
3. Launch ekf_localization_node
4. Drive the robot in a square path
5. Compare fused odometry `/odometry/filtered` to ground truth

**Expected Outcome**: Fused estimate has &lt;5% position error after 10m traveled.

### Exercise 2: GPS + IMU Fusion

**Objective**: Integrate GPS for outdoor localization.

**Steps**:
1. Add GPS sensor to robot URDF
2. Configure robot_localization with GPS as odom2
3. Drive robot outdoors and record GPS + IMU + wheel odometry
4. Compare fused estimate to RTK GPS ground truth
5. Test GPS outlier rejection by injecting fake measurements

**Expected Outcome**: Fused estimate maintains &lt;2m error even with GPS outages.

### Exercise 3: Visual-Inertial SLAM

**Objective**: Deploy Isaac ROS visual_slam with IMU fusion.

**Steps**:
1. Mount a camera and IMU on robot with calibrated extrinsics
2. Launch Isaac ROS visual_slam with IMU enabled
3. Drive robot through indoor environment
4. Measure drift after loop closure
5. Compare to wheel odometry drift

**Expected Outcome**: VIO drift &lt;1% of distance traveled, significantly better than wheel odometry.

## Further Reading

- **Probabilistic Robotics** (Thrun, Burgard, Fox): Chapters on Kalman filtering and sensor fusion
- **robot_localization Documentation**: [http://docs.ros.org/en/humble/p/robot_localization/](http://docs.ros.org/en/humble/p/robot_localization/)
- **ORB-SLAM3 Paper**: "ORB-SLAM3: An Accurate Open-Source Library for Visual, Visual-Inertial and Multi-Map SLAM" (2021)
- **Kalman Filter Tutorial**: [https://www.kalmanfilter.net/](https://www.kalmanfilter.net/)
- **Isaac ROS Visual SLAM**: [https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_visual_slam/](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_visual_slam/)

---

**Previous**: [Chapter 15 - Reinforcement Learning and Sim-to-Real](ch15-rl-sim2real.md)
**Next**: [Chapter 17 - Humanoid Robot URDF and Kinematics](../../module-04-vla/week-11/ch17-humanoid-urdf.md)
