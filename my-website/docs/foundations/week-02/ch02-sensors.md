---
id: ch02-sensors
title: Sensors in Physical AI Systems
sidebar_label: Sensors in Physical AI
sidebar_position: 1
---

# Sensors in Physical AI Systems

## Learning Objectives

By the end of this chapter, you will be able to:

- Understand different types of sensors used in robotics (vision, LiDAR, IMU, proprioceptive)
- Compare RGB cameras vs depth cameras and their use cases
- Explain how LiDAR and range sensors work
- Understand inertial measurement units (IMUs) for orientation tracking
- Describe proprioceptive sensors for internal state monitoring
- Apply basic sensor fusion strategies

## Introduction

Physical AI systems depend entirely on sensors to perceive and interact with the world. Unlike pure software AI that operates on digital data, embodied AI must bridge the physical-digital divide through sensors that convert real-world phenomena (light, distance, acceleration, force) into digital signals.

**The sensor challenge**: Robots must answer fundamental questions:
- **Where am I?** (localization) - Requires odometry, IMU, GPS, or visual landmarks
- **What's around me?** (perception) - Requires cameras, LiDAR, or range sensors
- **How am I moving?** (proprioception) - Requires joint encoders, force sensors, IMUs
- **What am I touching?** (tactile sensing) - Requires force/torque sensors, tactile arrays

The quality and diversity of sensors directly determine a robot's capabilities. A mobile robot with only forward-facing sensors cannot detect obstacles behind it. A manipulator without force sensing cannot grasp fragile objects safely.

This chapter explores the main sensor categories used in Physical AI systems and how they enable intelligent embodied behavior.

## 1. Vision Sensors (RGB and Depth)

### RGB Cameras

RGB (Red-Green-Blue) cameras are the most common vision sensors, capturing 2D color images similar to smartphone cameras.

**How they work:**
- Light passes through a lens onto a sensor (CCD or CMOS)
- Sensor array converts photons to electrical signals
- Image processor creates digital image (e.g., 1920×1080 pixels, 3 channels)

**Advantages:**
- ✅ Rich visual information (color, texture, patterns)
- ✅ High resolution (1080p, 4K, 8K)
- ✅ Inexpensive and widely available
- ✅ Excellent for object recognition and scene understanding

**Limitations:**
- ❌ No depth information (2D projection of 3D world)
- ❌ Sensitive to lighting conditions
- ❌ Requires complex algorithms to extract 3D information
- ❌ Struggles in low light or high contrast

**Common uses:**
- Object detection and classification
- Lane detection for autonomous vehicles
- Visual SLAM (Simultaneous Localization and Mapping)
- QR code/AprilTag detection
- Human pose estimation

### Depth Cameras

Depth cameras add a third dimension by measuring distance to each pixel.

**Technologies:**

1. **Stereo Vision** (passive)
   - Uses two cameras (like human eyes)
   - Computes depth from disparity between images
   - Examples: ZED camera, OAK-D

2. **Structured Light** (active)
   - Projects known pattern onto scene
   - Measures pattern deformation to compute depth
   - Examples: Intel RealSense SR300 (discontinued)

3. **Time-of-Flight (ToF)** (active)
   - Measures time for light pulse to return
   - Computes distance from speed of light
   - Examples: Intel RealSense D435, Azure Kinect

**Depth camera output:**
- **RGB image**: Standard color image (H × W × 3)
- **Depth map**: Distance values for each pixel (H × W × 1)
- **Point cloud**: 3D points in space (N × 3), where N = H × W

**Advantages:**
- ✅ Direct 3D information
- ✅ Enables accurate distance measurement
- ✅ Useful for obstacle avoidance and manipulation
- ✅ Works in textureless environments

**Limitations:**
- ❌ Limited range (typically 0.3-10 meters)
- ❌ Struggles with transparent/reflective surfaces
- ❌ Active sensors affected by sunlight
- ❌ Higher cost than RGB cameras

**Common uses:**
- 3D object detection and pose estimation
- Obstacle avoidance for mobile robots
- Grasp point detection for manipulation
- 3D scene reconstruction
- Person tracking and gesture recognition

### Choosing Between RGB and Depth

| Task | RGB Camera | Depth Camera |
|------|------------|--------------|
| Object classification | ✅ Excellent | ⚠️ Can help |
| Distance measurement | ❌ Difficult | ✅ Direct |
| Outdoor use | ✅ Good | ⚠️ Limited (sunlight) |
| Grasp planning | ⚠️ Challenging | ✅ Essential |
| Visual SLAM | ✅ Mature | ✅ Emerging |
| Low light | ⚠️ Poor | ✅ Better (active) |

**Best practice**: Use both! RGB for semantic understanding, depth for geometric reasoning.

## 2. LiDAR and Range Sensing

### What is LiDAR?

LiDAR (Light Detection and Ranging) measures distance by timing laser pulses.

**Principle:**
1. Emit laser pulse in a direction
2. Measure time until reflection returns
3. Calculate distance: `d = (c × t) / 2` where c = speed of light
4. Rotate to scan 360° (2D) or entire environment (3D)

**Output**: Point cloud of (x, y, z) coordinates showing obstacle locations

### Types of LiDAR

**1. 2D LiDAR (Planar Scanning)**
- Scans in a single plane (e.g., horizontal)
- Common in mobile robots for navigation
- Examples: SICK TIM, Hokuyo URG, RPLiDAR A1

**Characteristics:**
- Range: 0.1-30 meters (varies by model)
- Angular resolution: 0.25-1 degree
- Scan rate: 5-15 Hz
- Cost: $100-$5,000

**2. 3D LiDAR (Volumetric Scanning)**
- Multiple scanning planes or rotating head
- Creates 3D point cloud of environment
- Examples: Velodyne VLP-16, Ouster OS1, Livox Mid-360

**Characteristics:**
- Range: 10-200 meters
- Millions of points per second
- Scan rate: 5-20 Hz
- Cost: $1,000-$75,000+

**3. Solid-State LiDAR**
- No moving parts (flash LiDAR or MEMS mirrors)
- More durable, compact, cheaper
- Emerging technology for autonomous vehicles

### LiDAR vs Other Sensors

| Feature | LiDAR | Camera | Ultrasonic | Radar |
|---------|-------|--------|------------|-------|
| Range | 0.1-200m | 0-∞ | 0.02-5m | 1-300m |
| Resolution | High | Very High | Low | Medium |
| Weather | Good | Poor | Good | Excellent |
| Cost | High | Low | Very Low | Medium |
| Sunlight | Immune | Sensitive | Immune | Immune |
| Texture | No | Yes | No | No |

**Advantages:**
- ✅ Accurate distance measurement (&plusmn;2cm)
- ✅ Works in any lighting condition
- ✅ Long range
- ✅ Direct 3D geometry

**Limitations:**
- ❌ No color/texture information
- ❌ Expensive (especially 3D)
- ❌ Struggles with transparent/absorptive surfaces
- ❌ Data processing overhead

**Common uses:**
- SLAM (Simultaneous Localization and Mapping)
- Obstacle detection and avoidance
- 3D mapping and surveying
- Autonomous vehicle perception
- Warehouse robot navigation

### Ultrasonic and IR Range Sensors

**Ultrasonic sensors:**
- Emit high-frequency sound waves (40 kHz)
- Measure time for echo to return
- Range: 2cm - 5m
- Very inexpensive ($1-10)
- Wide beam angle (~15-30°)

**Infrared (IR) distance sensors:**
- Emit IR light and measure reflected intensity
- Range: 4cm - 5m (varies by model)
- Affected by surface color/reflectivity
- Sharp GP2Y0A21YK example

**Use cases:**
- Collision avoidance for small robots
- Parking sensors in vehicles
- Liquid level detection
- Proximity detection

## 3. Inertial Measurement Units (IMUs)

### What is an IMU?

An IMU combines multiple sensors to measure orientation, angular velocity, and linear acceleration.

**Components:**

1. **Accelerometer** (3-axis)
   - Measures linear acceleration in x, y, z
   - Detects gravity direction when stationary
   - Used to estimate tilt/roll

2. **Gyroscope** (3-axis)
   - Measures angular velocity (rotation rate)
   - Detects how fast robot is rotating
   - Drifts over time (integration error)

3. **Magnetometer** (3-axis) - optional
   - Measures magnetic field (like a compass)
   - Determines heading relative to Earth's magnetic north
   - Affected by ferromagnetic objects

**9-DOF IMU** = 3-axis accel + 3-axis gyro + 3-axis mag

### How IMUs Work

**Sensor fusion:**
IMUs combine accelerometer and gyroscope data to estimate orientation:

- **Accelerometer alone**: Good for tilt, but noisy and affected by movement
- **Gyroscope alone**: Smooth but drifts over time
- **Fusion (complementary filter, Kalman filter)**: Best of both

**Common output:**
- **Euler angles**: Roll, pitch, yaw (in degrees or radians)
- **Quaternion**: 4D representation avoiding gimbal lock
- **Rotation matrix**: 3×3 matrix for transformations

### IMU Applications in Robotics

**1. Orientation tracking**
- Drones: Maintain level flight
- Humanoids: Balance control
- Mobile robots: Heading estimation

**2. Dead reckoning**
- Estimate position by integrating acceleration
- Useful when GPS/vision unavailable
- Accumulates error quickly (double integration)

**3. Sensor fusion**
- Combine with wheel odometry for better localization
- Merge with visual SLAM for robust navigation
- Improve state estimation in Kalman filters

**Example: Quadcopter stabilization**
```python
class QuadcopterStabilizer:
    def __init__(self):
        self.imu = IMU()
        self.target_roll = 0
        self.target_pitch = 0

    def update(self):
        # Read current orientation from IMU
        roll, pitch, yaw = self.imu.get_euler_angles()

        # Compute error
        roll_error = self.target_roll - roll
        pitch_error = self.target_pitch - pitch

        # Apply PD control
        roll_correction = self.kp * roll_error + self.kd * self.imu.gyro_x
        pitch_correction = self.kp * pitch_error + self.kd * self.imu.gyro_y

        # Adjust motor speeds
        self.adjust_motors(roll_correction, pitch_correction)
```

**Common IMUs:**
- Cheap: MPU6050 ($2-5) - 6-DOF
- Mid-range: BNO055 ($20-30) - 9-DOF with on-board fusion
- High-end: VectorNav VN-100 ($500+) - Industrial-grade

## 4. Proprioceptive Sensors

Proprioceptive sensors measure the robot's internal state (as opposed to exteroceptive sensors that measure the environment).

### Joint Encoders

**Purpose**: Measure joint angles or positions

**Types:**

1. **Incremental encoders**
   - Count pulses as shaft rotates
   - Relative position only
   - Need homing sequence

2. **Absolute encoders**
   - Know position immediately on power-up
   - More expensive
   - Common in industrial robots

**Resolution**: Measured in counts per revolution (CPR)
- Low: 100-500 CPR
- Medium: 1000-2048 CPR
- High: 4096-16384 CPR

**Applications:**
- Joint position feedback for control
- Odometry (wheel rotation → distance traveled)
- Gripper opening width
- Elevator/lift position

### Force/Torque Sensors

**Purpose**: Measure forces and torques applied to robot

**6-DOF F/T sensor**: Measures:
- Forces: Fx, Fy, Fz
- Torques: Tx, Ty, Tz

**Applications:**
- **Compliant manipulation**: Adjust grip force
- **Contact detection**: Detect collisions
- **Force control**: Polish, assemble, sand
- **Human-robot collaboration**: Safety

**Example: Adaptive grasping**
```python
def adaptive_grasp(object, target_force=10.0):
    """Close gripper until reaching target force"""
    gripper.open()

    while gripper.get_force() < target_force:
        gripper.close(speed=0.01)  # Slow close
        time.sleep(0.01)

        if gripper.is_fully_closed():
            print("Cannot grasp - object too large or missing")
            return False

    print(f"Grasped with {gripper.get_force()}N force")
    return True
```

### Current/Torque Sensing

**Motor current monitoring:**
- Measure current draw of motors
- Infer applied torque (torque ∝ current)
- Detect collisions (current spike)
- Cheaper alternative to F/T sensors

**Use cases:**
- Collision detection without external sensors
- Torque limiting for safety
- Detecting stuck motors
- Battery monitoring

## 5. Sensor Fusion Strategies

No single sensor is perfect. Sensor fusion combines multiple sensors to achieve better perception than any individual sensor.

### Why Fuse Sensors?

**Complementary strengths:**
- Camera: Rich semantic info, no depth
- Depth camera: 3D info, limited range
- LiDAR: Long range 3D, no color
- IMU: Fast orientation, drifts over time
- Wheel odometry: Smooth short-term, drifts long-term

**Goals:**
- ✅ Increase accuracy
- ✅ Increase robustness (sensor failure tolerance)
- ✅ Expand operational envelope
- ✅ Reduce uncertainty

### Common Fusion Approaches

**1. Complementary Filter**
- Simple weighted average of sensors
- Example: Fuse accelerometer (low-pass) + gyroscope (high-pass)
- Fast, lightweight, good for IMU orientation

**2. Kalman Filter**
- Optimal fusion under Gaussian noise assumptions
- Predicts state, then corrects with measurements
- Example: Fuse GPS + IMU + wheel odometry for robot position

**3. Particle Filter**
- Non-parametric, handles non-Gaussian distributions
- Used in Monte Carlo Localization (MCL)
- Computationally expensive

**4. Sensor-specific fusion**
- **Visual-Inertial Odometry (VIO)**: Camera + IMU
- **LiDAR-Inertial Odometry (LIO)**: LiDAR + IMU
- **Multi-modal SLAM**: Camera + LiDAR + wheel encoders

### Example: Robot Localization Fusion

```python
class RobotLocalizer:
    def __init__(self):
        self.position = (0, 0, 0)  # x, y, theta
        self.covariance = np.eye(3)

    def predict(self, wheel_odom, dt):
        """Prediction step using wheel odometry"""
        # Move predicted position based on odometry
        dx, dy, dtheta = wheel_odom
        self.position = integrate_motion(self.position, dx, dy, dtheta)

        # Increase uncertainty (odometry drifts)
        self.covariance += process_noise(dt)

    def update_lidar(self, lidar_scan):
        """Correction step using LiDAR scan matching"""
        # Match current scan to map
        position_estimate = scan_match(lidar_scan, self.map)

        # Fuse with prediction using Kalman update
        self.position, self.covariance = kalman_update(
            self.position, self.covariance,
            position_estimate, lidar_noise
        )

    def update_imu(self, imu_data):
        """Correction step for orientation using IMU"""
        theta_estimate = imu_data.yaw

        # Update only theta component
        self.position[2] = kalman_update_1d(
            self.position[2], self.covariance[2,2],
            theta_estimate, imu_noise
        )
```

### Best Practices

1. **Sensor selection**: Choose sensors with complementary strengths
2. **Calibration**: Properly calibrate all sensors (intrinsics, extrinsics)
3. **Time synchronization**: Timestamp all measurements accurately
4. **Outlier rejection**: Detect and discard anomalous readings
5. **Graceful degradation**: System should work even if sensors fail

## Summary

Key takeaways from this chapter:

- **Vision sensors** provide rich semantic information but struggle with depth
- **Depth cameras** add geometric understanding but have limited range
- **LiDAR** offers accurate long-range 3D sensing but is expensive
- **IMUs** track orientation and acceleration, essential for balance and navigation
- **Proprioceptive sensors** monitor internal state (joint angles, forces)
- **Sensor fusion** combines strengths of multiple sensors for robust perception

**Choosing sensors depends on:**
- Environment (indoor/outdoor, lighting, weather)
- Task requirements (precision, range, speed)
- Budget constraints
- Computational resources

## Review Questions

1. What is the main advantage of depth cameras over RGB cameras?
2. Why does LiDAR work better than cameras in poor lighting?
3. What causes IMU gyroscopes to drift over time?
4. When would you use a force sensor instead of vision for grasping?
5. Why combine wheel odometry with IMU instead of using just one?
6. What is the difference between proprioceptive and exteroceptive sensors?
7. How does sensor fusion improve robot localization?

## Hands-on Exercises

### Exercise 1: Camera vs Depth Camera
- Capture RGB and depth images of the same scene
- Attempt to measure object distance from RGB alone
- Compare with direct depth measurement
- Discuss accuracy and challenges

### Exercise 2: IMU Orientation Tracking
- Read IMU data (accelerometer, gyroscope)
- Implement complementary filter to estimate roll/pitch
- Compare against gyroscope-only and accelerometer-only
- Observe drift and noise characteristics

### Exercise 3: Simple Sensor Fusion
- Collect wheel odometry and IMU yaw
- Implement dead reckoning using odometry alone
- Add IMU yaw to correct heading drift
- Compare trajectories with/without fusion

## Further Reading

- [Camera Calibration and 3D Reconstruction](https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html)
- [LiDAR for Robotics](https://velodynelidar.com/blog/lidar-applications-robotics/)
- [IMU Sensor Fusion](https://www.nxp.com/docs/en/application-note/AN3461.pdf)
- [Kalman Filtering Tutorial](https://www.kalmanfilter.net/)

---

**Next Chapter**: [Actuators and Robotics Architectures →](ch03-actuators-robotics-arch.md)

**Previous Chapter**: [← Embodied Intelligence](../week-01/ch01-embodied-intelligence.md)
