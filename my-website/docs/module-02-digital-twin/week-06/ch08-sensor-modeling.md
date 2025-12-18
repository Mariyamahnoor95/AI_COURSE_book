---
id: ch08-sensor-modeling
title: Sensor Modeling in Simulation
sidebar_label: Sensor Modeling in Simulation
sidebar_position: 9
---

# Sensor Modeling in Simulation

## Learning Objectives

By the end of this chapter, you will be able to:

- Configure realistic camera sensors in Gazebo (RGB, depth, fisheye)
- Implement LiDAR sensors with ray tracing and GPU acceleration
- Model IMU sensors with realistic noise characteristics
- Leverage ground truth data for algorithm development and validation
- Understand and address the sim-to-real gap in sensor modeling

## Introduction

Real-world sensors are noisy, imperfect, and expensive. Cameras have lens distortion, depth sensors have limited range, LiDAR misses transparent objects, IMUs drift over time. **How do we develop and test perception algorithms without needing dozens of physical robots and sensors?**

This is where **sensor simulation** becomes critical. Modern simulators like Gazebo and NVIDIA Isaac Sim can model sensor physics with remarkable realism:
- **Cameras** with proper optics, rolling shutter, motion blur
- **LiDAR** with ray tracing, reflection properties, atmospheric effects
- **IMUs** with gyro drift, accelerometer noise, temperature effects
- **Ground truth** data that's impossible to get in the real world

**Why sensor simulation matters:**
- ✅ **Rapid prototyping**: Test algorithms without hardware
- ✅ **Edge case testing**: Create scenarios hard to reproduce (rain, night, obstacles)
- ✅ **Data generation**: Create labeled datasets for machine learning
- ✅ **Safe testing**: Crash virtual robots, not real ones
- ✅ **Parallel development**: Hardware and software teams work concurrently

**Real-world scenario**: You're developing a vision-based navigation system. With sensor simulation, you can:
1. Generate 10,000 labeled images in a day
2. Test performance in fog, rain, night conditions
3. Validate algorithms before hardware arrives
4. Identify edge cases that would take months to encounter

However, simulation isn't perfect - there's always a **sim-to-real gap** between virtual and physical sensors. This chapter teaches you how to model sensors realistically and minimize this gap.

## 1. Camera Simulation

### Types of Simulated Cameras

**1. RGB Camera** - Standard color images
**2. Depth Camera** - Distance to each pixel
**3. Segmentation Camera** - Per-pixel labels (for ground truth)
**4. Fisheye Camera** - Wide field-of-view with distortion

### Configuring RGB Camera in Gazebo

In URDF/Xacro with Gazebo plugin:

```xml
&lt;gazebo reference="camera_link"&gt;
  &lt;sensor name="rgb_camera" type="camera"&gt;
    &lt;always_on&gt;true&lt;/always_on&gt;
    &lt;update_rate&gt;30&lt;/update_rate&gt;
    &lt;visualize&gt;true&lt;/visualize&gt;

    &lt;camera name="front_camera"&gt;
      &lt;!-- Field of view (radians) --&gt;
      &lt;horizontal_fov&gt;1.3962634&lt;/horizontal_fov&gt;  &lt;!-- 80 degrees --&gt;

      &lt;!-- Image resolution --&gt;
      &lt;image&gt;
        &lt;width&gt;1920&lt;/width&gt;
        &lt;height&gt;1080&lt;/height&gt;
        &lt;format&gt;R8G8B8&lt;/format&gt;
      &lt;/image&gt;

      &lt;!-- Clipping planes --&gt;
      &lt;clip&gt;
        &lt;near&gt;0.1&lt;/near&gt;  &lt;!-- Min distance --&gt;
        &lt;far&gt;100.0&lt;/far&gt;   &lt;!-- Max distance --&gt;
      &lt;/clip&gt;

      &lt;!-- Lens distortion (optional) --&gt;
      &lt;distortion&gt;
        &lt;k1&gt;0.1&lt;/k1&gt;  &lt;!-- Radial distortion --&gt;
        &lt;k2&gt;0.05&lt;/k2&gt;
        &lt;k3&gt;0.01&lt;/k3&gt;
        &lt;p1&gt;0.001&lt;/p1&gt;  &lt;!-- Tangential distortion --&gt;
        &lt;p2&gt;0.001&lt;/p2&gt;
        &lt;center&gt;0.5 0.5&lt;/center&gt;
      &lt;/distortion&gt;

      &lt;!-- Image noise --&gt;
      &lt;noise&gt;
        &lt;type&gt;gaussian&lt;/type&gt;
        &lt;mean&gt;0.0&lt;/mean&gt;
        &lt;stddev&gt;0.007&lt;/stddev&gt;
      &lt;/noise&gt;
    &lt;/camera&gt;

    &lt;!-- ROS 2 plugin to publish images --&gt;
    &lt;plugin name="camera_controller" filename="libgazebo_ros_camera.so"&gt;
      &lt;ros&gt;
        &lt;namespace&gt;/camera&lt;/namespace&gt;
        &lt;remapping&gt;image_raw:=rgb&lt;/remapping&gt;
        &lt;remapping&gt;camera_info:=camera_info&lt;/remapping&gt;
      &lt;/ros&gt;
      &lt;camera_name&gt;front_camera&lt;/camera_name&gt;
      &lt;frame_name&gt;camera_link&lt;/frame_name&gt;
      &lt;hack_baseline&gt;0.07&lt;/hack_baseline&gt;
    &lt;/plugin&gt;
  &lt;/sensor&gt;
&lt;/gazebo&gt;
```

**Published topics:**
- `/camera/rgb` (sensor_msgs/Image) - Color images
- `/camera/camera_info` (sensor_msgs/CameraInfo) - Intrinsic parameters

### Depth Camera

```xml
&lt;gazebo reference="depth_camera_link"&gt;
  &lt;sensor name="depth_camera" type="depth"&gt;
    &lt;update_rate&gt;20&lt;/update_rate&gt;
    &lt;camera&gt;
      &lt;horizontal_fov&gt;1.047198&lt;/horizontal_fov&gt;
      &lt;image&gt;
        &lt;width&gt;640&lt;/width&gt;
        &lt;height&gt;480&lt;/height&gt;
      &lt;/image&gt;
      &lt;clip&gt;
        &lt;near&gt;0.3&lt;/near&gt;  &lt;!-- Depth cameras have limited min range --&gt;
        &lt;far&gt;10.0&lt;/far&gt;
      &lt;/clip&gt;
    &lt;/camera&gt;

    &lt;plugin name="depth_camera_controller" filename="libgazebo_ros_camera.so"&gt;
      &lt;ros&gt;
        &lt;namespace&gt;/depth_camera&lt;/namespace&gt;
        &lt;remapping&gt;depth/image_raw:=depth&lt;/remapping&gt;
      &lt;/ros&gt;
      &lt;min_depth&gt;0.3&lt;/min_depth&gt;
      &lt;max_depth&gt;10.0&lt;/max_depth&gt;
    &lt;/plugin&gt;
  &lt;/sensor&gt;
&lt;/gazebo&gt;
```

**Published topics:**
- `/depth_camera/depth` (sensor_msgs/Image, 32FC1) - Depth map in meters
- `/depth_camera/points` (sensor_msgs/PointCloud2) - 3D point cloud

### Camera Intrinsic Parameters

The `camera_info` message contains calibration data:

```python
# Intrinsic matrix K
# [ fx  0  cx ]
# [  0 fy  cy ]
# [  0  0   1 ]

# fx, fy: focal length in pixels
# cx, cy: principal point (image center)

# Calculated from FOV:
# fx = (image_width / 2) / tan(horizontal_fov / 2)
# fy = (image_height / 2) / tan(vertical_fov / 2)
```

### Visualizing Camera Output

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraViewer(Node):
    def __init__(self):
        super().__init__('camera_viewer')
        self.bridge = CvBridge()

        self.rgb_sub = self.create_subscription(
            Image, '/camera/rgb', self.rgb_callback, 10
        )

    def rgb_callback(self, msg):
        # Convert ROS Image to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Display
        cv2.imshow('Camera', cv_image)
        cv2.waitKey(1)

def main():
    rclpy.init()
    node = CameraViewer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

## 2. LiDAR and Ray Tracing

### 2D LiDAR (Planar Scan)

2D LiDAR scans in a horizontal plane (common for mobile robots):

```xml
&lt;gazebo reference="lidar_link"&gt;
  &lt;sensor name="lidar_2d" type="gpu_ray"&gt;
    &lt;always_on&gt;true&lt;/always_on&gt;
    &lt;update_rate&gt;10&lt;/update_rate&gt;
    &lt;visualize&gt;true&lt;/visualize&gt;

    &lt;ray&gt;
      &lt;scan&gt;
        &lt;horizontal&gt;
          &lt;samples&gt;360&lt;/samples&gt;        &lt;!-- 360 laser beams --&gt;
          &lt;resolution&gt;1&lt;/resolution&gt;    &lt;!-- Sample every beam --&gt;
          &lt;min_angle&gt;-3.14159&lt;/min_angle&gt;  &lt;!-- -180 degrees --&gt;
          &lt;max_angle&gt;3.14159&lt;/max_angle&gt;   &lt;!-- +180 degrees --&gt;
        &lt;/horizontal&gt;
      &lt;/scan&gt;

      &lt;range&gt;
        &lt;min&gt;0.1&lt;/min&gt;   &lt;!-- 10cm minimum --&gt;
        &lt;max&gt;30.0&lt;/max&gt;  &lt;!-- 30m maximum --&gt;
        &lt;resolution&gt;0.01&lt;/resolution&gt;  &lt;!-- 1cm resolution --&gt;
      &lt;/range&gt;

      &lt;!-- Ray properties --&gt;
      &lt;noise&gt;
        &lt;type&gt;gaussian&lt;/type&gt;
        &lt;mean&gt;0.0&lt;/mean&gt;
        &lt;stddev&gt;0.01&lt;/stddev&gt;  &lt;!-- 1cm noise --&gt;
      &lt;/noise&gt;
    &lt;/ray&gt;

    &lt;plugin name="lidar_plugin" filename="libgazebo_ros_ray_sensor.so"&gt;
      &lt;ros&gt;
        &lt;namespace&gt;/&lt;/namespace&gt;
        &lt;remapping&gt;~/out:=scan&lt;/remapping&gt;
      &lt;/ros&gt;
      &lt;output_type&gt;sensor_msgs/LaserScan&lt;/output_type&gt;
      &lt;frame_name&gt;lidar_link&lt;/frame_name&gt;
    &lt;/plugin&gt;
  &lt;/sensor&gt;
&lt;/gazebo&gt;
```

**Published topic:**
- `/scan` (sensor_msgs/LaserScan) - 2D range data

### 3D LiDAR (Volumetric)

3D LiDAR with multiple vertical layers:

```xml
&lt;gazebo reference="lidar_3d_link"&gt;
  &lt;sensor name="lidar_3d" type="gpu_ray"&gt;
    &lt;update_rate&gt;10&lt;/update_rate&gt;
    &lt;ray&gt;
      &lt;scan&gt;
        &lt;horizontal&gt;
          &lt;samples&gt;1024&lt;/samples&gt;
          &lt;min_angle&gt;-3.14159&lt;/min_angle&gt;
          &lt;max_angle&gt;3.14159&lt;/max_angle&gt;
        &lt;/horizontal&gt;
        &lt;vertical&gt;
          &lt;samples&gt;16&lt;/samples&gt;  &lt;!-- 16 laser layers --&gt;
          &lt;min_angle&gt;-0.2618&lt;/min_angle&gt;  &lt;!-- -15 degrees down --&gt;
          &lt;max_angle&gt;0.2618&lt;/max_angle&gt;   &lt;!-- +15 degrees up --&gt;
        &lt;/vertical&gt;
      &lt;/scan&gt;
      &lt;range&gt;
        &lt;min&gt;0.5&lt;/min&gt;
        &lt;max&gt;100.0&lt;/max&gt;
      &lt;/range&gt;
    &lt;/ray&gt;

    &lt;plugin name="lidar_3d_plugin" filename="libgazebo_ros_ray_sensor.so"&gt;
      &lt;ros&gt;
        &lt;remapping&gt;~/out:=points&lt;/remapping&gt;
      &lt;/ros&gt;
      &lt;output_type&gt;sensor_msgs/PointCloud2&lt;/output_type&gt;
    &lt;/plugin&gt;
  &lt;/sensor&gt;
&lt;/gazebo&gt;
```

**Published topic:**
- `/points` (sensor_msgs/PointCloud2) - 3D point cloud

### GPU vs CPU Ray Tracing

| Feature | CPU Ray (`ray`) | GPU Ray (`gpu_ray`) |
|---------|-----------------|---------------------|
| Speed | Slow | 10-100x faster |
| Accuracy | Identical | Identical |
| Samples | &lt;100 rays | 1000+ rays |
| Use case | Simple 2D LiDAR | 3D LiDAR, many sensors |

**Always use `gpu_ray` for:**
- 3D LiDAR with &gt;100 samples
- Multiple LiDAR sensors
- Real-time applications

### LiDAR Point Cloud Processing

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

class LidarProcessor(Node):
    def __init__(self):
        super().__init__('lidar_processor')
        self.sub = self.create_subscription(
            PointCloud2, '/points', self.lidar_callback, 10
        )

    def lidar_callback(self, msg):
        # Convert PointCloud2 to list of (x, y, z)
        points = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)

        # Process points
        obstacle_count = 0
        for x, y, z in points:
            distance = (x**2 + y**2 + z**2) ** 0.5
            if distance &lt; 1.0:  # Obstacle within 1m
                obstacle_count += 1

        if obstacle_count &gt; 10:
            self.get_logger().warn(f'Obstacle detected: {obstacle_count} points')
```

## 3. IMU and Noise Models

### What is an IMU in Simulation?

IMUs measure:
- **Linear acceleration** (m/s²) in x, y, z
- **Angular velocity** (rad/s) in roll, pitch, yaw
- **Orientation** (quaternion) - often from sensor fusion

### Configuring IMU in Gazebo

```xml
&lt;gazebo reference="imu_link"&gt;
  &lt;sensor name="imu_sensor" type="imu"&gt;
    &lt;always_on&gt;true&lt;/always_on&gt;
    &lt;update_rate&gt;100&lt;/update_rate&gt;  &lt;!-- IMUs are high-frequency --&gt;

    &lt;imu&gt;
      &lt;!-- Angular velocity noise --&gt;
      &lt;angular_velocity&gt;
        &lt;x&gt;
          &lt;noise type="gaussian"&gt;
            &lt;mean&gt;0.0&lt;/mean&gt;
            &lt;stddev&gt;0.0002&lt;/stddev&gt;
            &lt;bias_mean&gt;0.00001&lt;/bias_mean&gt;  &lt;!-- Gyro drift --&gt;
            &lt;bias_stddev&gt;0.000001&lt;/bias_stddev&gt;
          &lt;/noise&gt;
        &lt;/x&gt;
        &lt;y&gt;
          &lt;noise type="gaussian"&gt;
            &lt;mean&gt;0.0&lt;/mean&gt;
            &lt;stddev&gt;0.0002&lt;/stddev&gt;
            &lt;bias_mean&gt;0.00001&lt;/bias_mean&gt;
            &lt;bias_stddev&gt;0.000001&lt;/bias_stddev&gt;
          &lt;/noise&gt;
        &lt;/y&gt;
        &lt;z&gt;
          &lt;noise type="gaussian"&gt;
            &lt;mean&gt;0.0&lt;/mean&gt;
            &lt;stddev&gt;0.0002&lt;/stddev&gt;
            &lt;bias_mean&gt;0.00001&lt;/bias_mean&gt;
            &lt;bias_stddev&gt;0.000001&lt;/bias_stddev&gt;
          &lt;/noise&gt;
        &lt;/z&gt;
      &lt;/angular_velocity&gt;

      &lt;!-- Linear acceleration noise --&gt;
      &lt;linear_acceleration&gt;
        &lt;x&gt;
          &lt;noise type="gaussian"&gt;
            &lt;mean&gt;0.0&lt;/mean&gt;
            &lt;stddev&gt;0.017&lt;/stddev&gt;  &lt;!-- Typical accelerometer --&gt;
            &lt;bias_mean&gt;0.1&lt;/bias_mean&gt;
            &lt;bias_stddev&gt;0.001&lt;/bias_stddev&gt;
          &lt;/noise&gt;
        &lt;/x&gt;
        &lt;y&gt;
          &lt;noise type="gaussian"&gt;
            &lt;mean&gt;0.0&lt;/mean&gt;
            &lt;stddev&gt;0.017&lt;/stddev&gt;
            &lt;bias_mean&gt;0.1&lt;/bias_mean&gt;
            &lt;bias_stddev&gt;0.001&lt;/bias_stddev&gt;
          &lt;/noise&gt;
        &lt;/y&gt;
        &lt;z&gt;
          &lt;noise type="gaussian"&gt;
            &lt;mean&gt;0.0&lt;/mean&gt;
            &lt;stddev&gt;0.017&lt;/stddev&gt;
            &lt;bias_mean&gt;0.1&lt;/bias_mean&gt;
            &lt;bias_stddev&gt;0.001&lt;/bias_stddev&gt;
          &lt;/noise&gt;
        &lt;/z&gt;
      &lt;/linear_acceleration&gt;
    &lt;/imu&gt;

    &lt;plugin name="imu_plugin" filename="libgazebo_ros_imu_sensor.so"&gt;
      &lt;ros&gt;
        &lt;namespace&gt;/imu&lt;/namespace&gt;
        &lt;remapping&gt;~/out:=data&lt;/remapping&gt;
      &lt;/ros&gt;
      &lt;frame_name&gt;imu_link&lt;/frame_name&gt;
      &lt;initial_orientation_as_reference&gt;false&lt;/initial_orientation_as_reference&gt;
    &lt;/plugin&gt;
  &lt;/sensor&gt;
&lt;/gazebo&gt;
```

**Published topic:**
- `/imu/data` (sensor_msgs/Imu)

### Noise Model Parameters

**Gaussian noise**: Random error around mean
- `mean`: Average offset (often 0)
- `stddev`: Standard deviation (variability)

**Bias**: Systematic error that drifts
- `bias_mean`: Average bias offset
- `bias_stddev`: How much bias changes

**Realistic values** (based on common IMUs):

| Sensor | Noise (stddev) | Bias Mean | Application |
|--------|----------------|-----------|-------------|
| Gyro (cheap) | 0.01 rad/s | 0.001 rad/s | Hobby drones |
| Gyro (good) | 0.0002 rad/s | 0.00001 rad/s | Industrial robots |
| Accel (cheap) | 0.1 m/s² | 0.5 m/s² | Hobby projects |
| Accel (good) | 0.017 m/s² | 0.1 m/s² | Autonomous vehicles |

### Using IMU Data

```python
from sensor_msgs.msg import Imu

class IMUReader(Node):
    def __init__(self):
        super().__init__('imu_reader')
        self.sub = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)

    def imu_callback(self, msg):
        # Orientation (quaternion)
        q = msg.orientation
        # Convert to Euler angles for human readability
        roll, pitch, yaw = self.quaternion_to_euler(q.x, q.y, q.z, q.w)

        # Angular velocity (rad/s)
        gyro_x = msg.angular_velocity.x
        gyro_y = msg.angular_velocity.y
        gyro_z = msg.angular_velocity.z

        # Linear acceleration (m/s²)
        accel_x = msg.linear_acceleration.x
        accel_y = msg.linear_acceleration.y
        accel_z = msg.linear_acceleration.z

        self.get_logger().info(
            f'Orientation: roll={roll:.2f}, pitch={pitch:.2f}, yaw={yaw:.2f}'
        )
```

## 4. Ground Truth Data

### What is Ground Truth?

**Ground truth** = perfect, noise-free data about the environment

In simulation, you can access:
- ✅ Exact robot pose (position + orientation)
- ✅ Perfect object locations
- ✅ Semantic labels for every pixel
- ✅ Exact velocities and accelerations

**Why ground truth is valuable:**
- **Algorithm validation**: Compare estimated pose vs true pose
- **Dataset labeling**: Auto-label training data for ML
- **Performance metrics**: Measure localization error
- **Debugging**: Identify where algorithms fail

### Accessing Ground Truth Pose

```python
from gazebo_msgs.srv import GetModelState

class GroundTruthTracker(Node):
    def __init__(self):
        super().__init__('ground_truth_tracker')

        # Service client to get model state
        self.client = self.create_client(GetModelState, '/get_model_state')

        self.timer = self.create_timer(0.1, self.get_true_pose)

    def get_true_pose(self):
        request = GetModelState.Request()
        request.model_name = 'my_robot'
        request.relative_entity_name = 'world'

        future = self.client.call_async(request)
        future.add_done_callback(self.handle_response)

    def handle_response(self, future):
        try:
            response = future.result()
            pose = response.pose

            self.get_logger().info(
                f'True position: ({pose.position.x:.2f}, {pose.position.y:.2f})'
            )
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
```

### Semantic Segmentation Camera

Get per-pixel labels for object classes:

```xml
&lt;gazebo reference="seg_camera_link"&gt;
  &lt;sensor name="segmentation_camera" type="segmentation"&gt;
    &lt;update_rate&gt;10&lt;/update_rate&gt;
    &lt;camera&gt;
      &lt;image&gt;
        &lt;width&gt;640&lt;/width&gt;
        &lt;height&gt;480&lt;/height&gt;
      &lt;/image&gt;
    &lt;/camera&gt;
    &lt;plugin name="seg_plugin" filename="libgazebo_ros_segmentation_camera.so"&gt;
      &lt;ros&gt;
        &lt;remapping&gt;~/out:=segmentation&lt;/remapping&gt;
      &lt;/ros&gt;
    &lt;/plugin&gt;
  &lt;/sensor&gt;
&lt;/gazebo&gt;
```

**Output**: Image where pixel values = object class IDs

## 5. Sim-to-Real Considerations

### The Sim-to-Real Gap

**Problem**: Algorithms that work perfectly in simulation often fail on real robots.

**Causes:**
- ❌ Physics approximations (friction, contact, deformation)
- ❌ Sensor models don't match reality exactly
- ❌ Simulation is too clean (no dust, wear, vibration)
- ❌ Lighting and materials differ

### Strategies to Bridge the Gap

**1. Domain Randomization**

Add random variations to simulation:

```python
# Randomize lighting
light_intensity = random.uniform(0.5, 1.5)

# Randomize object textures
texture = random.choice(['wood', 'metal', 'plastic'])

# Randomize sensor noise
noise_stddev = random.uniform(0.01, 0.05)

# Randomize friction
friction = random.uniform(0.5, 1.5)
```

**Why it works**: Algorithms learn to be robust to variations

**2. System Identification**

Measure real robot parameters and update simulation:

```python
# Measure real robot mass by weighing
real_mass = 12.3  # kg

# Update URDF
&lt;inertial&gt;
  &lt;mass value="12.3"/&gt;  &lt;!-- Not default 10.0 --&gt;
&lt;/inertial&gt;

# Measure wheel friction by observing slip
real_friction_coefficient = 0.8

# Update Gazebo
&lt;gazebo reference="wheel"&gt;
  &lt;mu1&gt;0.8&lt;/mu1&gt;
  &lt;mu2&gt;0.8&lt;/mu2&gt;
&lt;/gazebo&gt;
```

**3. Sim-to-Real Transfer Learning**

Train in simulation, fine-tune on real data:

```python
# Stage 1: Pre-train in simulation (100k samples)
model.train(sim_dataset)

# Stage 2: Fine-tune on real data (1k samples)
model.fine_tune(real_dataset, epochs=10, lr=0.0001)
```

**4. Conservative Tuning**

Design for robustness, not performance:

```python
# Bad: Aggressive PID gains (work in sim, crash in reality)
kp_sim = 10.0

# Good: Conservative gains (stable in both)
kp_real = 3.0
```

**5. Realistic Sensor Modeling**

Match simulation noise to real sensor specs:

```python
# Measure real LiDAR noise
real_lidar_noise_cm = 2.5

# Configure simulation
&lt;noise&gt;
  &lt;stddev&gt;0.025&lt;/stddev&gt;  &lt;!-- 2.5cm in meters --&gt;
&lt;/noise&gt;
```

### Validation Workflow

```
1. Develop algorithm in simulation
2. Test with domain randomization
3. Validate on ground truth data
4. Deploy to real robot
5. Measure performance gap
6. Update simulation parameters
7. Re-train if gap is large
8. Iterate
```

## Summary

Key takeaways from this chapter:

- **Camera simulation** provides RGB, depth, and segmentation with realistic optics
- **LiDAR ray tracing** models 2D/3D sensors with GPU acceleration
- **IMU noise models** include gyro drift and accelerometer bias
- **Ground truth data** enables algorithm validation impossible in real world
- **Sim-to-real gap** requires domain randomization, system ID, and conservative design

**Best practices:**
- Always add realistic noise to sensors
- Use ground truth only for validation, not in main algorithms
- Test with domain randomization before real deployment
- Measure real sensor specs and match them in simulation
- Start conservative, tune aggressively only if needed
- Document sim parameters for reproducibility

## Review Questions

1. What is the difference between RGB and depth cameras in simulation?
2. Why use `gpu_ray` instead of `ray` for LiDAR?
3. What causes IMU gyroscope drift over time?
4. How does ground truth data help with algorithm development?
5. Name three strategies to reduce the sim-to-real gap.
6. Why add noise to simulated sensors instead of perfect data?
7. What is domain randomization and how does it help transfer learning?

## Hands-on Exercises

### Exercise 1: Camera Configuration
Create a robot with RGB and depth cameras:
- Configure 30 Hz RGB camera with 90° FOV
- Add Gaussian noise (stddev=0.01)
- Publish to `/camera/rgb` and `/camera/depth`
- Visualize in RViz and verify topics

### Exercise 2: LiDAR Obstacle Detection
Implement LiDAR-based obstacle avoidance:
- Configure 2D LiDAR (360°, 10m range)
- Subscribe to `/scan` topic
- Detect obstacles within 1m
- Publish warning messages
- Test in Gazebo with obstacles

### Exercise 3: Ground Truth Validation
Compare odometry to ground truth:
- Subscribe to `/odom` (from wheel encoders)
- Query ground truth pose from Gazebo
- Compute position error over time
- Plot error growth (demonstrates drift)
- Log results to file

## Further Reading

- [Gazebo Sensors Documentation](http://gazebosim.org/tutorials?tut=ros2_installing&cat=connect_ros)
- [ROS 2 Gazebo Plugins](https://github.com/ros-simulation/gazebo_ros_pkgs)
- [Domain Randomization for Sim-to-Real Transfer](https://arxiv.org/abs/1703.06907)
- [NVIDIA Isaac Sim Sensor Models](https://docs.omniverse.nvidia.com/isaacsim/latest/features/sensors_simulation/index.html)

---

**Next Chapter**: [Performance Monitoring →](ch09-performance-monitoring.md)

**Previous Chapter**: [← Gazebo Physics Simulation](ch07-gazebo-physics.md)
