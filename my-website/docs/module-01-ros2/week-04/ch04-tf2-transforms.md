---
id: ch04-tf2-transforms
title: Coordinate Transforms with TF2
sidebar_label: Coordinate Transforms
sidebar_position: 5
---

# Coordinate Transforms with TF2

## Learning Objectives

By the end of this chapter, you will be able to:

- Understand the TF2 transform library and its role in robotics
- Work with coordinate frames and the transform tree
- Broadcast static and dynamic transforms
- Look up transforms to convert between coordinate frames
- Debug transform issues using ROS 2 tools
- Apply TF2 in real robot applications

## Introduction

Every sensor and component on a robot has its own coordinate frame - the camera sees the world from one position, the LiDAR from another, wheels have their own reference points. **How do we relate measurements from all these different perspectives?**

This is the fundamental problem that **TF2** (Transform Library 2) solves. TF2 is ROS 2's distributed transform system that tracks the relationships between all coordinate frames over time, allowing you to:

- Transform sensor data between different coordinate frames
- Understand where robot parts are relative to each other
- Convert goals from "10 meters forward of the robot" to absolute map coordinates
- Synchronize data from multiple sensors with different positions and orientations

**Real-world scenario**: Your robot's camera detects an object at (0.5, 0.2) in camera coordinates. To command the robot to drive to that object, you need to:
1. Transform from camera frame → robot base frame
2. Transform from robot base → map frame (global coordinates)
3. Send navigation goal in map coordinates

Without TF2, you'd need to manually compute and manage dozens of geometric transformations. With TF2, you simply ask: "Where is this point in the map frame?" and the system handles all intermediate transformations automatically.

This chapter teaches you how to use TF2 to build robots that understand their geometry.

## 1. TF2 Library Fundamentals

### What is TF2?

**TF2** is a distributed system for tracking coordinate frame relationships that:
- Maintains a **tree structure** of coordinate frames
- Stores transforms with **timestamps** (transform history)
- Provides **efficient lookups** between any two frames
- Handles **time synchronization** for moving frames

### Core Concepts

**Coordinate Frame**: A 3D coordinate system (origin point + orientation)
- Every robot component has at least one frame
- Frames are named (e.g., `base_link`, `camera_link`, `odom`, `map`)

**Transform**: Describes how to convert from one frame to another
- **Translation**: (x, y, z) offset in meters
- **Rotation**: Quaternion (x, y, z, w) or Euler angles (roll, pitch, yaw)

**Transform Tree**: Directed acyclic graph of frame relationships
- Each frame has exactly **one parent**
- Frames can have **multiple children**
- There is typically one root frame (e.g., `map` or `odom`)

### TF2 vs TF (Original)

| Feature | TF (old) | TF2 (current) |
|---------|----------|---------------|
| Performance | Slower | 10x faster |
| Message types | Custom | Standard geometry_msgs |
| Extensibility | Limited | Plugin-based |
| Python API | tf | tf2_ros, tf2_geometry_msgs |

**Always use TF2 in ROS 2** - the original TF is deprecated.

### Key TF2 Packages

```python
import rclpy
from rclpy.node import Node

# Core TF2 libraries
import tf2_ros
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from tf2_ros import TransformListener, Buffer

# Message types
from geometry_msgs.msg import TransformStamped, PointStamped
from tf2_geometry_msgs import do_transform_point
```

## 2. Coordinate Frames in Robotics

### Standard Frame Naming Conventions

ROS 2 uses standardized frame names for consistency:

**1. `map`** - Fixed global frame
- Origin: Arbitrary point in the world
- Never moves
- Used for global navigation and localization

**2. `odom`** - Odometry frame
- Origin: Where robot started (or last reset)
- Continuous, but drifts over time
- Updated by wheel encoders or visual odometry

**3. `base_link`** - Robot's center
- Origin: Typically at center of rotation
- Moves with the robot
- **Most important frame** - all robot parts defined relative to this

**4. Sensor frames** - One per sensor
- `camera_link`, `lidar_link`, `imu_link`
- Fixed relative to `base_link`
- Defined by physical mounting position

**5. End-effector frames** (for manipulators)
- `gripper_link`, `tool_frame`
- May move relative to `base_link` (for robot arms)

### Transform Tree Example

```
map
 └─ odom
     └─ base_link
         ├─ laser_link
         ├─ camera_link
         └─ imu_link
```

**Reading this tree:**
- `map` → `odom`: Localization (updated by SLAM/AMCL)
- `odom` → `base_link`: Odometry (updated by wheel encoders)
- `base_link` → `laser_link`: Static (fixed sensor mount)

### Right-Handed Coordinate Systems

ROS 2 uses **right-handed coordinates** with standard axes:

```
z (up)
│   x (forward)
│  ╱
│ ╱
│╱_____ y (left)
```

**Convention for mobile robots:**
- **X-axis**: Forward
- **Y-axis**: Left
- **Z-axis**: Up

**Convention for cameras (different!):**
- **Z-axis**: Forward (depth)
- **X-axis**: Right
- **Y-axis**: Down

## 3. Broadcasting Transforms

### Static Transforms

**Static transforms** never change - they represent fixed relationships (e.g., camera mounted on robot chassis).

```python
import rclpy
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math

class StaticFramePublisher(Node):
    def __init__(self):
        super().__init__('static_frame_publisher')

        # Create static broadcaster
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)

        # Publish camera transform
        self.publish_camera_transform()

    def publish_camera_transform(self):
        """Publish static transform from base_link to camera_link"""
        static_transform = TransformStamped()

        # Header
        static_transform.header.stamp = self.get_clock().now().to_msg()
        static_transform.header.frame_id = 'base_link'
        static_transform.child_frame_id = 'camera_link'

        # Translation: camera is 0.3m forward, 0.05m up from base
        static_transform.transform.translation.x = 0.3
        static_transform.transform.translation.y = 0.0
        static_transform.transform.translation.z = 0.05

        # Rotation: camera tilted down 15 degrees
        # Convert Euler (roll, pitch, yaw) to quaternion
        roll = 0.0
        pitch = math.radians(-15)  # Tilt down
        yaw = 0.0

        quat = self.euler_to_quaternion(roll, pitch, yaw)
        static_transform.transform.rotation.x = quat[0]
        static_transform.transform.rotation.y = quat[1]
        static_transform.transform.rotation.z = quat[2]
        static_transform.transform.rotation.w = quat[3]

        # Send transform
        self.tf_static_broadcaster.sendTransform(static_transform)
        self.get_logger().info('Published static transform: base_link → camera_link')

    def euler_to_quaternion(self, roll, pitch, yaw):
        """Convert Euler angles to quaternion"""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        qw = cr * cp * cy + sr * sp * sy

        return [qx, qy, qz, qw]

def main():
    rclpy.init()
    node = StaticFramePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

**Key points:**
- Static transforms are published **once** on startup
- Use `StaticTransformBroadcaster` not `TransformBroadcaster`
- Always use current timestamp from `self.get_clock().now()`

### Dynamic Transforms

**Dynamic transforms** change over time - they represent moving relationships (e.g., robot moving in the world).

```python
from tf2_ros import TransformBroadcaster
from nav_msgs.msg import Odometry

class OdometryToTF(Node):
    def __init__(self):
        super().__init__('odom_to_tf')

        # Create dynamic broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscribe to odometry
        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
        )

    def odom_callback(self, msg):
        """Convert odometry message to TF transform"""
        t = TransformStamped()

        # Header - use timestamp from odometry message
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        # Copy translation from odometry
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z

        # Copy rotation from odometry
        t.transform.rotation = msg.pose.pose.orientation

        # Broadcast transform
        self.tf_broadcaster.sendTransform(t)
```

**Key points:**
- Dynamic transforms are published **continuously** (every update)
- Use `TransformBroadcaster` not `StaticTransformBroadcaster`
- Use timestamps from sensor messages for time synchronization

## 4. Listening to Transforms

### Transform Lookup

To **use** transforms (convert points between frames), you need a listener:

```python
import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer, LookupException
from geometry_msgs.msg import PointStamped
import tf2_geometry_msgs

class FrameListener(Node):
    def __init__(self):
        super().__init__('frame_listener')

        # Create buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Create timer to periodically look up transforms
        self.timer = self.create_timer(1.0, self.lookup_transform)

    def lookup_transform(self):
        """Look up transform from camera to base"""
        try:
            # Get transform from camera_link to base_link
            # at the current time
            transform = self.tf_buffer.lookup_transform(
                'base_link',      # Target frame
                'camera_link',    # Source frame
                rclpy.time.Time()  # Time (Time() = latest available)
            )

            # Extract translation
            trans = transform.transform.translation
            self.get_logger().info(
                f'Camera is at ({trans.x:.2f}, {trans.y:.2f}, {trans.z:.2f}) '
                f'relative to base_link'
            )

        except LookupException as e:
            self.get_logger().error(f'Transform lookup failed: {e}')
```

### Transforming Points

```python
def transform_point_example(self):
    """Transform a point from camera frame to map frame"""
    # Create point in camera frame
    point_in_camera = PointStamped()
    point_in_camera.header.frame_id = 'camera_link'
    point_in_camera.header.stamp = self.get_clock().now().to_msg()
    point_in_camera.point.x = 1.0  # 1 meter in front of camera
    point_in_camera.point.y = 0.0
    point_in_camera.point.z = 0.0

    try:
        # Transform to map frame
        point_in_map = self.tf_buffer.transform(
            point_in_camera,
            'map',  # Target frame
            timeout=rclpy.duration.Duration(seconds=1.0)
        )

        self.get_logger().info(
            f'Point in map frame: '
            f'({point_in_map.point.x:.2f}, {point_in_map.point.y:.2f})'
        )

        return point_in_map

    except Exception as e:
        self.get_logger().error(f'Transform failed: {e}')
        return None
```

### Time Travel: Looking Up Past Transforms

```python
# Get transform as it was 2 seconds ago
past_time = self.get_clock().now() - rclpy.duration.Duration(seconds=2.0)

transform = self.tf_buffer.lookup_transform(
    'map',
    'base_link',
    past_time.to_msg()
)
```

**Use cases for historical transforms:**
- Synchronize sensor data with different timestamps
- Compute velocities from position changes
- Replay logged data

## 5. Command-Line Tools

### View Transform Tree

```bash
# Install tools (if not already installed)
sudo apt install ros-humble-tf2-tools

# View the transform tree
ros2 run tf2_tools view_frames

# This creates frames.pdf showing the tree structure
```

### Echo Transforms in Real-Time

```bash
# Print transform from map to base_link
ros2 run tf2_ros tf2_echo map base_link

# Output shows:
# - Translation (x, y, z)
# - Rotation (quaternion and Euler)
# - Update rate
```

### Monitor Transform Issues

```bash
# Check for problems in the transform tree
ros2 run tf2_ros tf2_monitor

# Shows:
# - Frame rates
# - Average delays
# - Missing frames
```

## 6. Debugging TF2 Issues

### Common Problems

**Problem 1: "Frame does not exist"**
```
Lookup would require extrapolation into the future.
```

**Causes:**
- Frame not being published
- Timestamp in the future
- TF buffer not initialized

**Solutions:**
```python
# Check if transform exists
if self.tf_buffer.can_transform('map', 'base_link', rclpy.time.Time()):
    transform = self.tf_buffer.lookup_transform(...)
else:
    self.get_logger().warn('Transform not available yet')
```

**Problem 2: Multiple parents**
```
TF_REPEATED_DATA: Frame [base_link] has multiple parents!
```

**Cause:** Two nodes are publishing the same transform

**Solution:** Ensure each transform has exactly one broadcaster

**Problem 3: Timestamp errors**
```
Lookup would require extrapolation into the past.
```

**Cause:** Requesting transform at time before it was published

**Solution:** Use latest available transform:
```python
transform = self.tf_buffer.lookup_transform(
    'map', 'base_link',
    rclpy.time.Time(),  # Latest available, not specific time
    timeout=rclpy.duration.Duration(seconds=1.0)
)
```

## 7. Practical Example: Object Detection in Map Frame

Let's build a complete example: A robot detects an object with its camera and publishes its location in the map frame.

```python
import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker

class ObjectDetectorTF(Node):
    def __init__(self):
        super().__init__('object_detector_tf')

        # TF listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publisher for detected object in map frame
        self.marker_pub = self.create_publisher(Marker, 'detected_object', 10)

        # Timer to simulate object detection
        self.timer = self.create_timer(2.0, self.detect_object)

    def detect_object(self):
        """Simulate detecting an object 2m in front of camera"""
        # Object detected at (2, 0, 0) in camera frame
        object_camera = PointStamped()
        object_camera.header.frame_id = 'camera_link'
        object_camera.header.stamp = self.get_clock().now().to_msg()
        object_camera.point.x = 2.0  # 2 meters forward
        object_camera.point.y = 0.0
        object_camera.point.z = 0.0

        try:
            # Transform to map frame
            object_map = self.tf_buffer.transform(
                object_camera,
                'map',
                timeout=rclpy.duration.Duration(seconds=1.0)
            )

            self.get_logger().info(
                f'Object detected in map frame: '
                f'({object_map.point.x:.2f}, {object_map.point.y:.2f})'
            )

            # Publish marker for visualization in RViz
            self.publish_marker(object_map)

        except Exception as e:
            self.get_logger().error(f'Could not transform object: {e}')

    def publish_marker(self, point):
        """Publish visualization marker at detected object location"""
        marker = Marker()
        marker.header = point.header
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        # Position
        marker.pose.position = point.point
        marker.pose.orientation.w = 1.0

        # Size and color
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        self.marker_pub.publish(marker)

def main():
    rclpy.init()
    node = ObjectDetectorTF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

## Summary

Key takeaways from this chapter:

- **TF2** manages coordinate frame relationships in a tree structure
- **Static transforms** (fixed mounts) use `StaticTransformBroadcaster`
- **Dynamic transforms** (moving parts) use `TransformBroadcaster`
- **Transform lookup** converts data between frames using `Buffer` and `TransformListener`
- Standard frames: `map` (global), `odom` (local), `base_link` (robot center)
- Each frame has **one parent** and can have **multiple children**
- Always use timestamps from sensor messages for proper synchronization
- Command-line tools (`view_frames`, `tf2_echo`, `tf2_monitor`) help debug

**Best practices:**
- Publish static transforms once on startup
- Always timestamp dynamic transforms with sensor data time
- Use `rclpy.time.Time()` for latest available transform
- Check `can_transform()` before `lookup_transform()` to avoid errors
- Keep transform tree simple - avoid deeply nested frames

## Review Questions

1. What is the difference between `StaticTransformBroadcaster` and `TransformBroadcaster`?
2. Why does each frame have exactly one parent in the transform tree?
3. When would you use `rclpy.time.Time()` vs a specific timestamp in transform lookup?
4. How do you transform a point from `camera_link` to `map` frame?
5. What causes the "Frame has multiple parents" error?
6. Why are timestamps important in TF2?
7. What is the standard convention for mobile robot coordinate axes?

## Hands-on Exercises

### Exercise 1: Static Transform Publisher
Create a node that publishes a static transform from `base_link` to `lidar_link`:
- LiDAR mounted 0.2m forward, 0.3m up
- No rotation
- Verify with `ros2 run tf2_ros tf2_echo base_link lidar_link`

### Exercise 2: Dynamic Broadcaster
Create a node that broadcasts `odom → base_link` transform:
- Move robot in a circle (radius 1m)
- Update position at 10 Hz
- Visualize in RViz

### Exercise 3: Point Transformer
Create a node that:
- Subscribes to `/clicked_point` (from RViz)
- Transforms point from `map` frame to `base_link` frame
- Publishes result as a marker
- Test by clicking points in RViz

## Further Reading

- [TF2 Official Documentation](https://docs.ros.org/en/humble/Concepts/About-Tf2.html)
- [TF2 Python Tutorials](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Introduction-To-Tf2.html)
- [REP 105: Coordinate Frames for Mobile Platforms](https://www.ros.org/reps/rep-0105.html)
- [Quaternion Math Explained](https://eater.net/quaternions)

---

**Next Chapter**: [URDF Robot Modeling →](ch05-urdf-models.md)

**Previous Chapter**: [← Python Programming with rclpy](ch03-python-rclpy.md)
