---
id: ch03-python-rclpy
title: Python Programming with rclpy
sidebar_label: Python with rclpy
sidebar_position: 1
---

# Python Programming with rclpy

## Learning Objectives

By the end of this chapter, you will be able to:

- Understand the rclpy API structure and core concepts
- Create custom ROS 2 nodes using object-oriented Python
- Implement timers, callbacks, and lifecycle management
- Handle parameters for runtime configuration
- Apply message handling patterns for robust communication
- Debug and test Python ROS 2 nodes effectively

## Introduction

While we've used `rclpy` in previous chapters, this chapter dives deeper into Python programming patterns for ROS 2. **rclpy** is the official Python client library for ROS 2, providing a Pythonic interface to ROS 2's core functionality.

**Why Python for robotics?**
- ✅ Rapid prototyping and iteration
- ✅ Rich ecosystem (NumPy, OpenCV, TensorFlow)
- ✅ Easy integration with AI/ML libraries
- ✅ Readable code for algorithm development
- ✅ Excellent for high-level logic and coordination

**When to use Python vs C++:**
- **Python**: High-level planning, vision processing, machine learning, prototyping
- **C++**: Real-time control, driver implementations, performance-critical code

This chapter teaches professional Python patterns for building maintainable, robust ROS 2 applications.

## 1. rclpy API Overview

### Core Components

```python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.parameter import Parameter
from rclpy.executors import MultiThreadedExecutor
```

**Key classes:**
- `Node`: Base class for all ROS 2 nodes
- `Publisher`: Publishes messages to topics
- `Subscription`: Receives messages from topics
- `Client`: Calls services
- `Service`: Provides service functionality
- `ActionClient/ActionServer`: For long-running tasks
- `Timer`: Executes callbacks periodically
- `Parameter`: Runtime configuration

### Node Lifecycle

```python
# 1. Initialize ROS 2
rclpy.init(args=args)

# 2. Create node
node = MyNode()

# 3. Spin (process callbacks)
rclpy.spin(node)

# 4. Cleanup
node.destroy_node()
rclpy.shutdown()
```

**Best practice**: Always use try/finally for cleanup:

```python
def main():
    rclpy.init()
    node = MyNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

## 2. Creating Custom Nodes

### Basic Node Structure

```python
from rclpy.node import Node

class MyRobotNode(Node):
    def __init__(self):
        # ALWAYS call super().__init__ first
        super().__init__('my_robot_node')

        # Declare parameters
        self.declare_parameter('update_rate', 10.0)

        # Create publishers/subscribers
        self.pub = self.create_publisher(String, 'status', 10)
        self.sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_callback, 10
        )

        # Create timer
        rate = self.get_parameter('update_rate').value
        self.timer = self.create_timer(1.0/rate, self.timer_callback)

        # Initialize state
        self.counter = 0

        self.get_logger().info('Node initialized')

    def cmd_callback(self, msg):
        """Handle incoming velocity commands"""
        self.get_logger().info(f'Received: {msg.linear.x}')

    def timer_callback(self):
        """Periodic update loop"""
        self.counter += 1
        msg = String()
        msg.data = f'Update {self.counter}'
        self.pub.publish(msg)
```

### Node Naming Best Practices

```python
# Good: Descriptive, lowercase with underscores
super().__init__('camera_processor')
super().__init__('robot_state_publisher')
super().__init__('obstacle_detector')

# Bad: Too generic or unclear
super().__init__('node1')
super().__init__('MyNode')
super().__init__('test')
```

### Multiple Publishers/Subscribers

```python
class SensorFusionNode(Node):
    def __init__(self):
        super().__init__('sensor_fusion')

        # Multiple subscribers
        self.lidar_sub = self.create_subscription(
            LaserScan, 'scan', self.lidar_callback, 10
        )
        self.imu_sub = self.create_subscription(
            Imu, 'imu', self.imu_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10
        )

        # Multiple publishers
        self.pose_pub = self.create_publisher(
            PoseStamped, 'fused_pose', 10
        )
        self.status_pub = self.create_publisher(
            String, 'fusion_status', 10
        )

        # Store latest data
        self.latest_scan = None
        self.latest_imu = None
        self.latest_odom = None

    def lidar_callback(self, msg):
        self.latest_scan = msg
        self.fuse_sensors()

    def imu_callback(self, msg):
        self.latest_imu = msg

    def odom_callback(self, msg):
        self.latest_odom = msg

    def fuse_sensors(self):
        """Combine sensor data"""
        if all([self.latest_scan, self.latest_imu, self.latest_odom]):
            # Perform fusion
            fused_pose = self.compute_fused_pose()
            self.pose_pub.publish(fused_pose)
```

## 3. Timers and Periodic Execution

### Creating Timers

```python
class PeriodicNode(Node):
    def __init__(self):
        super().__init__('periodic_node')

        # Timer that fires every 0.1 seconds (10 Hz)
        self.timer = self.create_timer(0.1, self.timer_callback)

        # You can have multiple timers
        self.slow_timer = self.create_timer(1.0, self.slow_callback)
        self.fast_timer = self.create_timer(0.01, self.fast_callback)

    def timer_callback(self):
        """Runs at 10 Hz"""
        pass

    def slow_callback(self):
        """Runs at 1 Hz"""
        pass

    def fast_callback(self):
        """Runs at 100 Hz"""
        pass
```

### Dynamic Timer Control

```python
class AdaptiveNode(Node):
    def __init__(self):
        super().__init__('adaptive_node')
        self.timer = self.create_timer(1.0, self.callback)

    def speed_up(self):
        """Increase update rate"""
        self.timer.cancel()
        self.timer = self.create_timer(0.1, self.callback)  # 10x faster

    def slow_down(self):
        """Decrease update rate"""
        self.timer.cancel()
        self.timer = self.create_timer(2.0, self.callback)  # 2x slower

    def pause(self):
        """Stop timer"""
        self.timer.cancel()

    def callback(self):
        pass
```

## 4. Parameters

### Declaring and Using Parameters

```python
class ConfigurableNode(Node):
    def __init__(self):
        super().__init__('configurable_node')

        # Declare parameters with defaults
        self.declare_parameter('robot_name', 'robot_1')
        self.declare_parameter('max_speed', 1.0)
        self.declare_parameter('enable_safety', True)
        self.declare_parameter('control_gains', [1.0, 0.1, 0.05])

        # Get parameter values
        self.robot_name = self.get_parameter('robot_name').value
        self.max_speed = self.get_parameter('max_speed').value
        self.safety_enabled = self.get_parameter('enable_safety').value
        self.gains = self.get_parameter('control_gains').value

        self.get_logger().info(f'Robot: {self.robot_name}')
        self.get_logger().info(f'Max speed: {self.max_speed} m/s')
```

### Setting Parameters at Launch

```bash
# Command line
ros2 run my_package my_node --ros-args \
  -p robot_name:=robot_2 \
  -p max_speed:=2.5 \
  -p enable_safety:=false
```

### Dynamic Parameter Updates

```python
class DynamicNode(Node):
    def __init__(self):
        super().__init__('dynamic_node')

        self.declare_parameter('threshold', 0.5)

        # Register callback for parameter changes
        self.add_on_set_parameters_callback(self.parameter_callback)

    def parameter_callback(self, params):
        """Called when parameters change"""
        for param in params:
            if param.name == 'threshold':
                if param.value > 0.0 and param.value < 1.0:
                    self.get_logger().info(f'Threshold updated: {param.value}')
                    return SetParametersResult(successful=True)
                else:
                    self.get_logger().error('Threshold must be 0-1')
                    return SetParametersResult(successful=False)

        return SetParametersResult(successful=True)
```

## 5. Message Handling Patterns

### Pattern 1: Synchronous Processing

```python
def callback(self, msg):
    """Process message immediately"""
    result = self.process_data(msg)
    self.publisher.publish(result)
```

**Use when:**
- Processing is fast (&lt;10ms)
- No blocking operations
- Order matters

### Pattern 2: Asynchronous with Queue

```python
from queue import Queue
from threading import Thread

class AsyncNode(Node):
    def __init__(self):
        super().__init__('async_node')

        self.queue = Queue(maxsize=100)

        # Subscriber puts messages in queue
        self.sub = self.create_subscription(
            Image, 'camera', self.image_callback, 10
        )

        # Worker thread processes queue
        self.worker = Thread(target=self.process_queue, daemon=True)
        self.worker.start()

    def image_callback(self, msg):
        """Non-blocking: just queue the message"""
        if not self.queue.full():
            self.queue.put(msg)
        else:
            self.get_logger().warn('Queue full, dropping message')

    def process_queue(self):
        """Worker thread processes messages"""
        while rclpy.ok():
            msg = self.queue.get()
            # Time-consuming processing
            result = self.heavy_computation(msg)
            self.publish_result(result)
```

### Pattern 3: Latest Message Only

```python
class LatestMessageNode(Node):
    def __init__(self):
        super().__init__('latest_message_node')

        self.latest_msg = None

        self.sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10
        )

        # Process latest message periodically
        self.timer = self.create_timer(0.1, self.process_latest)

    def scan_callback(self, msg):
        """Just store the latest message"""
        self.latest_msg = msg

    def process_latest(self):
        """Process most recent scan"""
        if self.latest_msg is not None:
            self.process_scan(self.latest_msg)
```

### Pattern 4: Message Synchronization

```python
from message_filters import Subscriber, ApproximateTimeSynchronizer

class SyncNode(Node):
    def __init__(self):
        super().__init__('sync_node')

        # Create synchronized subscribers
        self.image_sub = Subscriber(self, Image, 'camera/image')
        self.depth_sub = Subscriber(self, Image, 'camera/depth')

        # Synchronize messages within 0.1 seconds
        self.sync = ApproximateTimeSynchronizer(
            [self.image_sub, self.depth_sub],
            queue_size=10,
            slop=0.1
        )
        self.sync.registerCallback(self.sync_callback)

    def sync_callback(self, image_msg, depth_msg):
        """Called when messages are synchronized"""
        # Process synchronized RGB and depth
        pass
```

## 6. Error Handling and Logging

### Logging Levels

```python
class LoggingNode(Node):
    def __init__(self):
        super().__init__('logging_node')

        # Different severity levels
        self.get_logger().debug('Detailed debugging info')
        self.get_logger().info('Normal operation info')
        self.get_logger().warn('Warning: potential issue')
        self.get_logger().error('Error occurred')
        self.get_logger().fatal('Critical failure')
```

### Exception Handling

```python
def safe_callback(self, msg):
    """Callback with error handling"""
    try:
        result = self.process_message(msg)
        self.publisher.publish(result)

    except ValueError as e:
        self.get_logger().error(f'Invalid data: {e}')

    except Exception as e:
        self.get_logger().fatal(f'Unexpected error: {e}')
        # Optionally: self.destroy_node()
```

### Validation

```python
def validate_message(self, msg):
    """Validate incoming data"""
    if msg.linear.x > self.max_speed:
        self.get_logger().warn(
            f'Speed {msg.linear.x} exceeds max {self.max_speed}'
        )
        msg.linear.x = self.max_speed

    if math.isnan(msg.linear.x):
        self.get_logger().error('NaN detected in velocity')
        return False

    return True
```

## 7. Multi-Threading

### MultiThreadedExecutor

```python
from rclpy.executors import MultiThreadedExecutor

def main():
    rclpy.init()

    node1 = Node1()
    node2 = Node2()

    # Multi-threaded executor
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node1)
    executor.add_node(node2)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        node1.destroy_node()
        node2.destroy_node()
        rclpy.shutdown()
```

## 8. Testing

### Unit Testing with unittest

```python
import unittest
import rclpy
from my_package.my_node import MyNode

class TestMyNode(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_node_creation(self):
        """Test node initializes correctly"""
        node = MyNode()
        self.assertIsNotNone(node)
        node.destroy_node()

    def test_parameter_default(self):
        """Test default parameter value"""
        node = MyNode()
        param = node.get_parameter('max_speed').value
        self.assertEqual(param, 1.0)
        node.destroy_node()
```

## Summary

Key takeaways:

- **rclpy** provides Pythonic interface to ROS 2
- **Object-oriented design** with Node base class
- **Timers** enable periodic execution
- **Parameters** allow runtime configuration
- **Multiple patterns** for message handling (sync, async, latest-only)
- **Proper error handling** and logging are essential
- **Multi-threading** improves performance for concurrent tasks

**Best practices:**
- Always use try/finally for cleanup
- Validate inputs in callbacks
- Log appropriately (not too verbose, not too quiet)
- Test your nodes with unit tests
- Use parameters for configurable values

## Review Questions

1. What is the difference between a timer callback and a subscription callback?
2. When should you use an asynchronous message processing pattern?
3. How do you change a parameter value after a node has started?
4. Why should you always call `super().__init__()` first in `__init__()`?
5. What happens if you don't call `destroy_node()` and `shutdown()`?

## Hands-on Exercises

### Exercise 1: Configurable Timer Node
Create a node with:
- Parameter for update rate (Hz)
- Timer that publishes a counter
- Ability to change rate without restarting

### Exercise 2: Sensor Validator
Create a node that:
- Subscribes to sensor data
- Validates ranges (min/max)
- Publishes only valid data
- Logs warnings for invalid data

### Exercise 3: Multi-Publisher Node
Create a node with:
- 3 different publishers (status, diagnostics, data)
- Single timer that updates all
- Parameters to enable/disable each publisher

## Further Reading

- [rclpy API Documentation](https://docs.ros2.org/latest/api/rclpy/)
- [ROS 2 Python Tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)
- [Python Style Guide for ROS](http://wiki.ros.org/PyStyleGuide)

---

**Next Chapter**: [TF2 Coordinate Transforms →](ch04-tf2-transforms.md)

**Previous Chapter**: [← ROS 2 Services and Actions](../week-03/ch02-services-actions.md)
