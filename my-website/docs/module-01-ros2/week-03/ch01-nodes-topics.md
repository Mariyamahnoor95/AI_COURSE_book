---
id: ch01-nodes-topics
title: ROS 2 Nodes and Topics
sidebar_label: Nodes and Topics
sidebar_position: 1
---

# ROS 2 Nodes and Topics

## Learning Objectives

By the end of this chapter, you will be able to:

- Understand the architecture of ROS 2 nodes and their role in robotic systems
- Create publisher and subscriber nodes using `rclpy` (ROS 2 Python library)
- Work with ROS 2 topics for asynchronous message passing
- Use standard ROS 2 message types for sensor data and robot control
- Debug and introspect running ROS 2 systems using command-line tools

## Introduction to ROS 2 Architecture

ROS 2 (Robot Operating System 2) is a flexible framework for writing robot software. It's a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robotic platforms.

### The Compute Graph

At the heart of ROS 2 is the **compute graph**—a peer-to-peer network of ROS 2 processes (nodes) that are collectively processing data. The compute graph concept enables:

- **Modularity**: Breaking complex systems into manageable, reusable components
- **Distributed computing**: Nodes can run on different machines in a network
- **Language agnosticism**: Nodes can be written in C++, Python, or other supported languages
- **Loose coupling**: Nodes communicate through well-defined interfaces without direct dependencies

## ROS 2 Nodes

A **node** is a fundamental ROS 2 entity—a single, modular process that performs a specific computation or task.

### Node Characteristics

- **Single responsibility**: Each node should do one thing well (e.g., read sensor data, plan paths, control motors)
- **Independent execution**: Nodes run as separate operating system processes
- **Communication**: Nodes interact through topics, services, actions, and parameters
- **Discoverability**: Nodes announce their presence to the ROS 2 network automatically

### Node Examples in a Mobile Robot

Consider a mobile robot navigating a warehouse. It might have these nodes:

- `camera_driver`: Publishes images from an RGB camera
- `lidar_driver`: Publishes laser scan data
- `localization`: Estimates robot position using sensor data
- `path_planner`: Computes collision-free paths to goals
- `controller`: Sends velocity commands to motor drivers
- `safety_monitor`: Emergency stop if obstacles are too close

Each node operates independently but coordinates through message passing.

## Creating Your First Node

Let's create a simple node that publishes a "Hello, ROS 2!" message.

### Step 1: Import Required Libraries

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
```

- `rclpy`: The ROS 2 Python client library
- `Node`: Base class for all ROS 2 nodes
- `String`: A standard message type for text data

### Step 2: Define the Node Class

```python
class HelloPublisher(Node):
    def __init__(self):
        super().__init__('hello_publisher')

        # Create a publisher on the 'hello_topic' topic
        self.publisher = self.create_publisher(String, 'hello_topic', 10)

        # Create a timer that calls the callback every 1 second
        self.timer = self.create_timer(1.0, self.timer_callback)

        self.count = 0
        self.get_logger().info('Hello Publisher node initialized')

    def timer_callback(self):
        # Create and publish a message
        msg = String()
        msg.data = f'Hello, ROS 2! Message #{self.count}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Published: "{msg.data}"')
        self.count += 1
```

**Key Components:**

- `super().__init__('hello_publisher')`: Initializes the node with name "hello_publisher"
- `create_publisher()`: Creates a publisher for the `String` message type on topic "hello_topic"
  - `10` is the **queue size**—how many messages to buffer if subscribers can't keep up
- `create_timer()`: Schedules the callback function to run every 1.0 seconds
- `get_logger()`: Accesses the node's logging system for status messages

### Step 3: Main Function

```python
def main(args=None):
    # Initialize the ROS 2 Python client library
    rclpy.init(args=args)

    # Create an instance of the node
    node = HelloPublisher()

    # Spin the node so callbacks are called
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean shutdown
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

- `rclpy.init()`: Initializes the ROS 2 context (must be called before creating nodes)
- `rclpy.spin()`: Keeps the node alive and processes callbacks
- `destroy_node()` and `shutdown()`: Clean up resources when exiting

### Running the Node

Save the code as `hello_publisher.py` and run:

```bash
python3 hello_publisher.py
```

Output:
```
[INFO] [hello_publisher]: Hello Publisher node initialized
[INFO] [hello_publisher]: Published: "Hello, ROS 2! Message #0"
[INFO] [hello_publisher]: Published: "Hello, ROS 2! Message #1"
[INFO] [hello_publisher]: Published: "Hello, ROS 2! Message #2"
...
```

## ROS 2 Topics

**Topics** are named buses over which nodes exchange messages. Topics implement a **publish-subscribe** pattern:

- **Publishers** send messages to topics (one-to-many)
- **Subscribers** receive messages from topics
- Communication is **asynchronous**—publishers don't wait for subscribers

### Topic Properties

- **Typed**: Each topic has a message type (e.g., `sensor_msgs/LaserScan`, `geometry_msgs/Twist`)
- **Anonymous**: Publishers and subscribers don't know about each other
- **Buffered**: Recent messages are queued (queue depth configurable)
- **Many-to-many**: Multiple publishers and subscribers can use the same topic

### Topic Naming Conventions

Topics follow a hierarchical naming scheme:

- `/camera/image_raw`: Raw image from camera
- `/robot1/cmd_vel`: Velocity commands for robot1
- `/scan`: Laser scan data (relative to node's namespace)

**Best Practices:**

- Use descriptive names: `/front_camera/rgb` instead of `/cam1`
- Group related topics under namespaces: `/sensors/lidar`, `/sensors/imu`
- Follow ROS conventions: `cmd_vel` for velocity, `odom` for odometry

## Creating a Subscriber Node

Now let's create a node that subscribes to the "hello_topic" and prints received messages.

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class HelloSubscriber(Node):
    def __init__(self):
        super().__init__('hello_subscriber')

        # Create a subscription to 'hello_topic'
        self.subscription = self.create_subscription(
            String,
            'hello_topic',
            self.listener_callback,
            10
        )

        self.get_logger().info('Hello Subscriber node initialized')

    def listener_callback(self, msg):
        # This function is called whenever a message is received
        self.get_logger().info(f'Received: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = HelloSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Key Differences from Publisher:**

- `create_subscription()` instead of `create_publisher()`
- `listener_callback()`: Called automatically when messages arrive
- No timer needed—callbacks are event-driven

### Running Publisher and Subscriber Together

Terminal 1:
```bash
python3 hello_publisher.py
```

Terminal 2:
```bash
python3 hello_subscriber.py
```

Output (Terminal 2):
```
[INFO] [hello_subscriber]: Hello Subscriber node initialized
[INFO] [hello_subscriber]: Received: "Hello, ROS 2! Message #5"
[INFO] [hello_subscriber]: Received: "Hello, ROS 2! Message #6"
...
```

## Standard ROS 2 Message Types

ROS 2 provides a rich set of standard message types for common robotics tasks:

### Primitive Types (std_msgs)

```python
from std_msgs.msg import String, Int32, Float64, Bool
```

- `String`: Text data
- `Int32`, `Float64`: Numeric values
- `Bool`: True/false flags

### Geometry Messages (geometry_msgs)

Used for positions, velocities, and poses:

```python
from geometry_msgs.msg import Twist, Point, Pose, PoseStamped

# Example: Velocity command for a mobile robot
twist = Twist()
twist.linear.x = 0.5   # Forward velocity (m/s)
twist.angular.z = 0.2  # Rotational velocity (rad/s)
```

### Sensor Messages (sensor_msgs)

```python
from sensor_msgs.msg import LaserScan, Image, Imu, JointState

# LaserScan: 2D/3D LiDAR data
# Image: Camera images (RGB, depth, etc.)
# Imu: Inertial measurement unit (acceleration, gyroscope)
# JointState: Robot joint positions and velocities
```

### Navigation Messages (nav_msgs)

```python
from nav_msgs.msg import Odometry, Path

# Odometry: Robot position, orientation, and velocities
# Path: Sequence of poses defining a trajectory
```

## Practical Example: Velocity Publisher for a Mobile Robot

Let's create a node that publishes velocity commands to drive a robot in a square pattern.

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math

class SquareDriveNode(Node):
    def __init__(self):
        super().__init__('square_drive')

        # Publisher for velocity commands
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # State machine for square pattern
        self.state = 'forward'  # States: 'forward', 'turn'
        self.distance_traveled = 0.0
        self.angle_turned = 0.0

        # Parameters
        self.side_length = 2.0  # meters
        self.linear_speed = 0.3  # m/s
        self.angular_speed = 0.5  # rad/s

        # Timer for control loop (10 Hz)
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('Square Drive node initialized')

    def control_loop(self):
        twist = Twist()

        if self.state == 'forward':
            # Drive forward
            twist.linear.x = self.linear_speed
            twist.angular.z = 0.0

            self.distance_traveled += self.linear_speed * 0.1  # dt = 0.1s

            if self.distance_traveled >= self.side_length:
                self.state = 'turn'
                self.distance_traveled = 0.0
                self.get_logger().info('Switching to turn')

        elif self.state == 'turn':
            # Turn 90 degrees
            twist.linear.x = 0.0
            twist.angular.z = self.angular_speed

            self.angle_turned += self.angular_speed * 0.1

            if self.angle_turned >= math.pi / 2:  # 90 degrees
                self.state = 'forward'
                self.angle_turned = 0.0
                self.get_logger().info('Switching to forward')

        self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = SquareDriveNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Stop the robot on exit
        stop_msg = Twist()
        node.publisher.publish(stop_msg)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Key Concepts Demonstrated:**

- **State machine**: Alternating between forward and turning states
- **Dead reckoning**: Estimating distance/angle from velocity commands (not accurate in practice—odometry is needed)
- **Control frequency**: 10 Hz control loop is typical for mobile robots
- **Safety**: Sending zero velocity on shutdown

## ROS 2 Command-Line Tools

ROS 2 provides powerful CLI tools for introspecting and debugging running systems.

### Listing Active Nodes

```bash
ros2 node list
```

Output:
```
/hello_publisher
/hello_subscriber
/square_drive
```

### Node Information

```bash
ros2 node info /hello_publisher
```

Output:
```
/hello_publisher
  Subscribers:
  Publishers:
    /hello_topic: std_msgs/msg/String
  Service Servers:
  Service Clients:
  Action Servers:
  Action Clients:
```

### Listing Topics

```bash
ros2 topic list
```

Output:
```
/hello_topic
/cmd_vel
/parameter_events
/rosout
```

### Topic Information

```bash
ros2 topic info /hello_topic
```

Output:
```
Type: std_msgs/msg/String
Publisher count: 1
Subscription count: 1
```

### Echoing Topic Data

View messages in real-time:

```bash
ros2 topic echo /hello_topic
```

Output:
```
data: 'Hello, ROS 2! Message #10'
---
data: 'Hello, ROS 2! Message #11'
---
...
```

### Publishing from Command Line

Manually publish a message:

```bash
ros2 topic pub /hello_topic std_msgs/msg/String "data: 'Manual message'"
```

### Measuring Topic Frequency

Check how fast messages are being published:

```bash
ros2 topic hz /cmd_vel
```

Output:
```
average rate: 10.003
    min: 0.099s max: 0.101s std dev: 0.00050s window: 100
```

## Quality of Service (QoS) in ROS 2

ROS 2 introduces **QoS policies** that control message delivery characteristics. This is a major improvement over ROS 1.

### Key QoS Parameters

- **Reliability**:
  - `RELIABLE`: Guarantee delivery (like TCP)
  - `BEST_EFFORT`: No guarantees (like UDP, lower latency)

- **Durability**:
  - `VOLATILE`: Only send to active subscribers
  - `TRANSIENT_LOCAL`: Late-joining subscribers receive recent messages

- **History**:
  - `KEEP_LAST(n)`: Buffer last n messages
  - `KEEP_ALL`: Buffer all messages (limited by system resources)

### Example: Reliable Communication

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# Define QoS profile for reliable, ordered delivery
qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    depth=10
)

self.publisher = self.create_publisher(
    Twist,
    'cmd_vel',
    qos_profile
)
```

**Use Cases:**

- **Sensor data**: Often uses `BEST_EFFORT` for low latency
- **Commands**: Use `RELIABLE` to ensure delivery
- **Configuration**: Use `TRANSIENT_LOCAL` so late joiners get current config

## Best Practices for Nodes and Topics

### Node Design

1. **Single Responsibility**: One node, one job
2. **Configurability**: Use ROS parameters for tunable values
3. **Error Handling**: Check return values, handle exceptions
4. **Logging**: Use `get_logger()` instead of `print()`
5. **Graceful Shutdown**: Clean up resources in `finally` blocks

### Topic Design

1. **Naming**: Descriptive, hierarchical names
2. **Message Types**: Use standard types when possible
3. **Frequency**: Match publishing rate to consumer needs
4. **QoS**: Choose appropriate reliability and durability
5. **Bandwidth**: Compress large messages (images, point clouds)

### Performance Considerations

- **Avoid busy loops**: Use timers or callbacks, not `while True` with `sleep()`
- **Minimize allocations**: Reuse message objects in high-frequency publishers
- **Batch processing**: Process multiple messages together when latency allows
- **Zero-copy**: Use intra-process communication when nodes run in the same process

## Summary

In this chapter, we explored the fundamental building blocks of ROS 2:

- **Nodes**: Modular processes that perform specific tasks
- **Topics**: Publish-subscribe message passing for asynchronous communication
- **Message types**: Standard interfaces for sensor data, commands, and more
- **Command-line tools**: Essential for debugging and introspection
- **QoS policies**: Fine-grained control over message delivery

With these concepts, you can build distributed robotic systems where independent nodes coordinate through message passing. In the next chapter, we'll explore **services and actions**—synchronous request-response communication and long-running tasks.

## Review Questions

1. What is the difference between a node and a topic in ROS 2?
2. Explain the publish-subscribe pattern. How does it differ from a request-response pattern?
3. What are QoS policies, and when would you use `RELIABLE` vs. `BEST_EFFORT`?
4. Write pseudo-code for a subscriber that counts the number of messages received on a topic.
5. How would you debug a scenario where a publisher is running but a subscriber isn't receiving messages?

## Hands-On Exercises

1. **Temperature Monitor**: Create a publisher that publishes random temperature values (15-30°C) every second, and a subscriber that prints a warning if temperature >25°C.

2. **Multi-Publisher**: Create two publishers sending messages to the same topic from different nodes. Verify that a single subscriber receives messages from both.

3. **Parameterized Node**: Modify the `SquareDriveNode` to accept `side_length` and `linear_speed` as ROS parameters instead of hardcoded values.

4. **CLI Practice**: Use `ros2 topic` commands to:
   - List all active topics
   - Echo messages from `/cmd_vel`
   - Manually publish a velocity command
   - Measure the publishing frequency

Complete code for these exercises is available in `/static/code/ros2/chapter01/`.

---

**Next Chapter**: [Services and Actions](./ch02-services-actions.md)
