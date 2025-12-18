---
id: ch07-gazebo-physics
title: Gazebo Physics Simulation
sidebar_label: Gazebo Physics
sidebar_position: 1
---

# Gazebo Physics Simulation

## Learning Objectives

By the end of this chapter, you will be able to:

- Understand what Gazebo is and why physics simulation matters for robotics
- Launch and navigate the Gazebo simulation environment
- Spawn robots and objects in simulation
- Understand physics engines (ODE, Bullet, Simbody)
- Control simulated robots using ROS 2
- Recognize the sim-to-real gap and mitigation strategies

## Introduction

Before deploying a robot in the real world, we need to test our algorithms safely and repeatedly. **Gazebo** is an open-source 3D robot simulator that provides realistic physics, high-quality graphics, and seamless ROS 2 integration.

**Why simulate?**
- ✅ **Safety**: Test dangerous scenarios (falling, collisions) without hardware damage
- ✅ **Speed**: Iterate algorithms 10-100x faster than real-time
- ✅ **Cost**: Avoid expensive hardware for initial development
- ✅ **Reproducibility**: Same initial conditions every time
- ✅ **Scalability**: Test with thousands of robots or environments
- ✅ **Impossible scenarios**: Perfect sensors, instant teleportation for testing

**The digital twin concept**: Gazebo creates a virtual replica of your physical robot where you can:
- Develop and test control algorithms
- Train machine learning models
- Validate sensor processing pipelines
- Debug multi-robot coordination
- Generate synthetic training data

This chapter introduces Gazebo and demonstrates how to use it for Physical AI development.

## 1. What is Gazebo?

### Overview

Gazebo (also called Gazebo Classic or Gazebo 11) is a standalone simulator that works with ROS 2 through the `gazebo_ros` packages. The newer **Gazebo** (formerly Ignition Gazebo) is a complete rewrite with improved architecture.

**Core components:**
1. **Physics engine**: Simulates gravity, collisions, friction, forces
2. **Rendering engine**: OGRE-based 3D graphics
3. **Sensor simulation**: Camera, LiDAR, IMU, GPS, depth sensors
4. **Plugin system**: Extend functionality with custom code
5. **ROS 2 integration**: Publish/subscribe to ROS topics from simulation

### Physics Engines

Gazebo supports multiple physics backends:

| Engine | Speed | Accuracy | Best For |
|--------|-------|----------|----------|
| **ODE** | Fast | Moderate | Mobile robots, prototyping |
| **Bullet** | Fastest | Lower | Real-time, many objects |
| **Simbody** | Slowest | Highest | Precise manipulation, biomechanics |
| **DART** | Medium | High | Humanoids, complex dynamics |

**Default**: ODE (Open Dynamics Engine) balances speed and accuracy for most robotics applications.

### Simulated Sensors

Gazebo can simulate various sensors:
- **Cameras**: RGB, depth, stereo
- **LiDAR**: 2D and 3D laser scanners
- **IMU**: Accelerometer, gyroscope, magnetometer (with noise)
- **GPS**: Global positioning (configurable accuracy)
- **Contact**: Collision detection
- **Force/torque**: Joint effort sensing

**Sensor noise**: Add Gaussian noise to simulate real sensor imperfections:

```xml
<sensor name="imu" type="imu">
  <noise>
    <type>gaussian</type>
    <rate>
      <mean>0.0</mean>
      <stddev>0.001</stddev>
    </rate>
  </noise>
</sensor>
```

## 2. Installing and Launching Gazebo

### Installation (Ubuntu 22.04 + ROS 2 Humble)

Gazebo is included with `ros-humble-desktop`:

```bash
# Install if not already present
sudo apt update
sudo apt install ros-humble-gazebo-ros-pkgs

# Verify installation
gazebo --version
```

### Launching Gazebo

**Method 1: Standalone Gazebo**
```bash
gazebo
```

**Method 2: With ROS 2**
```bash
ros2 launch gazebo_ros gazebo.launch.py
```

**Method 3: With a world file**
```bash
gazebo /usr/share/gazebo-11/worlds/cafe.world
```

**Method 4: Empty world (fastest)**
```bash
ros2 launch gazebo_ros empty_world.launch.py
```

### Gazebo Interface

**Main panels:**
- **Scene**: 3D view of the simulated world
- **Insert**: Add models from library
- **World**: Configure lighting, physics, atmosphere
- **Models**: List of spawned objects
- **Time**: Real-time vs sim-time, play/pause controls

**Camera controls:**
- **Left-click + drag**: Rotate view
- **Scroll**: Zoom in/out
- **Middle-click + drag**: Pan view
- **Shift + left-click**: Rotate around object

## 3. Spawning and Controlling Robots

### Spawning a Robot

**Using TurtleBot3 (popular mobile robot):**

```bash
# Set model type (burger, waffle, waffle_pi)
export TURTLEBOT3_MODEL=burger

# Launch world with robot
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

**Programmatic spawning:**

```python
import rclpy
from gazebo_msgs.srv import SpawnEntity

def spawn_robot():
    """Spawn a robot model in Gazebo"""
    rclpy.init()
    node = rclpy.create_node('robot_spawner')

    # Create service client
    client = node.create_client(SpawnEntity, '/spawn_entity')
    client.wait_for_service()

    # Read robot description (URDF/SDF)
    with open('my_robot.urdf', 'r') as f:
        robot_xml = f.read()

    # Create request
    request = SpawnEntity.Request()
    request.name = 'my_robot'
    request.xml = robot_xml
    request.robot_namespace = 'robot1'
    request.initial_pose.position.x = 0.0
    request.initial_pose.position.y = 0.0
    request.initial_pose.position.z = 0.5

    # Spawn robot
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)

    if future.result().success:
        node.get_logger().info('Robot spawned successfully!')
    else:
        node.get_logger().error(f'Failed: {future.result().status_message}')

    node.destroy_node()
    rclpy.shutdown()
```

### Controlling the Robot

**Velocity control (differential drive robot):**

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class SimpleController(Node):
    def __init__(self):
        super().__init__('simple_controller')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.control_loop)

    def control_loop(self):
        """Send velocity commands to robot"""
        msg = Twist()

        # Drive forward at 0.2 m/s
        msg.linear.x = 0.2

        # Turn slightly left
        msg.angular.z = 0.1

        self.publisher.publish(msg)
```

**See complete example**: `/static/code/digital-twin/chapter07/simple_robot_controller.py`

### Reading Sensor Data

**LiDAR data:**

```python
from sensor_msgs.msg import LaserScan

class SensorReader(Node):
    def __init__(self):
        super().__init__('sensor_reader')
        self.sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

    def scan_callback(self, msg):
        """Process laser scan data"""
        # Get minimum distance
        min_dist = min(msg.ranges)

        # Get distance directly ahead
        front_idx = len(msg.ranges) // 2
        front_dist = msg.ranges[front_idx]

        self.get_logger().info(
            f'Min: {min_dist:.2f}m, Front: {front_dist:.2f}m'
        )
```

## 4. Physics Configuration

### World Configuration

Create a custom world file (`my_world.world`):

```xml
<?xml version="1.0"?>
<sdf version="1.6">
  <world name="default">
    <!-- Physics settings -->
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>

      <gravity>0 0 -9.81</gravity>

      <ode>
        <solver>
          <type>quick</type>
          <iters>50</iters>
        </solver>
      </ode>
    </physics>

    <!-- Lighting -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Add obstacles -->
    <include>
      <uri>model://cafe_table</uri>
      <pose>2 0 0 0 0 0</pose>
    </include>

  </world>
</sdf>
```

### Physics Parameters

**Key parameters:**
- `max_step_size`: Simulation time step (smaller = more accurate, slower)
- `real_time_factor`: Target speed (1.0 = real-time, 0.5 = half-speed)
- `gravity`: Gravitational acceleration (default: -9.81 m/s²)
- `solver iterations`: Higher = more stable contacts, slower

**Tuning for performance:**

```xml
<!-- Fast but less accurate -->
<max_step_size>0.01</max_step_size>
<real_time_update_rate>100</real_time_update_rate>
<iters>20</iters>

<!-- Slow but very accurate -->
<max_step_size>0.0001</max_step_size>
<real_time_update_rate>10000</real_time_update_rate>
<iters>100</iters>
```

## 5. Digital Twin Workflow

### Development Pipeline

**1. Design robot (URDF/SDF)**
- Define links (rigid bodies)
- Define joints (connections)
- Add sensors and actuators
- Specify mass, inertia, collision geometry

**2. Test in Gazebo**
- Spawn robot in simulated world
- Verify kinematics and dynamics
- Tune controller parameters
- Test edge cases (falling, collisions)

**3. Transfer to hardware**
- Same ROS 2 nodes work on real robot
- Adjust for hardware differences
- Fine-tune parameters based on real-world tests

**4. Iterate**
- Update simulation based on real-world observations
- Improve physics parameters
- Add noise to sensors
- Refine controller

### Sim-to-Real Transfer

**The gap**: Simulation is imperfect. Real robots experience:
- Sensor noise and delays
- Actuator lag and backlash
- Unmodeled friction and compliance
- Environmental variations (lighting, surface, temperature)

**Bridging strategies:**

1. **Domain randomization**
   - Randomize physics parameters during training
   - Vary sensor noise, friction, mass
   - Train robust policies that work despite uncertainty

2. **System identification**
   - Measure real robot parameters precisely
   - Update simulation to match reality
   - Iteratively refine model

3. **Sim-to-real adaptation**
   - Fine-tune in simulation
   - Quick adaptation on real hardware
   - Use reality gap as a feature (domain adaptation)

4. **Hardware-in-the-loop**
   - Connect real sensors to simulation
   - Test sensor processing with simulated actuators
   - Gradual transition from sim to real

## 6. Practical Example: Square Pattern Controller

This example demonstrates a robot driving in a 2m × 2m square using odometry feedback.

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class SquarePatternController(Node):
    """Drive robot in square pattern using state machine"""

    STATE_FORWARD = 0
    STATE_TURN = 1

    def __init__(self):
        super().__init__('square_controller')

        # Publishers and subscribers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )

        # Parameters
        self.side_length = 2.0  # meters
        self.linear_speed = 0.2  # m/s
        self.angular_speed = 0.5  # rad/s

        # State
        self.state = self.STATE_FORWARD
        self.start_x = 0.0
        self.start_y = 0.0
        self.start_theta = 0.0
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        self.sides_completed = 0

        # Control loop
        self.timer = self.create_timer(0.1, self.control_loop)

    def odom_callback(self, msg):
        """Update position from odometry"""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        # Convert quaternion to yaw
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_theta = math.atan2(siny_cosp, cosy_cosp)

    def control_loop(self):
        """State machine for square pattern"""
        twist = Twist()

        if self.state == self.STATE_FORWARD:
            # Calculate distance traveled
            distance = math.sqrt(
                (self.current_x - self.start_x) ** 2 +
                (self.current_y - self.start_y) ** 2
            )

            if distance < self.side_length:
                # Keep moving forward
                twist.linear.x = self.linear_speed
            else:
                # Switch to turning
                self.state = self.STATE_TURN
                self.start_theta = self.current_theta
                self.sides_completed += 1
                self.get_logger().info(f'Side {self.sides_completed}/4 complete')

        elif self.state == self.STATE_TURN:
            # Calculate angle turned
            angle_turned = abs(self.current_theta - self.start_theta)

            if angle_turned < math.pi / 2 - 0.1:  # 90° minus tolerance
                # Keep turning
                twist.angular.z = self.angular_speed
            else:
                # Switch to forward
                self.state = self.STATE_FORWARD
                self.start_x = self.current_x
                self.start_y = self.current_y

                if self.sides_completed >= 4:
                    self.get_logger().info('Square complete!')
                    self.cmd_pub.publish(Twist())  # Stop
                    self.timer.cancel()
                    return

        self.cmd_pub.publish(twist)

def main():
    rclpy.init()
    controller = SquarePatternController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**How it works:**
1. **State machine**: Alternates between driving forward and turning
2. **Odometry feedback**: Tracks position and orientation
3. **Distance calculation**: Determines when to switch states
4. **Angle measurement**: Ensures 90° turns
5. **Completion**: Stops after 4 sides

**Run it:**
```bash
# Terminal 1: Launch Gazebo with TurtleBot3
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo empty_world.launch.py

# Terminal 2: Run controller
python3 simple_robot_controller.py
```

## 7. Best Practices

### Performance Optimization

1. **Simplify collision geometry**
   - Use primitive shapes (boxes, cylinders) instead of meshes
   - Reduce polygon count for visual meshes

2. **Disable unused features**
   - Turn off shadows if not needed
   - Reduce sensor update rates
   - Disable GUI rendering when running headless

3. **Adjust time step**
   - Larger time step = faster simulation, less accurate
   - Find balance for your application

### Debugging Tips

1. **Visualize frames**: Use RViz to see coordinate frames
2. **Check topics**: `ros2 topic list` and `ros2 topic echo /odom`
3. **Monitor TF**: `ros2 run tf2_tools view_frames`
4. **Log messages**: Use `self.get_logger().info()` liberally
5. **Pause simulation**: Press space bar to freeze and inspect

### Common Issues

**Robot falls through ground:**
- Check collision geometry is present
- Ensure mass and inertia are defined
- Verify ground plane exists

**Slow simulation:**
- Reduce physics accuracy (larger time step)
- Simplify models
- Close GUI panels
- Run headless (no graphics)

**Unstable motion/vibrations:**
- Increase solver iterations
- Reduce time step
- Check joint limits and damping

## Summary

In this chapter, we learned:

- **Gazebo** is a physics-based robot simulator for safe, fast iteration
- **Physics engines** (ODE, Bullet, Simbody) trade off speed vs accuracy
- **Digital twins** mirror physical robots for algorithm development
- **Sim-to-real gap** requires careful modeling and domain randomization
- **ROS 2 integration** allows same code to work in sim and on real hardware
- **State machines** enable structured control logic for complex behaviors

**Key benefits of simulation:**
- Test dangerous scenarios safely
- Iterate algorithms much faster than real-time
- Reduce hardware costs during development
- Generate synthetic training data at scale

**Remember**: Simulation is a tool, not a replacement for real-world testing. Always validate on hardware.

## Review Questions

1. What is the main advantage of using Gazebo for robotics development?
2. How do physics engines like ODE and Bullet differ?
3. What is the "sim-to-real gap" and why does it matter?
4. Why add sensor noise in simulation?
5. How does odometry help the square pattern controller?
6. What would happen if the controller didn't check angle tolerance in turns?
7. Name three strategies for improving sim-to-real transfer.

## Hands-on Exercises

### Exercise 1: Modify the Square
- Change side length to 3 meters
- Make the robot drive a triangle instead
- Add obstacle avoidance using `/scan` topic

### Exercise 2: Sensor Exploration
- Spawn TurtleBot3 in `turtlebot3_world.launch.py`
- Subscribe to `/scan` and `/camera/image_raw`
- Visualize in RViz
- Compare sensor data in different environments

### Exercise 3: Custom World
- Create a world file with walls and obstacles
- Spawn a robot and test navigation
- Add simulated noise to sensors
- Compare noisy vs ideal sensor behavior

## Further Reading

- [Gazebo Documentation](http://classic.gazebosim.org/)
- [ROS 2 + Gazebo Integration](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Gazebo/Gazebo.html)
- [TurtleBot3 Simulation](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/)
- [Digital Twin Concepts](https://www.sciencedirect.com/science/article/pii/S0736584520301314)
- Code example: `/static/code/digital-twin/chapter07/`

---

**Next Chapter**: [Sensor Modeling in Simulation →](ch08-sensor-modeling.md)

**Previous Chapter**: [← ROS 2 Nav2 Basics](../../module-01-ros2/week-05/ch06-nav2-basics.md)
