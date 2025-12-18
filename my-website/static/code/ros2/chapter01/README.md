# Chapter 1: ROS 2 Nodes and Topics - Code Examples

This directory contains executable code examples for Chapter 1 of the Physical AI & Humanoid Robotics textbook.

## Files

1. **hello_publisher.py** - Simple publisher that sends "Hello, ROS 2!" messages
2. **hello_subscriber.py** - Simple subscriber that receives and displays messages
3. **wall_follower.py** - Reactive wall-following robot using proportional control

## Prerequisites

- ROS 2 Humble (or later)
- Python 3.10+
- Required ROS 2 packages:
  - `rclpy` - ROS 2 Python client library
  - `std_msgs` - Standard message types
  - `sensor_msgs` - Sensor message types
  - `geometry_msgs` - Geometry message types

## Installation

If you haven't already, install ROS 2:

```bash
# Ubuntu 22.04
sudo apt update
sudo apt install ros-humble-desktop

# Source ROS 2
source /opt/ros/humble/setup.bash
```

## Running the Examples

### Example 1: Hello World Publisher/Subscriber

This demonstrates the basic pub/sub pattern in ROS 2.

**Terminal 1 - Run the publisher:**
```bash
cd /path/to/static/code/ros2/chapter01
python3 hello_publisher.py
```

**Terminal 2 - Run the subscriber:**
```bash
cd /path/to/static/code/ros2/chapter01
python3 hello_subscriber.py
```

You should see:
- Publisher: "Published: Hello, ROS 2! Message #0", incrementing each second
- Subscriber: "Received: Hello, ROS 2! Message #0", matching the publisher

**To verify topics are working:**
```bash
# In a third terminal
ros2 topic list        # Should show /hello_topic
ros2 topic echo /hello_topic  # Should display messages
ros2 node list         # Should show hello_publisher and hello_subscriber
```

### Example 2: Wall Following Robot

This demonstrates reactive behavior and closed-loop control.

**Option A: Run with simulation (requires Gazebo)**

```bash
# Terminal 1 - Launch Gazebo with a simple world
ros2 launch gazebo_ros empty_world.launch.py

# Terminal 2 - Spawn a robot with laser scanner (TurtleBot3 or custom URDF)
# (Setup depends on your robot model)

# Terminal 3 - Run wall follower
python3 wall_follower.py
```

**Option B: Run with custom parameters**

```bash
python3 wall_follower.py --ros-args \
  -p target_distance:=0.7 \
  -p forward_speed:=0.3 \
  -p kp:=0.8
```

**Parameters:**
- `target_distance` (default: 0.5m) - Desired distance from wall
- `forward_speed` (default: 0.2 m/s) - Robot forward velocity
- `kp` (default: 0.5) - Proportional gain for controller
- `min_distance` (default: 0.2m) - Safety threshold

**Expected Behavior:**
The robot should move forward while maintaining a constant distance from the wall on its right side. If it drifts away from the wall, it will turn right. If it gets too close, it will turn left.

**Debugging:**
```bash
# Monitor velocity commands
ros2 topic echo /cmd_vel

# Monitor laser scan data
ros2 topic echo /scan

# View node info
ros2 node info /wall_follower
```

## Key Concepts Demonstrated

### hello_publisher.py & hello_subscriber.py
- Creating ROS 2 nodes using `rclpy`
- Publishing messages with `create_publisher()`
- Subscribing to topics with `create_subscription()`
- Using timers for periodic publishing
- Proper node lifecycle (init, spin, shutdown)

### wall_follower.py
- **Embodied Intelligence**: Robot behavior emerges from sensor-motor coupling
- **Closed-Loop Control**: Continuous feedback from laser scanner
- **Proportional Control**: Correction proportional to error from target distance
- **Reactive Behavior**: Immediate response to sensor input (no planning)
- **ROS 2 Parameters**: Runtime configuration without code changes

## Troubleshooting

**Problem**: `ModuleNotFoundError: No module named 'rclpy'`
**Solution**: Make sure ROS 2 is sourced: `source /opt/ros/humble/setup.bash`

**Problem**: No output from subscriber
**Solution**:
1. Check both nodes are running: `ros2 node list`
2. Check topic exists: `ros2 topic list`
3. Verify publisher is publishing: `ros2 topic echo /hello_topic`

**Problem**: Wall follower doesn't move
**Solution**:
1. Check laser scan topic: `ros2 topic echo /scan` (should show data)
2. Verify correct topic names match your robot
3. Check for error messages in node output

**Problem**: Wall follower is unstable or oscillates
**Solution**: Tune the proportional gain (`kp` parameter):
- Decrease `kp` for smoother, slower response
- Increase `kp` for faster, more aggressive response
- Typical range: 0.3 to 1.0

## Learning Exercises

1. **Modify the publisher**: Change the message format and publish rate
2. **Add a filter**: Modify subscriber to only print messages containing specific text
3. **Tune the controller**: Experiment with different `kp` values in wall_follower.py
4. **Add safety features**: Implement front obstacle detection in wall_follower.py
5. **Create a new behavior**: Combine multiple sensor readings for corridor following

## Further Reading

- [ROS 2 Python Tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)
- [Understanding ROS 2 Topics](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html)
- [rclpy API Documentation](https://docs.ros2.org/latest/api/rclpy/)

## License

These examples are provided for educational purposes under the MIT License.
