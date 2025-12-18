# Module 2 Week 6: Gazebo Physics Simulation - Code Examples

Code examples for controlling robots in Gazebo simulation, demonstrating digital twin concepts.

## Files

1. **simple_robot_controller.py** - State machine controller that drives a robot in a square pattern

## Overview

These examples demonstrate:
- **Digital Twin Concept**: Virtual robot in simulation mirrors physical robot behavior
- **Physics Simulation**: Gazebo provides realistic physics (friction, inertia, collisions)
- **Sensor Simulation**: Odometry data from simulated sensors
- **Closed-Loop Control**: Using feedback from simulation to control behavior

## Prerequisites

- ROS 2 Humble or later
- Gazebo (included with ROS 2 desktop)
- TurtleBot3 simulation packages (or any differential drive robot)

```bash
sudo apt install ros-humble-gazebo-ros-pkgs
sudo apt install ros-humble-turtlebot3-gazebo
```

## Running the Example

### Step 1: Launch Gazebo with a robot

```bash
# Set robot model
export TURTLEBOT3_MODEL=burger

# Launch empty world with TurtleBot3
ros2 launch turtlebot3_gazebo empty_world.launch.py
```

### Step 2: Run the controller

```bash
cd /path/to/static/code/digital-twin/chapter07
python3 simple_robot_controller.py
```

**Expected Behavior:**
The robot will drive in a 2m x 2m square pattern:
1. Drive forward 2 meters
2. Turn 90 degrees
3. Repeat 4 times
4. Stop

### Modify Parameters

```bash
python3 simple_robot_controller.py --ros-args \
  -p side_length:=3.0 \
  -p linear_speed:=0.3 \
  -p angular_speed:=0.7
```

## Key Concepts

### Digital Twin Architecture

```
Physical Robot <--> Digital Twin (Gazebo)
       |                    |
       |                    |
   Sensors              Simulated Sensors
   Actuators            Simulated Actuators
       |                    |
       └─── Same Controller ───┘
```

**Benefits:**
- Test algorithms safely before deploying to hardware
- Iterate faster (no hardware setup time)
- Simulate dangerous scenarios
- Generate synthetic training data

### State Machine Pattern

This example uses a simple state machine for sequential behavior:

```
      ┌─────────┐
      │ FORWARD │
      └────┬────┘
           │ distance >= target
           ▼
       ┌──────┐
       │ TURN │
       └───┬──┘
           │ angle >= 90°
           └──────────► Back to FORWARD
```

### Odometry Feedback

Odometry provides robot pose estimation:
- **Position**: (x, y, z) in world frame
- **Orientation**: Quaternion (w, x, y, z)
- **Velocities**: Linear and angular

**Limitations:**
- Accumulates error over time (drift)
- Wheel slip causes inaccuracies
- Need sensor fusion for accurate localization (later chapters)

## Gazebo Topics

When running this example, these topics are active:

```bash
# Velocity commands (published by controller)
ros2 topic echo /cmd_vel

# Odometry (published by Gazebo)
ros2 topic echo /odom

# Laser scan (if robot has LiDAR)
ros2 topic echo /scan

# Ground truth pose (Gazebo only, not available on real robot)
ros2 topic echo /gazebo/model_states
```

## Debugging

**Visualize in RViz:**
```bash
ros2 run rviz2 rviz2
# Add displays: RobotModel, Odometry, LaserScan
```

**Monitor position:**
```bash
ros2 topic echo /odom --field pose.pose.position
```

**Check for errors:**
```bash
ros2 wtf  # Diagnose ROS 2 setup issues
```

## Comparison: Simulation vs Real Robot

| Aspect | Gazebo Simulation | Real Robot |
|--------|------------------|------------|
| **Physics** | Simplified (fast) | Complex (realistic) |
| **Sensors** | Ideal (no noise by default) | Noisy, unreliable |
| **Safety** | Can crash, no damage | Damage risk |
| **Iteration** | Very fast | Slow (hardware setup) |
| **Cost** | Free | Expensive |
| **Realism** | Sim-to-real gap | Ground truth |

## Exercises

1. **Add obstacle avoidance:**
   - Subscribe to `/scan` topic
   - Stop if obstacle detected within 0.5m
   - Resume when path is clear

2. **Implement figure-8 pattern:**
   - Drive in a figure-8 shape
   - Use odometry for position tracking
   - Challenge: smooth curves vs sharp turns

3. **Add visualization:**
   - Publish markers to show the planned path
   - Visualize current state in RViz
   - Display completed portion of square

4. **Sim-to-real transfer:**
   - Test controller in simulation
   - Deploy to real TurtleBot3
   - Compare behavior and identify differences

5. **Multi-robot coordination:**
   - Launch two robots in Gazebo
   - Coordinate to avoid collisions
   - Maintain formation while moving

## Common Issues

**Problem:** Robot doesn't move
**Solution:**
- Check Gazebo is running: `gz topic -l`
- Verify `/cmd_vel` topic: `ros2 topic info /cmd_vel`
- Ensure robot is spawned: Look for robot in Gazebo window

**Problem:** Odometry not updating
**Solution:**
- Check `/odom` topic: `ros2 topic hz /odom`
- Verify topic remapping if using custom robot
- Check Gazebo plugins are loaded

**Problem:** Square is not accurate
**Solution:**
- Increase tolerance in angle/distance checks
- Tune PID gains (if using velocity control)
- Account for acceleration/deceleration time

## Further Reading

- [Gazebo Classic Documentation](http://classic.gazebosim.org/)
- [ROS 2 + Gazebo Integration](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Gazebo/Gazebo.html)
- [TurtleBot3 Simulation](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/)
- Chapter 7: Gazebo Physics Simulation
- Chapter 10: Digital Twin Concepts

## License

MIT License - Educational purposes only.
