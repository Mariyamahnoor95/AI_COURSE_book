# Foundations Week 1 - Code Examples

Code examples for Embodied Intelligence and Sensor-Motor Loops chapter.

## Files

1. **simple_reactive_behavior.py** - Obstacle avoidance using reactive control

## Overview

These examples demonstrate foundational concepts in Physical AI:
- **Embodied Intelligence**: Intelligence emerges from physical interaction
- **Sensor-Motor Loops**: Direct coupling between perception and action
- **Reactive Behavior**: Immediate response without planning

## Prerequisites

- ROS 2 Humble or later
- Python 3.10+
- A simulated robot with laser scanner (TurtleBot3 recommended)

## Running the Example

### Setup Simulation (TurtleBot3)

```bash
# Install TurtleBot3 packages
sudo apt install ros-humble-turtlebot3*

# Set robot model
export TURTLEBOT3_MODEL=burger

# Launch Gazebo world with obstacles
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

### Run Reactive Behavior

```bash
# In a new terminal
cd /path/to/static/code/foundations/chapter01
python3 simple_reactive_behavior.py
```

**Expected Behavior:**
The robot will move forward until it detects an obstacle within 0.6m, then turn away from it. This creates emergent exploration behavior without any planning or map building.

### Modify Parameters

```bash
python3 simple_reactive_behavior.py --ros-args \
  -p obstacle_threshold:=0.8 \
  -p default_speed:=0.3 \
  -p turn_speed:=0.7
```

## Key Concepts

### Reactive Control
- No internal world model
- Immediate stimulus-response mapping
- Fast, low-latency behavior
- Suitable for dynamic environments

### Sensor-Motor Loop
```
Sensors → Perception → Decision → Action → Environment
   ↑                                           ↓
   └─────────── Feedback ─────────────────────┘
```

### Comparison: Reactive vs Deliberative

| Aspect | Reactive (This Example) | Deliberative |
|--------|------------------------|--------------|
| Speed | Fast (<10ms) | Slow (100-1000ms) |
| Model | No world model | Maintains map |
| Planning | No planning | Path planning |
| Adaptability | High | Moderate |
| Optimality | Not optimal | Can find optimal paths |

## Exercises

1. **Add memory**: Make the robot remember where it saw obstacles
2. **Combine sensors**: Use both front and side laser readings
3. **Goal-directed**: Add a target position and navigate toward it while avoiding obstacles
4. **Compare approaches**: Implement a deliberative planner and compare performance

## Troubleshooting

**Robot doesn't move:**
- Check `/scan` topic: `ros2 topic echo /scan`
- Verify Gazebo is running and robot is spawned
- Check node is running: `ros2 node list`

**Robot gets stuck in corners:**
- This is a known limitation of purely reactive control
- Try adjusting `obstacle_threshold` and `turn_speed`
- For better performance, combine with deliberative planning (later chapters)

## Further Reading

- Brooks, R. A. (1991). "Intelligence without Representation"
- Pfeifer & Bongard (2006). "How the Body Shapes the Way We Think"
- Chapter 1: Embodied Intelligence and Sensor-Motor Loops

## License

MIT License - Educational purposes only.
