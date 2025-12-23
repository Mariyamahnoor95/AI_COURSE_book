---
id: ch03-actuators-robotics-arch
title: Actuators and Robotics Architectures
sidebar_label: Actuators and Architectures
sidebar_position: 2
---

# Actuators and Robotics Architectures

## Learning Objectives

By the end of this chapter, you will be able to:

- Understand different types of actuators (electric motors, servos, hydraulic, pneumatic)
- Compare DC motors, stepper motors, and servo motors for robotic applications
- Explain hydraulic and pneumatic systems and their trade-offs
- Describe hierarchical control architectures (reactive, deliberative, hybrid)
- Identify common robot platform types (mobile, manipulator, humanoid, aerial)
- Apply safety principles and redundancy strategies in robot design

## Introduction

If sensors are the eyes and ears of Physical AI systems, then **actuators are the muscles**. Actuators convert electrical, hydraulic, or pneumatic energy into mechanical motion, enabling robots to interact with and manipulate the physical world.

**The actuator challenge**: Robots must perform diverse physical tasks:
- **Move through environments** (mobile robots need wheels, legs, or propellers)
- **Manipulate objects** (arms need joints with precise control)
- **Apply forces** (grippers, drills, and tools need power and feedback)
- **React quickly** (safety requires fast emergency stops)

Unlike software systems that manipulate bits, embodied AI systems must generate real forces and motions with:
- **Precision**: Position accuracy within millimeters
- **Speed**: Response times under 10 milliseconds for safety-critical tasks
- **Power**: Sufficient torque to lift, push, or hold loads
- **Reliability**: Consistent performance over millions of cycles

This chapter explores the main actuator technologies used in robotics and examines control architectures that coordinate sensors and actuators to achieve intelligent behavior.

## 1. Electric Motors and Servos

Electric motors are the most common actuators in modern robotics due to their versatility, efficiency, and precise control.

### DC Brushed Motors

**How they work:**
- Electric current flows through coils in a magnetic field
- Commutator switches current direction to maintain rotation
- Speed proportional to voltage, torque proportional to current

**Advantages:**
- ✅ Simple and inexpensive ($2-50)
- ✅ Easy to control (just vary voltage)
- ✅ High speed (thousands of RPM possible)
- ✅ Good power-to-weight ratio

**Limitations:**
- ❌ Brushes wear out over time
- ❌ Electrical noise from commutation
- ❌ No position feedback (need external encoder)
- ❌ Limited precision without closed-loop control

**Common uses:**
- Wheeled mobile robots (drivetrain)
- Small hobbyist projects
- Continuous rotation applications
- Cost-sensitive designs

### Brushless DC Motors (BLDC)

**How they work:**
- Electronic controller (ESC) switches current to coils
- No physical commutator (brushes)
- Requires hall sensors or back-EMF sensing for commutation

**Advantages:**
- ✅ No brush wear (longer lifespan)
- ✅ Higher efficiency (90%+ vs 75-80% brushed)
- ✅ Higher power density
- ✅ Less electrical noise

**Limitations:**
- ❌ More expensive ($20-500+)
- ❌ Requires electronic speed controller (ESC)
- ❌ More complex control

**Common uses:**
- Quadcopters and drones
- Electric vehicles
- High-performance robotics
- Industrial automation

### Stepper Motors

**How they work:**
- Rotate in precise steps (e.g., 1.8° per step = 200 steps/revolution)
- Energize coils in sequence to move step-by-step
- Open-loop control (no feedback sensor needed)

**Advantages:**
- ✅ Precise positioning without encoders
- ✅ Holds position when powered
- ✅ Predictable and repeatable
- ✅ No feedback loop required

**Limitations:**
- ❌ Can lose steps under high load (no feedback)
- ❌ Lower speed than DC motors
- ❌ Noisy and vibrate at certain speeds
- ❌ Consume power even when stationary

**Common uses:**
- 3D printers (precise positioning)
- CNC machines
- Camera gimbals
- Small robot joints

### Servo Motors

**What is a servo?**
A servo is not a motor type but a **closed-loop control system** consisting of:
1. **Motor** (usually DC or BLDC)
2. **Position sensor** (potentiometer or encoder)
3. **Control circuit** (compares desired vs actual position)
4. **Gearbox** (reduces speed, increases torque)

**Types:**

**1. Hobby Servos** (RC servos)
- Control signal: PWM (1-2ms pulse width)
- Rotation: Limited (0-180° typical)
- Torque: Low to medium (2-20 kg·cm)
- Cost: $5-100
- Uses: Robot arms, pan-tilt mechanisms

**2. Industrial Servos**
- Control signal: Analog voltage, digital protocols (EtherCAT, CANopen)
- Rotation: Continuous with multi-turn encoders
- Torque: High (up to thousands of N·m)
- Cost: $500-$10,000+
- Uses: Industrial robots, CNC, precision automation

**Example: Position control with servo**
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class ServoController(Node):
    def __init__(self):
        super().__init__('servo_controller')
        self.publisher = self.create_publisher(Float64, '/joint_position', 10)
        self.timer = self.create_timer(0.02, self.control_loop)  # 50 Hz
        self.target_angle = 0.0

    def control_loop(self):
        """Send target position to servo"""
        msg = Float64()
        msg.data = self.target_angle
        self.publisher.publish(msg)

    def set_target(self, angle_radians):
        """Set desired angle in radians"""
        self.target_angle = angle_radians
```

### Choosing Electric Motors

| Application | Recommended Motor |
|-------------|-------------------|
| Mobile robot wheels | DC brushed or BLDC |
| Drone propellers | BLDC (high efficiency) |
| 3D printer axes | Stepper motors |
| Robot arm joints | Servo motors (hobby or industrial) |
| Gripper | Servo or small DC with encoder |
| High-precision positioning | Industrial servo or stepper |

## 2. Hydraulic and Pneumatic Systems

For applications requiring high force or power, hydraulic and pneumatic actuators often outperform electric motors.

### Hydraulic Actuators

**How they work:**
- Pressurized fluid (oil) flows through valves
- Fluid pushes piston or rotates hydraulic motor
- Power transmission through fluid pressure

**Characteristics:**
- **Force**: Very high (lift tons)
- **Precision**: Moderate (harder than electric)
- **Speed**: Moderate
- **Compliance**: Low (rigid, strong)

**Advantages:**
- ✅ Extremely high force-to-weight ratio
- ✅ Can hold loads without power consumption
- ✅ Smooth motion at varying speeds
- ✅ Overload protection (pressure relief valves)

**Limitations:**
- ❌ Complex system (pump, reservoir, valves, lines)
- ❌ Maintenance intensive (leaks, fluid changes)
- ❌ Messy (oil leaks)
- ❌ Expensive ($1,000-$100,000+ systems)

**Common uses:**
- Excavators and construction equipment
- Heavy-duty industrial robots
- Humanoid robots (e.g., Boston Dynamics Atlas uses hydraulics)
- Aircraft control surfaces
- Large-scale manipulation

### Pneumatic Actuators

**How they work:**
- Compressed air (5-10 bar) drives pistons
- Valves control airflow direction
- Exhaust air vented to atmosphere

**Characteristics:**
- **Force**: Moderate (less than hydraulic)
- **Precision**: Low (compressible air)
- **Speed**: Fast (lightweight, low inertia)
- **Compliance**: High (soft, safe for human interaction)

**Advantages:**
- ✅ Clean (no fluid leaks)
- ✅ Fast response times
- ✅ Naturally compliant (safety)
- ✅ Simple and cheap components

**Limitations:**
- ❌ Requires air compressor
- ❌ Poor position control (air compresses)
- ❌ Noisy
- ❌ Energy inefficient

**Common uses:**
- Soft robotics and grippers (compliant grasping)
- Pick-and-place automation
- Pneumatic artificial muscles (PAMs)
- Human-safe collaborative robots
- Medical devices

### Comparison: Electric vs Hydraulic vs Pneumatic

| Feature | Electric | Hydraulic | Pneumatic |
|---------|----------|-----------|-----------|
| **Force** | Low-Medium | Very High | Medium |
| **Precision** | Excellent | Good | Poor |
| **Speed** | High | Medium | Very High |
| **Efficiency** | High (90%) | Medium (60%) | Low (30%) |
| **Compliance** | Low | Low | High |
| **Cleanliness** | Excellent | Poor (oil) | Excellent |
| **Cost** | Low-Medium | High | Medium |
| **Maintenance** | Low | High | Medium |

**Rule of thumb**:
- **Electric**: Precision, efficiency, most applications
- **Hydraulic**: High force, heavy lifting
- **Pneumatic**: Speed, compliance, safety

## 3. Control System Hierarchies

Robot control architectures organize how sensors, actuators, and decision-making layers interact.

### Hierarchical (Deliberative) Architecture

**Structure**: Sequential layers from high-level planning to low-level control

```
┌─────────────────────────────┐
│  Mission Planning           │  (Path planning, task scheduling)
│  (Slow, complex reasoning)  │
└─────────────┬───────────────┘
              │
┌─────────────▼───────────────┐
│  Tactical Execution         │  (Obstacle avoidance, waypoint tracking)
│  (Medium speed)             │
└─────────────┬───────────────┘
              │
┌─────────────▼───────────────┐
│  Low-Level Control          │  (Motor commands, PID loops)
│  (Fast, &lt;10ms cycles)     │
└─────────────────────────────┘
```

**Characteristics:**
- Top-down information flow
- Each layer abstracts complexity
- Global optimization possible

**Advantages:**
- ✅ Optimal path planning
- ✅ Clear separation of concerns
- ✅ Easy to reason about system behavior

**Limitations:**
- ❌ Slow reaction time (must propagate through layers)
- ❌ Fails if world model is inaccurate
- ❌ Not robust to unexpected events

**Example**: NASA Mars rovers (plan entire day's activities, then execute)

### Reactive (Behavior-Based) Architecture

**Structure**: Parallel behaviors that react directly to sensors

```
Sensors ─┬─> [Avoid Obstacles] ─┬─> Motor Commands
         ├─> [Follow Wall]     ─┤   (Behavior arbitration)
         ├─> [Seek Goal]       ─┤
         └─> [Low Battery?]    ─┘
```

**Characteristics:**
- No world model or planning
- Fast sensor-to-actuator loops
- Emergent behavior from interaction

**Advantages:**
- ✅ Extremely fast reactions (&lt;10ms)
- ✅ Robust to sensor noise and failures
- ✅ Works in unpredictable environments

**Limitations:**
- ❌ No long-term planning
- ❌ Difficult to predict behavior
- ❌ Hard to achieve complex goals

**Example**: Roomba vacuum (wall following + obstacle avoidance + random walk)

### Hybrid (Three-Layer) Architecture

**Structure**: Combines planning and reactive control

```
┌────────────────────────────────┐
│  Deliberative Layer            │  (Plan paths, make decisions)
│  (Slow, updates at 1-10 Hz)    │
└───────────┬────────────────────┘
            │ Goals/Plans
┌───────────▼────────────────────┐
│  Sequencing Layer              │  (Decompose plans into behaviors)
│  (Medium, 10-100 Hz)           │
└───────────┬────────────────────┘
            │ Behavior activation
┌───────────▼────────────────────┐
│  Reactive Layer                │  (Execute behaviors, safety reflexes)
│  (Fast, 100-1000 Hz)           │
└────────────────────────────────┘
```

**How it works:**
1. **Deliberative layer**: Computes global path (e.g., A* algorithm)
2. **Sequencing layer**: Breaks path into waypoints
3. **Reactive layer**: Follows waypoints while avoiding obstacles

**Advantages:**
- ✅ Best of both worlds (planning + reactivity)
- ✅ Handles unexpected obstacles while reaching goals
- ✅ Most common in modern robotics

**Example**: Autonomous vehicles, warehouse robots, humanoid robots

## 4. Robot Platforms and Architectures

### Mobile Robots

**Wheeled robots:**
- **Differential drive**: Two independent wheels (simple, turning in place)
- **Ackermann steering**: Car-like steering (fast, smooth, can't turn in place)
- **Omnidirectional**: Mecanum or holonomic wheels (move in any direction)

**Legged robots:**
- **Bipedal**: Humanoid, complex balance (e.g., Atlas, Digit)
- **Quadrupedal**: Four legs, stable (e.g., Spot, ANYmal)
- **Hexapod**: Six legs, very stable, slow

**Aerial robots:**
- **Quadcopters**: Four rotors, hover capability
- **Fixed-wing**: Efficient flight, no hovering

### Manipulators (Robot Arms)

**Types:**
- **Serial manipulator**: Chain of joints (most common)
- **Parallel manipulator**: Multiple chains to end-effector (e.g., Delta robot, Stewart platform)
- **Continuum manipulator**: Flexible, snake-like

**Common configurations:**
- **6-DOF arm**: Full 3D position + 3D orientation
- **7-DOF arm**: Redundant (multiple solutions for same pose)
- **Collaborative robots (cobots)**: Designed for human interaction

### Humanoid Robots

**Why humanoid?**
- Operate in human-designed environments
- Use human tools and interfaces
- Social interaction and acceptance

**Challenges:**
- Balance and locomotion (complex)
- High DOF (20-40+ joints)
- Energy efficiency
- Expensive and fragile

**Examples**: Atlas (Boston Dynamics), Optimus (Tesla), ASIMO (Honda), Ameca

### Hybrid Platforms

**Mobile manipulator**: Mobile base + arm (e.g., Fetch, TIAGo)
- Combines navigation and manipulation
- Can perform tasks across large spaces

**Aerial manipulator**: Drone + arm
- Access hard-to-reach locations
- Inspect bridges, power lines

## 5. Safety and Redundancy

### Safety Principles

**1. Mechanical Safety**
- **E-stops**: Physical emergency stop buttons
- **Torque limiting**: Limit joint torques to safe levels
- **Soft materials**: Compliant grippers, padded surfaces
- **Caging/barriers**: Physical separation from humans

**2. Software Safety**
- **Watchdog timers**: Detect software crashes
- **Motion limits**: Enforce joint and workspace boundaries
- **Collision detection**: Stop on unexpected resistance
- **Safe mode**: Fallback behavior when errors occur

**3. Sensor Safety**
- **Redundant sensors**: Multiple sensors for critical measurements
- **Sensor validation**: Detect faulty sensor data
- **Fail-safe defaults**: Assume safe state on sensor failure

### Redundancy Strategies

**Why redundancy?**
Robots in critical applications (surgery, nuclear, space) cannot afford single points of failure.

**Types:**

**1. Hardware redundancy**
- Duplicate sensors (e.g., dual IMUs)
- Backup actuators
- Redundant power supplies

**2. Analytical redundancy**
- Estimate sensor values from other sensors
- Example: Estimate velocity from position sensor

**3. Kinematic redundancy**
- 7-DOF arm for 6-DOF task (extra DOF for obstacle avoidance)

**Example: Redundant E-stop system**
```python
class SafetyController:
    def __init__(self):
        self.hardware_estop = False  # Physical button
        self.software_estop = False  # Software watchdog
        self.last_heartbeat = time.time()

    def check_safety(self):
        """Return True if safe to operate"""
        # Check physical e-stop
        if self.hardware_estop:
            return False

        # Check software heartbeat
        if time.time() - self.last_heartbeat > 0.5:  # 500ms timeout
            self.software_estop = True
            return False

        # Check sensor health
        if not self.sensors_healthy():
            return False

        return True

    def emergency_stop(self):
        """Immediately halt all motion"""
        for motor in self.motors:
            motor.disable()
        self.publish_estop_message()
        self.log_incident()
```

## Summary

Key takeaways from this chapter:

- **Electric motors** (DC, BLDC, stepper, servo) are the most common actuators for precision and versatility
- **Hydraulic systems** provide immense force for heavy-duty applications but are complex
- **Pneumatic systems** offer speed and compliance for safe human interaction
- **Control architectures** range from pure reactive (fast but limited) to deliberative (slow but optimal)
- **Hybrid architectures** combine planning and reactivity for real-world robotics
- **Robot platforms** include mobile, manipulators, humanoid, and hybrid configurations
- **Safety and redundancy** are critical for reliable and safe robot operation

**Design trade-offs:**
- Precision vs force (electric vs hydraulic)
- Planning vs reactivity (deliberative vs reactive)
- Simplicity vs capability (fewer DOF vs more DOF)
- Cost vs reliability (single components vs redundancy)

## Review Questions

1. When would you choose a stepper motor over a servo motor?
2. Why are pneumatic actuators preferred for soft grippers?
3. What is the main advantage of a hybrid control architecture over pure reactive control?
4. Why do humanoid robots have so many joints compared to mobile robots?
5. How does analytical redundancy differ from hardware redundancy?
6. What is the purpose of a watchdog timer in robot safety systems?
7. Why can't a 6-DOF arm avoid obstacles while maintaining end-effector pose, but a 7-DOF arm can?

## Hands-On Exercise

### Exercise 1: Motor Selection
Given a robot application, select appropriate actuators:
- Quadcopter requiring 10-minute flight time
- 3D printer with 0.1mm positioning accuracy
- Gripper for fragile eggs
- Heavy industrial manipulator lifting 100kg

Justify your choices based on force, precision, speed, and cost.

### Exercise 2: Safety System Design
Design a redundant safety system for a mobile robot:
- What sensors would you use?
- How would you detect sensor failures?
- What is the safe fallback behavior?
- Draw a block diagram showing redundancy

### Exercise 3: Control Architecture
Implement a simple hybrid architecture:
- Deliberative: Plan a path from A to B
- Reactive: Avoid obstacles using sensor data
- Sequencing: Break path into waypoints
- Observe how the robot balances planning and reactivity

## Further Reading

- [DC Motors Explained](https://www.machinedesign.com/motors-drives/article/21832047/whats-the-difference-between-ac-induction-permanent-magnet-and-servomotor-technologies)
- [Servo Motor Control](https://www.mathworks.com/discovery/servo-motor.html)
- [Hybrid Robot Architectures](https://www.ri.cmu.edu/pub_files/pub2/arkin_r_1998_1/arkin_r_1998_1.pdf)
- [Robot Safety Standards (ISO 10218)](https://www.iso.org/standard/51330.html)

---

**Next Chapter**: [Module 1: ROS 2 Nodes and Topics →](../../module-01-ros2/week-03/ch01-nodes-topics.md)

**Previous Chapter**: [← Sensors in Physical AI Systems](ch02-sensors.md)
