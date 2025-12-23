---
id: ch05-urdf-models
title: Robot Modeling with URDF
sidebar_label: Robot Modeling with URDF
sidebar_position: 6
---

# Robot Modeling with URDF

## Learning Objectives

By the end of this chapter, you will be able to:

- Understand URDF syntax and structure for robot description
- Define links (rigid bodies) and joints (connections) in URDF
- Specify visual geometry, collision geometry, and inertial properties
- Integrate URDF models with Gazebo simulation
- Use Xacro for modular, parameterized robot descriptions
- Visualize and debug URDF models in RViz

## Introduction

Every robot has a physical structure - wheels, arms, sensors, grippers - and to work with robots in simulation or visualization, ROS 2 needs a precise description of this geometry. **How do we describe a robot's physical structure in a way that both humans and software can understand?**

This is where **URDF** (Unified Robot Description Format) comes in. URDF is an XML-based format for describing:
- **Kinematics**: How parts connect and move
- **Geometry**: Visual appearance and collision shapes
- **Dynamics**: Masses, inertias, friction
- **Sensors**: Cameras, LiDAR, IMUs
- **Actuators**: Motors, servos, grippers

**Why URDF matters:**
- **Visualization** in RViz - see your robot's structure
- **Simulation** in Gazebo - test algorithms safely
- **Motion planning** - compute collision-free paths
- **TF integration** - automatic transform publishing
- **Documentation** - self-documenting robot structure

**Real-world scenario**: You're building a mobile manipulator. Without URDF, you'd manually:
- Compute forward kinematics for the arm
- Publish TF transforms for every joint
- Track collision geometries separately
- Recreate the robot structure in every simulator

With URDF, you describe the robot once, and ROS 2 tools automatically:
- Publish all TF transforms
- Visualize the robot in RViz
- Simulate physics in Gazebo
- Compute kinematics and collisions

This chapter teaches you how to create URDF models for any robot, from simple mobile bases to complex humanoids.

## 1. URDF Syntax and Structure

### What is URDF?

URDF is an **XML specification** that describes a robot as:
- **Links**: Rigid bodies (chassis, wheels, arms)
- **Joints**: Connections between links (hinges, sliders, fixed)

### Basic URDF Structure

```xml
&lt;?xml version="1.0"?&gt;
&lt;robot name="my_robot"&gt;

  &lt;!-- Links define rigid bodies --&gt;
  &lt;link name="base_link"&gt;
    &lt;visual&gt;...&lt;/visual&gt;
    &lt;collision&gt;...&lt;/collision&gt;
    &lt;inertial&gt;...&lt;/inertial&gt;
  &lt;/link&gt;

  &lt;link name="wheel_left"&gt;
    &lt;visual&gt;...&lt;/visual&gt;
    &lt;collision&gt;...&lt;/collision&gt;
    &lt;inertial&gt;...&lt;/inertial&gt;
  &lt;/link&gt;

  &lt;!-- Joints connect links --&gt;
  &lt;joint name="wheel_left_joint" type="continuous"&gt;
    &lt;parent link="base_link"/&gt;
    &lt;child link="wheel_left"/&gt;
    &lt;origin xyz="0 0.15 0" rpy="0 0 0"/&gt;
    &lt;axis xyz="0 1 0"/&gt;
  &lt;/joint&gt;

&lt;/robot&gt;
```

### URDF Tree Structure

URDF forms a **tree** (not a graph):
- Exactly **one root link** (typically `base_link`)
- Each link has **one parent** (via a joint)
- Links can have **multiple children**

```
base_link (root)
 ├─ wheel_left (via wheel_left_joint)
 ├─ wheel_right (via wheel_right_joint)
 └─ sensor_mount (via sensor_joint)
     └─ camera_link (via camera_joint)
```

## 2. Links and Joints

### Links: Rigid Bodies

A **link** represents a rigid body with three properties:

**1. Visual**: How it looks (for RViz visualization)
**2. Collision**: Shape for collision detection
**3. Inertial**: Mass and inertia (for physics simulation)

```xml
&lt;link name="base_link"&gt;
  &lt;!-- Visual geometry (what you see) --&gt;
  &lt;visual&gt;
    &lt;origin xyz="0 0 0.05" rpy="0 0 0"/&gt;
    &lt;geometry&gt;
      &lt;box size="0.5 0.3 0.1"/&gt;
    &lt;/geometry&gt;
    &lt;material name="blue"&gt;
      &lt;color rgba="0 0 1 1"/&gt;
    &lt;/material&gt;
  &lt;/visual&gt;

  &lt;!-- Collision geometry (for physics) --&gt;
  &lt;collision&gt;
    &lt;origin xyz="0 0 0.05" rpy="0 0 0"/&gt;
    &lt;geometry&gt;
      &lt;box size="0.5 0.3 0.1"/&gt;
    &lt;/geometry&gt;
  &lt;/collision&gt;

  &lt;!-- Inertial properties (for dynamics) --&gt;
  &lt;inertial&gt;
    &lt;mass value="10.0"/&gt;
    &lt;origin xyz="0 0 0.05" rpy="0 0 0"/&gt;
    &lt;inertia ixx="0.1" ixy="0" ixz="0"
             iyy="0.15" iyz="0"
             izz="0.2"/&gt;
  &lt;/inertial&gt;
&lt;/link&gt;
```

### Joints: Connections

A **joint** connects two links and defines how they move relative to each other.

**Joint types:**

| Type | Description | Example |
|------|-------------|---------|
| `fixed` | No movement | Camera mount |
| `revolute` | Rotation with limits | Robot arm joint |
| `continuous` | Rotation without limits | Wheel |
| `prismatic` | Linear sliding | Elevator, gripper |
| `planar` | 2D motion in plane | (rarely used) |
| `floating` | 6-DOF freedom | (rarely used) |

```xml
&lt;!-- Continuous joint for a wheel --&gt;
&lt;joint name="wheel_left_joint" type="continuous"&gt;
  &lt;parent link="base_link"/&gt;
  &lt;child link="wheel_left"/&gt;
  &lt;origin xyz="0 0.15 -0.05" rpy="-1.5708 0 0"/&gt;
  &lt;axis xyz="0 0 1"/&gt;
&lt;/joint&gt;

&lt;!-- Revolute joint with limits for arm --&gt;
&lt;joint name="shoulder_joint" type="revolute"&gt;
  &lt;parent link="base_link"/&gt;
  &lt;child link="upper_arm"/&gt;
  &lt;origin xyz="0.2 0 0.3" rpy="0 0 0"/&gt;
  &lt;axis xyz="0 1 0"/&gt;
  &lt;limit lower="-1.57" upper="1.57" effort="50" velocity="1.0"/&gt;
&lt;/joint&gt;

&lt;!-- Fixed joint for sensor --&gt;
&lt;joint name="camera_joint" type="fixed"&gt;
  &lt;parent link="base_link"/&gt;
  &lt;child link="camera_link"/&gt;
  &lt;origin xyz="0.3 0 0.2" rpy="0 0 0"/&gt;
&lt;/joint&gt;
```

**Key attributes:**
- `origin`: Position (xyz) and orientation (rpy = roll, pitch, yaw)
- `axis`: Rotation/translation axis (for revolute/prismatic/continuous)
- `limit`: Motion limits (position, velocity, effort/torque)

## 3. Visual and Collision Geometry

### Why Separate Visual and Collision?

**Visual geometry**: High detail for looks
**Collision geometry**: Simplified shapes for performance

```xml
&lt;link name="robot_arm"&gt;
  &lt;!-- Detailed visual (mesh file) --&gt;
  &lt;visual&gt;
    &lt;geometry&gt;
      &lt;mesh filename="package://my_robot/meshes/arm.dae" scale="1 1 1"/&gt;
    &lt;/geometry&gt;
  &lt;/visual&gt;

  &lt;!-- Simplified collision (cylinder approximation) --&gt;
  &lt;collision&gt;
    &lt;geometry&gt;
      &lt;cylinder radius="0.05" length="0.5"/&gt;
    &lt;/geometry&gt;
  &lt;/collision&gt;
&lt;/link&gt;
```

**Why simplify collision?**
- ✅ Faster collision detection
- ✅ More stable physics simulation
- ✅ Easier to debug contact points

### Geometry Primitives

**1. Box**
```xml
&lt;geometry&gt;
  &lt;box size="0.5 0.3 0.2"/&gt;  &lt;!-- width, depth, height --&gt;
&lt;/geometry&gt;
```

**2. Cylinder**
```xml
&lt;geometry&gt;
  &lt;cylinder radius="0.1" length="0.5"/&gt;
&lt;/geometry&gt;
```

**3. Sphere**
```xml
&lt;geometry&gt;
  &lt;sphere radius="0.1"/&gt;
&lt;/geometry&gt;
```

**4. Mesh** (for complex shapes)
```xml
&lt;geometry&gt;
  &lt;mesh filename="package://my_robot/meshes/part.stl" scale="0.001 0.001 0.001"/&gt;
&lt;/geometry&gt;
```

**Mesh formats:**
- `.stl` - Simple triangle mesh (most common)
- `.dae` (Collada) - Supports colors and textures
- `.obj` - Wavefront format

### Material and Color

```xml
&lt;!-- Inline color --&gt;
&lt;material name="red"&gt;
  &lt;color rgba="1 0 0 1"/&gt;  &lt;!-- red, green, blue, alpha --&gt;
&lt;/material&gt;

&lt;!-- Texture from file --&gt;
&lt;material name="wood_texture"&gt;
  &lt;texture filename="package://my_robot/textures/wood.png"/&gt;
&lt;/material&gt;
```

### Inertial Properties

Critical for accurate physics simulation:

```xml
&lt;inertial&gt;
  &lt;mass value="5.0"/&gt;
  &lt;origin xyz="0 0 0" rpy="0 0 0"/&gt;  &lt;!-- center of mass --&gt;
  &lt;inertia ixx="0.1" ixy="0.0" ixz="0.0"
           iyy="0.1" iyz="0.0"
           izz="0.1"/&gt;
&lt;/inertial&gt;
```

**Quick formulas for common shapes:**

**Box** (width w, depth d, height h):
```
ixx = (1/12) * m * (d² + h²)
iyy = (1/12) * m * (w² + h²)
izz = (1/12) * m * (w² + d²)
```

**Cylinder** (radius r, length l):
```
ixx = iyy = (1/12) * m * (3r² + l²)
izz = (1/2) * m * r²
```

**Sphere** (radius r):
```
ixx = iyy = izz = (2/5) * m * r²
```

## 4. Gazebo Integration

### Adding Gazebo-Specific Properties

URDF files can include Gazebo-specific tags:

```xml
&lt;gazebo reference="base_link"&gt;
  &lt;material&gt;Gazebo/Blue&lt;/material&gt;
  &lt;mu1&gt;0.2&lt;/mu1&gt;  &lt;!-- friction coefficient 1 --&gt;
  &lt;mu2&gt;0.2&lt;/mu2&gt;  &lt;!-- friction coefficient 2 --&gt;
  &lt;selfCollide&gt;false&lt;/selfCollide&gt;
&lt;/gazebo&gt;
```

### Adding Sensors

**LiDAR sensor:**
```xml
&lt;gazebo reference="lidar_link"&gt;
  &lt;sensor name="lidar_sensor" type="gpu_ray"&gt;
    &lt;always_on&gt;true&lt;/always_on&gt;
    &lt;update_rate&gt;10&lt;/update_rate&gt;
    &lt;ray&gt;
      &lt;scan&gt;
        &lt;horizontal&gt;
          &lt;samples&gt;360&lt;/samples&gt;
          &lt;resolution&gt;1&lt;/resolution&gt;
          &lt;min_angle&gt;-3.14159&lt;/min_angle&gt;
          &lt;max_angle&gt;3.14159&lt;/max_angle&gt;
        &lt;/horizontal&gt;
      &lt;/scan&gt;
      &lt;range&gt;
        &lt;min&gt;0.1&lt;/min&gt;
        &lt;max&gt;30.0&lt;/max&gt;
      &lt;/range&gt;
    &lt;/ray&gt;
    &lt;plugin name="lidar_plugin" filename="libgazebo_ros_ray_sensor.so"&gt;
      &lt;ros&gt;
        &lt;namespace&gt;/&lt;/namespace&gt;
        &lt;remapping&gt;~/out:=scan&lt;/remapping&gt;
      &lt;/ros&gt;
      &lt;output_type&gt;sensor_msgs/LaserScan&lt;/output_type&gt;
    &lt;/plugin&gt;
  &lt;/sensor&gt;
&lt;/gazebo&gt;
```

**Camera sensor:**
```xml
&lt;gazebo reference="camera_link"&gt;
  &lt;sensor name="camera" type="camera"&gt;
    &lt;always_on&gt;true&lt;/always_on&gt;
    &lt;update_rate&gt;30&lt;/update_rate&gt;
    &lt;camera&gt;
      &lt;horizontal_fov&gt;1.3962634&lt;/horizontal_fov&gt;
      &lt;image&gt;
        &lt;width&gt;1280&lt;/width&gt;
        &lt;height&gt;720&lt;/height&gt;
      &lt;/image&gt;
      &lt;clip&gt;
        &lt;near&gt;0.02&lt;/near&gt;
        &lt;far&gt;300&lt;/far&gt;
      &lt;/clip&gt;
    &lt;/camera&gt;
    &lt;plugin name="camera_plugin" filename="libgazebo_ros_camera.so"&gt;
      &lt;ros&gt;
        &lt;namespace&gt;/camera&lt;/namespace&gt;
        &lt;remapping&gt;image_raw:=image&lt;/remapping&gt;
      &lt;/ros&gt;
    &lt;/plugin&gt;
  &lt;/sensor&gt;
&lt;/gazebo&gt;
```

### Differential Drive Plugin

For mobile robots:

```xml
&lt;gazebo&gt;
  &lt;plugin name="diff_drive" filename="libgazebo_ros_diff_drive.so"&gt;
    &lt;ros&gt;
      &lt;namespace&gt;/&lt;/namespace&gt;
    &lt;/ros&gt;

    &lt;left_joint&gt;wheel_left_joint&lt;/left_joint&gt;
    &lt;right_joint&gt;wheel_right_joint&lt;/right_joint&gt;

    &lt;wheel_separation&gt;0.3&lt;/wheel_separation&gt;
    &lt;wheel_diameter&gt;0.2&lt;/wheel_diameter&gt;

    &lt;max_wheel_torque&gt;20&lt;/max_wheel_torque&gt;
    &lt;max_wheel_acceleration&gt;1.0&lt;/max_wheel_acceleration&gt;

    &lt;publish_odom&gt;true&lt;/publish_odom&gt;
    &lt;publish_odom_tf&gt;true&lt;/publish_odom_tf&gt;
    &lt;publish_wheel_tf&gt;true&lt;/publish_wheel_tf&gt;

    &lt;odometry_frame&gt;odom&lt;/odometry_frame&gt;
    &lt;robot_base_frame&gt;base_link&lt;/robot_base_frame&gt;
  &lt;/plugin&gt;
&lt;/gazebo&gt;
```

## 5. Xacro for Modularity

### Why Xacro?

**Problem with pure URDF:**
- Repetitive (define 4 wheels = copy-paste 4 times)
- Hard-coded values
- No parameterization

**Xacro** (XML Macros) adds:
- ✅ Variables and constants
- ✅ Mathematical expressions
- ✅ Macros (reusable components)
- ✅ File inclusion

### Xacro Example: Variables

```xml
&lt;?xml version="1.0"?&gt;
&lt;robot name="my_robot" xmlns:xacro="http://www.ros.org/wiki/xacro"&gt;

  &lt;!-- Define constants --&gt;
  &lt;xacro:property name="wheel_radius" value="0.1"/&gt;
  &lt;xacro:property name="wheel_width" value="0.05"/&gt;
  &lt;xacro:property name="wheel_separation" value="0.3"/&gt;
  &lt;xacro:property name="base_length" value="0.5"/&gt;

  &lt;!-- Use variables --&gt;
  &lt;link name="base_link"&gt;
    &lt;visual&gt;
      &lt;geometry&gt;
        &lt;box size="${base_length} ${wheel_separation + 0.1} 0.1"/&gt;
      &lt;/geometry&gt;
    &lt;/visual&gt;
  &lt;/link&gt;

&lt;/robot&gt;
```

### Xacro Macros

```xml
&lt;!-- Define reusable wheel macro --&gt;
&lt;xacro:macro name="wheel" params="prefix reflect"&gt;
  &lt;link name="${prefix}_wheel"&gt;
    &lt;visual&gt;
      &lt;geometry&gt;
        &lt;cylinder radius="${wheel_radius}" length="${wheel_width}"/&gt;
      &lt;/geometry&gt;
      &lt;material name="black"&gt;
        &lt;color rgba="0 0 0 1"/&gt;
      &lt;/material&gt;
    &lt;/visual&gt;

    &lt;collision&gt;
      &lt;geometry&gt;
        &lt;cylinder radius="${wheel_radius}" length="${wheel_width}"/&gt;
      &lt;/geometry&gt;
    &lt;/collision&gt;

    &lt;inertial&gt;
      &lt;mass value="0.5"/&gt;
      &lt;inertia ixx="0.001" ixy="0" ixz="0"
               iyy="0.001" iyz="0"
               izz="0.001"/&gt;
    &lt;/inertial&gt;
  &lt;/link&gt;

  &lt;joint name="${prefix}_wheel_joint" type="continuous"&gt;
    &lt;parent link="base_link"/&gt;
    &lt;child link="${prefix}_wheel"/&gt;
    &lt;origin xyz="0 ${reflect * wheel_separation / 2} 0" rpy="-1.5708 0 0"/&gt;
    &lt;axis xyz="0 0 1"/&gt;
  &lt;/joint&gt;
&lt;/xacro:macro&gt;

&lt;!-- Instantiate wheels --&gt;
&lt;xacro:wheel prefix="left" reflect="1"/&gt;
&lt;xacro:wheel prefix="right" reflect="-1"/&gt;
```

**Benefits:**
- Define wheel once, use twice
- Change `wheel_radius` in one place → updates everywhere
- Easy to add more wheels

### Include Files

```xml
&lt;!-- main_robot.xacro --&gt;
&lt;robot name="my_robot" xmlns:xacro="http://www.ros.org/wiki/xacro"&gt;

  &lt;!-- Include common definitions --&gt;
  &lt;xacro:include filename="$(find my_robot)/urdf/common.xacro"/&gt;
  &lt;xacro:include filename="$(find my_robot)/urdf/wheels.xacro"/&gt;
  &lt;xacro:include filename="$(find my_robot)/urdf/sensors.xacro"/&gt;

  &lt;!-- Use included macros --&gt;
  &lt;xacro:base_link/&gt;
  &lt;xacro:wheels/&gt;
  &lt;xacro:camera mount_point="base_link"/&gt;

&lt;/robot&gt;
```

### Converting Xacro to URDF

```bash
# Generate URDF from Xacro
ros2 run xacro xacro my_robot.urdf.xacro &gt; my_robot.urdf

# Check URDF validity
check_urdf my_robot.urdf

# Visualize kinematic tree
urdf_to_graphviz my_robot.urdf
```

## 6. Visualization and Debugging

### Visualize in RViz

```bash
# Launch robot state publisher with URDF
ros2 launch my_robot display.launch.py

# Or manually:
ros2 run robot_state_publisher robot_state_publisher --ros-args \
  -p robot_description:="$(xacro /path/to/robot.urdf.xacro)"

ros2 run joint_state_publisher_gui joint_state_publisher_gui

rviz2
```

**In RViz:**
1. Add "RobotModel" display
2. Set Fixed Frame to `base_link`
3. Use joint sliders to move joints

### Check URDF Validity

```bash
# Install tools
sudo apt install liburdfdom-tools

# Check for errors
check_urdf my_robot.urdf

# Output should show:
# robot name is: my_robot
# ---------- Successfully Parsed XML ---------------
```

### Common URDF Errors

**Error 1: Joint without parent/child**
```
Error: joint [wheel_joint] missing parent or child
```
**Fix:** Ensure every joint has both `&lt;parent&gt;` and `&lt;child&gt;` tags

**Error 2: Disconnected link**
```
Warning: link [sensor_link] is not connected to any parent
```
**Fix:** Create a joint connecting the link to the tree

**Error 3: Invalid inertia**
```
Warning: inertia matrix for link [base_link] is not positive definite
```
**Fix:** Use proper inertia formulas or increase diagonal values

## Summary

Key takeaways from this chapter:

- **URDF** describes robot structure in XML format
- **Links** are rigid bodies (visual, collision, inertial properties)
- **Joints** connect links and define motion constraints
- **Visual geometry** (detailed meshes) vs **collision geometry** (simplified)
- **Gazebo tags** add sensors, materials, and physics properties
- **Xacro** enables modular, parameterized robot descriptions
- ROS 2 tools automatically publish TF transforms from URDF

**Best practices:**
- Keep visual and collision geometry simple for performance
- Always define inertial properties for physics simulation
- Use Xacro for any robot with repeated components
- Separate URDF into multiple files (base, wheels, sensors)
- Test in RViz before Gazebo
- Use descriptive link and joint names

## Review Questions

1. What is the difference between visual and collision geometry?
2. Why does URDF form a tree structure instead of a graph?
3. What are the six joint types and when would you use each?
4. How do you calculate inertia for a box-shaped link?
5. Why use Xacro instead of pure URDF?
6. What Gazebo plugin enables differential drive control?
7. How do you convert a Xacro file to URDF?

## Hands-on Exercises

### Exercise 1: Simple Robot URDF
Create a URDF for a two-wheeled robot:
- Box-shaped base (0.5m × 0.3m × 0.1m)
- Two cylinder wheels (radius 0.1m, width 0.05m)
- Continuous joints for wheels
- Visualize in RViz

### Exercise 2: Add a Camera
Extend Exercise 1:
- Add camera link (small box 0.05m cube)
- Fixed joint mounting camera on front of base
- Define Gazebo camera sensor
- Test in Gazebo, verify `/camera/image` topic

### Exercise 3: Xacro Conversion
Convert Exercise 1 to Xacro:
- Define constants for dimensions
- Create wheel macro
- Instantiate left and right wheels using macro
- Generate URDF and verify identical to original

## Further Reading

- [URDF Official Specification](http://wiki.ros.org/urdf/XML)
- [Xacro Documentation](http://wiki.ros.org/xacro)
- [Gazebo ROS 2 Plugins](https://github.com/ros-simulation/gazebo_ros_pkgs/wiki)
- [SolidWorks to URDF Exporter](http://wiki.ros.org/sw_urdf_exporter)
- [Inertia Calculator Tool](https://inertialtensor.readthedocs.io/)

---

**Next Chapter**: [Navigation with Nav2 →](ch06-nav2-basics.md)

**Previous Chapter**: [← TF2 Coordinate Transforms](../week-04/ch04-tf2-transforms.md)
