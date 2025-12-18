---
id: ch17-humanoid-urdf
title: Humanoid Robot Modeling
sidebar_label: Humanoid Robot Modeling
sidebar_position: 18
---

# Humanoid Robot Modeling

## Learning Objectives

By the end of this chapter, you will be able to:

- Design and implement URDF models for humanoid robots with complex kinematic structures
- Configure kinematic chains for arms, legs, torso, and head with proper joint hierarchies
- Define joint limits, dynamics, and friction parameters for realistic humanoid motion
- Calculate and verify center of mass properties for balance and stability analysis
- Apply balance constraints and zero-moment point (ZMP) principles for bipedal locomotion

## Introduction

Humanoid robots represent one of the most challenging applications of robotic modeling due to their complex kinematic structures, high degrees of freedom, and strict balance requirements. Unlike wheeled or tracked robots, humanoid robots must maintain dynamic balance while walking, manipulating objects, and interacting with human environments. The Unified Robot Description Format (URDF) provides the foundation for modeling these sophisticated systems, enabling simulation, motion planning, and control algorithm development.

Modeling a humanoid robot requires careful attention to anatomical structure, mass distribution, joint constraints, and stability principles. A typical humanoid has 20-40 degrees of freedom distributed across the torso, arms, legs, and head. Each joint must be precisely characterized with its range of motion, velocity limits, effort constraints, and friction properties. The mass and inertia properties of each link directly affect the robot's center of mass, which is critical for maintaining balance during locomotion and manipulation tasks.

This chapter builds on URDF fundamentals from Chapter 5 and sensor modeling from Chapter 8, extending these concepts to humanoid-specific challenges. We'll explore how to structure kinematic chains for bipedal locomotion, configure realistic joint dynamics, compute center of mass properties, and apply balance constraints based on zero-moment point theory. These skills are essential for developing Vision-Language-Action (VLA) models that can control humanoid robots in real-world environments.

## 1. Humanoid URDF Structure

### Hierarchical Kinematic Architecture

A humanoid robot URDF follows a hierarchical tree structure with the torso (or pelvis) typically serving as the root link. From this root, kinematic chains branch out to the head, left arm, right arm, left leg, and right leg. This design mirrors human anatomy and simplifies motion planning by treating limbs as independent chains when appropriate. The root link is often connected to a virtual `base_link` or `world` frame through a floating base joint (6-DOF) to allow the robot to move freely in space.

The standard kinematic hierarchy for a humanoid robot:

```
base_link (floating base)
└── pelvis
    ├── torso
    │   ├── head_chain
    │   ├── left_arm_chain
    │   └── right_arm_chain
    ├── left_leg_chain
    └── right_leg_chain
```

Each chain consists of links connected by joints. For example, a typical arm chain includes: shoulder_pitch, shoulder_roll, shoulder_yaw, elbow, wrist_pitch, wrist_roll, and gripper joints. A leg chain includes: hip_yaw, hip_roll, hip_pitch, knee, ankle_pitch, and ankle_roll joints. The exact structure depends on the specific humanoid design but generally follows biological joint arrangements.

### Link Properties for Humanoid Robots

Each link in a humanoid URDF must define three essential properties: visual geometry (for rendering), collision geometry (for physics simulation), and inertial properties (mass, center of mass offset, inertia tensor). For humanoid robots, accurate inertial properties are critical because they directly affect balance calculations. Underestimating limb masses or incorrectly positioning centers of mass can lead to unstable walking gaits or tipping.

Example torso link with realistic properties:

```xml
&lt;link name="torso"&gt;
  &lt;visual&gt;
    &lt;origin xyz="0 0 0.2" rpy="0 0 0"/&gt;
    &lt;geometry&gt;
      &lt;box size="0.3 0.2 0.4"/&gt;
    &lt;/geometry&gt;
    &lt;material name="body_color"&gt;
      &lt;color rgba="0.8 0.8 0.8 1.0"/&gt;
    &lt;/material&gt;
  &lt;/visual&gt;

  &lt;collision&gt;
    &lt;origin xyz="0 0 0.2" rpy="0 0 0"/&gt;
    &lt;geometry&gt;
      &lt;box size="0.3 0.2 0.4"/&gt;
    &lt;/geometry&gt;
  &lt;/collision&gt;

  &lt;inertial&gt;
    &lt;origin xyz="0 0 0.2" rpy="0 0 0"/&gt;
    &lt;mass value="8.0"/&gt;
    &lt;inertia ixx="0.16" ixy="0" ixz="0"
             iyy="0.20" iyz="0"
             izz="0.08"/&gt;
  &lt;/inertial&gt;
&lt;/link&gt;
```

The inertia tensor values are calculated based on the link's geometry and mass distribution. For a box, the moments of inertia are: Ixx = (1/12)m(h² + d²), Iyy = (1/12)m(w² + d²), Izz = (1/12)m(w² + h²), where m is mass, w is width, h is height, and d is depth.

### Material and Friction Properties

Humanoid robots often require contact modeling for foot-ground interaction, hand-object manipulation, and collision handling. URDF allows specification of material properties through Gazebo-specific tags that define coefficients of friction, restitution, and contact stiffness. These properties affect how the robot interacts with its environment during simulation.

Example foot link with contact properties:

```xml
&lt;gazebo reference="left_foot"&gt;
  &lt;mu1&gt;1.0&lt;/mu1&gt;  &lt;!-- Friction coefficient direction 1 --&gt;
  &lt;mu2&gt;1.0&lt;/mu2&gt;  &lt;!-- Friction coefficient direction 2 --&gt;
  &lt;kp&gt;1000000.0&lt;/kp&gt;  &lt;!-- Contact stiffness --&gt;
  &lt;kd&gt;100.0&lt;/kd&gt;  &lt;!-- Contact damping --&gt;
  &lt;minDepth&gt;0.001&lt;/minDepth&gt;  &lt;!-- Minimum penetration depth --&gt;
  &lt;maxVel&gt;0.01&lt;/maxVel&gt;  &lt;!-- Maximum contact correction velocity --&gt;
  &lt;material&gt;Gazebo/Grey&lt;/material&gt;
&lt;/gazebo&gt;
```

High friction coefficients (mu1, mu2 ≈ 1.0) prevent foot slippage during walking. Contact stiffness (kp) and damping (kd) values affect how the simulator resolves penetration between the foot and ground. Tuning these parameters is essential for stable bipedal locomotion in simulation.

## 2. Kinematic Chains

### Serial Kinematic Chains for Limbs

Each limb of a humanoid robot is modeled as a serial kinematic chain—a sequence of links connected by joints where each joint has a single parent and (at most) a single child. Serial chains simplify forward kinematics calculations and motion planning. For humanoid robots, the four primary chains are the left arm, right arm, left leg, and right leg, each with 6-7 degrees of freedom.

Example left arm kinematic chain:

```xml
&lt;!-- Shoulder joint (3 DOF: pitch, roll, yaw) --&gt;
&lt;joint name="left_shoulder_pitch" type="revolute"&gt;
  &lt;parent link="torso"/&gt;
  &lt;child link="left_upper_arm"/&gt;
  &lt;origin xyz="0 0.15 0.35" rpy="0 0 0"/&gt;
  &lt;axis xyz="0 1 0"/&gt;
  &lt;limit lower="-3.14" upper="3.14" effort="100" velocity="2.0"/&gt;
&lt;/joint&gt;

&lt;joint name="left_shoulder_roll" type="revolute"&gt;
  &lt;parent link="left_upper_arm"/&gt;
  &lt;child link="left_upper_arm_roll"/&gt;
  &lt;origin xyz="0 0 0" rpy="0 0 0"/&gt;
  &lt;axis xyz="1 0 0"/&gt;
  &lt;limit lower="-1.57" upper="1.57" effort="100" velocity="2.0"/&gt;
&lt;/joint&gt;

&lt;!-- Elbow joint (1 DOF) --&gt;
&lt;joint name="left_elbow" type="revolute"&gt;
  &lt;parent link="left_upper_arm_roll"/&gt;
  &lt;child link="left_forearm"/&gt;
  &lt;origin xyz="0 0 -0.25" rpy="0 0 0"/&gt;
  &lt;axis xyz="0 1 0"/&gt;
  &lt;limit lower="0" upper="2.35" effort="80" velocity="2.0"/&gt;
&lt;/joint&gt;

&lt;!-- Wrist joint (2 DOF: pitch, roll) --&gt;
&lt;joint name="left_wrist_pitch" type="revolute"&gt;
  &lt;parent link="left_forearm"/&gt;
  &lt;child link="left_hand"/&gt;
  &lt;origin xyz="0 0 -0.25" rpy="0 0 0"/&gt;
  &lt;axis xyz="0 1 0"/&gt;
  &lt;limit lower="-1.57" upper="1.57" effort="50" velocity="2.0"/&gt;
&lt;/joint&gt;
```

This chain provides 6 degrees of freedom for the arm: 3 at the shoulder (pitch, roll, yaw), 1 at the elbow, and 2 at the wrist. The `origin` elements define the spatial relationship between parent and child frames, while the `axis` elements specify the rotation axis for each joint in the parent frame.

### Leg Kinematic Chains for Bipedal Locomotion

Leg chains are critical for bipedal locomotion and must support the robot's full weight while providing mobility. A typical humanoid leg has 6 DOF: 3 at the hip (yaw, roll, pitch), 1 at the knee, and 2 at the ankle (pitch, roll). This configuration allows the robot to lift its foot, swing the leg forward, and place the foot with proper orientation.

Example left leg kinematic chain:

```xml
&lt;!-- Hip joint (3 DOF) --&gt;
&lt;joint name="left_hip_yaw" type="revolute"&gt;
  &lt;parent link="pelvis"/&gt;
  &lt;child link="left_hip_yaw_link"/&gt;
  &lt;origin xyz="0 0.1 0" rpy="0 0 0"/&gt;
  &lt;axis xyz="0 0 1"/&gt;
  &lt;limit lower="-0.785" upper="0.785" effort="150" velocity="1.5"/&gt;
&lt;/joint&gt;

&lt;joint name="left_hip_roll" type="revolute"&gt;
  &lt;parent link="left_hip_yaw_link"/&gt;
  &lt;child link="left_hip_roll_link"/&gt;
  &lt;origin xyz="0 0 0" rpy="0 0 0"/&gt;
  &lt;axis xyz="1 0 0"/&gt;
  &lt;limit lower="-0.523" upper="0.523" effort="150" velocity="1.5"/&gt;
&lt;/joint&gt;

&lt;joint name="left_hip_pitch" type="revolute"&gt;
  &lt;parent link="left_hip_roll_link"/&gt;
  &lt;child link="left_thigh"/&gt;
  &lt;origin xyz="0 0 0" rpy="0 0 0"/&gt;
  &lt;axis xyz="0 1 0"/&gt;
  &lt;limit lower="-1.57" upper="1.57" effort="150" velocity="1.5"/&gt;
&lt;/joint&gt;

&lt;!-- Knee joint (1 DOF) --&gt;
&lt;joint name="left_knee" type="revolute"&gt;
  &lt;parent link="left_thigh"/&gt;
  &lt;child link="left_shin"/&gt;
  &lt;origin xyz="0 0 -0.35" rpy="0 0 0"/&gt;
  &lt;axis xyz="0 1 0"/&gt;
  &lt;limit lower="0" upper="2.35" effort="150" velocity="2.0"/&gt;
&lt;/joint&gt;

&lt;!-- Ankle joint (2 DOF) --&gt;
&lt;joint name="left_ankle_pitch" type="revolute"&gt;
  &lt;parent link="left_shin"/&gt;
  &lt;child link="left_foot"/&gt;
  &lt;origin xyz="0 0 -0.35" rpy="0 0 0"/&gt;
  &lt;axis xyz="0 1 0"/&gt;
  &lt;limit lower="-0.785" upper="0.785" effort="100" velocity="2.0"/&gt;
&lt;/joint&gt;
```

The leg chain's joint limits are carefully chosen to match human biomechanics while preventing singularities and self-collisions. The knee joint, for example, only allows flexion (0 to 135 degrees) and prevents hyperextension.

### Forward and Inverse Kinematics

Forward kinematics (FK) computes the end-effector pose (position and orientation) given joint angles. For a serial chain with n joints, FK involves multiplying transformation matrices from the base to the tip. In ROS 2, the `robot_state_publisher` package performs FK automatically by listening to joint states and publishing TF transforms.

Inverse kinematics (IK) solves the opposite problem: finding joint angles that achieve a desired end-effector pose. IK is essential for motion planning tasks like reaching for objects or positioning the foot during walking. For humanoid robots, IK is often solved using numerical methods (Jacobian-based, optimization-based) or analytical solutions for specific kinematic structures. Libraries like MoveIt 2 provide IK solvers that integrate with URDF models.

## 3. Joint Limits and Dynamics

### Position, Velocity, and Effort Limits

Every revolute and prismatic joint in a humanoid URDF must specify three types of limits: position (angular or linear range), velocity (maximum speed), and effort (maximum force or torque). These limits ensure the robot operates within safe mechanical constraints and prevent damage to actuators and linkages.

```xml
&lt;joint name="left_knee" type="revolute"&gt;
  &lt;parent link="left_thigh"/&gt;
  &lt;child link="left_shin"/&gt;
  &lt;origin xyz="0 0 -0.35" rpy="0 0 0"/&gt;
  &lt;axis xyz="0 1 0"/&gt;
  &lt;limit lower="0.0" upper="2.35" effort="150.0" velocity="2.0"/&gt;
  &lt;dynamics damping="0.5" friction="0.1"/&gt;
&lt;/joint&gt;
```

- **Position limits** (lower, upper): Define the joint's range of motion in radians (revolute) or meters (prismatic). For the knee, 0 to 2.35 rad (0° to 135°) allows flexion without hyperextension.
- **Velocity limit**: Maximum angular or linear velocity in rad/s or m/s. A 2.0 rad/s limit allows the knee to bend from fully extended to fully flexed in ~1.2 seconds.
- **Effort limit**: Maximum torque (N⋅m) or force (N) the actuator can exert. A 150 N⋅m knee torque supports body weight during walking.

### Joint Dynamics: Damping and Friction

The `dynamics` tag specifies damping and friction coefficients that model energy dissipation in the joint. Damping opposes motion proportional to velocity (viscous damping), while friction represents static and kinetic resistance. These parameters affect simulation realism and control performance.

- **Damping** (kg⋅m²/s for revolute, kg/s for prismatic): Models viscous resistance. Higher damping reduces oscillations but requires more control effort. Typical values: 0.1-1.0 for small joints, 0.5-5.0 for large joints.
- **Friction** (N⋅m for revolute, N for prismatic): Models static and kinetic friction. Affects startup torque and steady-state control. Typical values: 0.01-0.5 for well-lubricated joints.

Example leg joint with realistic dynamics:

```xml
&lt;joint name="left_hip_pitch" type="revolute"&gt;
  &lt;parent link="left_hip_roll_link"/&gt;
  &lt;child link="left_thigh"/&gt;
  &lt;origin xyz="0 0 0" rpy="0 0 0"/&gt;
  &lt;axis xyz="0 1 0"/&gt;
  &lt;limit lower="-1.57" upper="1.57" effort="150.0" velocity="1.5"/&gt;
  &lt;dynamics damping="1.0" friction="0.2"/&gt;
&lt;/joint&gt;
```

Higher damping and friction values at the hip joints reflect the mechanical complexity and weight-bearing requirements of these actuators.

### Transmission and Actuator Modeling

For humanoid robots with complex actuation systems (geared motors, series elastic actuators, hydraulic actuators), the URDF can include `transmission` elements that describe the mechanical relationship between actuators and joints. This is particularly important for robots with gear reduction or cable-driven joints.

Example transmission for a geared joint:

```xml
&lt;transmission name="left_knee_transmission"&gt;
  &lt;type&gt;transmission_interface/SimpleTransmission&lt;/type&gt;
  &lt;joint name="left_knee"&gt;
    &lt;hardwareInterface&gt;hardware_interface/EffortJointInterface&lt;/hardwareInterface&gt;
  &lt;/joint&gt;
  &lt;actuator name="left_knee_motor"&gt;
    &lt;mechanicalReduction&gt;100&lt;/mechanicalReduction&gt;
    &lt;hardwareInterface&gt;hardware_interface/EffortJointInterface&lt;/hardwareInterface&gt;
  &lt;/actuator&gt;
&lt;/transmission&gt;
```

The `mechanicalReduction` of 100 indicates a 100:1 gear ratio, meaning the motor rotates 100 times for each joint rotation. This increases torque at the cost of speed.

## 4. Center of Mass

### Calculating System Center of Mass

The center of mass (CoM) of a humanoid robot is the weighted average position of all link masses. For a robot with n links, the CoM position is:

**CoM = (Σᵢ mᵢ × pᵢ) / Σᵢ mᵢ**

where mᵢ is the mass of link i and pᵢ is the position of its center of mass in the world frame. The CoM position changes dynamically as the robot moves its limbs, making it critical for balance control.

In ROS 2, you can compute the CoM using the URDF model and current joint states:

```python
import numpy as np
from urdf_parser_py.urdf import URDF

class CoMCalculator:
    def __init__(self, urdf_path):
        self.robot = URDF.from_xml_file(urdf_path)
        self.link_masses = {}
        self.link_com_offsets = {}

        for link in self.robot.links:
            if link.inertial:
                self.link_masses[link.name] = link.inertial.mass
                self.link_com_offsets[link.name] = np.array(
                    link.inertial.origin.xyz
                )

    def compute_com(self, link_transforms):
        """
        Compute system center of mass given link transforms.

        Args:
            link_transforms: dict mapping link names to 4x4 transform matrices

        Returns:
            3D CoM position in world frame
        """
        total_mass = 0.0
        weighted_position = np.zeros(3)

        for link_name, mass in self.link_masses.items():
            if link_name in link_transforms:
                T = link_transforms[link_name]
                # Transform link CoM offset to world frame
                com_offset_homogeneous = np.append(
                    self.link_com_offsets[link_name], 1
                )
                com_world = T @ com_offset_homogeneous

                weighted_position += mass * com_world[:3]
                total_mass += mass

        return weighted_position / total_mass if total_mass &gt; 0 else np.zeros(3)
```

This calculator reads inertial properties from the URDF and computes the CoM given the current link transforms (obtained from TF2).

### CoM Projection and Support Polygon

For bipedal balance, the projection of the CoM onto the ground plane must lie within the support polygon—the convex hull of all ground contact points. During double support (both feet on the ground), the support polygon is the quadrilateral formed by the two feet. During single support, it's the contact area of the stance foot.

Example support polygon calculation for double support:

```python
from scipy.spatial import ConvexHull

def compute_support_polygon(left_foot_contacts, right_foot_contacts):
    """
    Compute support polygon from foot contact points.

    Args:
        left_foot_contacts: Nx2 array of left foot contact points (x, y)
        right_foot_contacts: Mx2 array of right foot contact points (x, y)

    Returns:
        Convex hull representing support polygon
    """
    all_contacts = np.vstack([left_foot_contacts, right_foot_contacts])
    hull = ConvexHull(all_contacts)
    return hull

def is_com_stable(com_position, support_polygon):
    """Check if CoM projection is inside support polygon."""
    from matplotlib.path import Path
    polygon_path = Path(support_polygon.points[support_polygon.vertices])
    return polygon_path.contains_point(com_position[:2])
```

If the CoM projection falls outside the support polygon, the robot will tip unless corrective action is taken (stepping, arm swinging, ankle torque).

### Dynamic CoM Trajectory Planning

During walking, the CoM trajectory must be planned to ensure continuous stability. A common approach is to use a linear inverted pendulum model (LIPM) where the CoM is treated as a point mass above the zero-moment point (ZMP). The LIPM dynamics are:

**ẍ = ω² (x - p)**

where x is the CoM horizontal position, p is the ZMP position, and ω = √(g/h) with g = 9.81 m/s² and h the CoM height. By controlling the ZMP position, the controller can influence CoM acceleration and maintain balance.

## 5. Balance Constraints

### Zero-Moment Point (ZMP) Theory

The Zero-Moment Point is the point on the ground where the net moment from ground reaction forces is zero. For a humanoid to maintain balance without tipping, the ZMP must remain inside the support polygon. The ZMP position is calculated from the robot's dynamics:

**pzmp = (Σᵢ mᵢ(ẍᵢ - g)(zᵢ - zground)) / (Σᵢ mᵢ(z̈ᵢ - g))**

where the sum is over all robot links, g is gravitational acceleration, and zground is the ground height. In practice, the ZMP is computed from the CoM trajectory and used as a feedback signal for balance control.

Example ZMP computation for a planar humanoid:

```python
def compute_zmp(com_position, com_acceleration, total_mass, com_height, g=9.81):
    """
    Compute ZMP position for a planar humanoid.

    Args:
        com_position: 2D CoM position [x, y]
        com_acceleration: 2D CoM acceleration [ax, ay]
        total_mass: Total robot mass (kg)
        com_height: CoM height above ground (m)
        g: Gravitational acceleration (m/s²)

    Returns:
        2D ZMP position [px, py]
    """
    zmp_x = com_position[0] - (com_height / g) * com_acceleration[0]
    zmp_y = com_position[1] - (com_height / g) * com_acceleration[1]
    return np.array([zmp_x, zmp_y])
```

ZMP-based controllers adjust the robot's joint trajectories to keep the ZMP inside the support polygon, preventing falls.

### Static vs. Dynamic Balance

**Static balance** occurs when the robot is stationary or moving slowly enough that inertial effects are negligible. The CoM projection must be inside the support polygon. Static balance is simpler to achieve but limits mobility.

**Dynamic balance** allows for faster motion and requires considering momentum and angular momentum. Even if the CoM momentarily exits the support polygon, the robot can remain upright if it's accelerating back toward stability (e.g., during running or jumping). Dynamic walking gaits use controlled falls where the robot briefly loses static balance but regains it with each step.

### Compliance and Ankle Strategies

Humanoid robots use several strategies to maintain balance:

1. **Ankle strategy**: Apply torques at the ankle joints to shift the CoM without moving the feet. Effective for small disturbances when both feet are planted.

2. **Hip strategy**: Rotate the torso at the hips to adjust CoM position. More powerful than ankle strategy but requires more coordination.

3. **Stepping strategy**: Move the feet to reposition the support polygon under the CoM. Used for large disturbances or planned walking.

4. **Compliance control**: Use compliant actuators or controllers that allow some joint deflection to absorb impacts and disturbances. Series elastic actuators (SEAs) and impedance control are common approaches.

Example ankle strategy controller:

```python
class AnkleBalanceController:
    def __init__(self, kp=500.0, kd=50.0):
        self.kp = kp  # Proportional gain
        self.kd = kd  # Derivative gain

    def compute_ankle_torque(self, com_error, com_velocity):
        """
        Compute ankle torque to correct CoM position.

        Args:
            com_error: Difference between desired and actual CoM position (m)
            com_velocity: CoM velocity (m/s)

        Returns:
            Ankle torque (N⋅m)
        """
        return self.kp * com_error + self.kd * com_velocity
```

This PD controller generates ankle torques proportional to the CoM position error and velocity, damping oscillations while correcting balance.

## Practical Example

Let's build a simplified humanoid URDF with legs, torso, and CoM calculation:

```xml
&lt;?xml version="1.0"?&gt;
&lt;robot name="simple_humanoid"&gt;

  &lt;!-- Base link (floating base) --&gt;
  &lt;link name="base_link"/&gt;

  &lt;!-- Pelvis link --&gt;
  &lt;link name="pelvis"&gt;
    &lt;visual&gt;
      &lt;geometry&gt;
        &lt;box size="0.3 0.2 0.15"/&gt;
      &lt;/geometry&gt;
      &lt;material name="grey"&gt;
        &lt;color rgba="0.5 0.5 0.5 1.0"/&gt;
      &lt;/material&gt;
    &lt;/visual&gt;
    &lt;collision&gt;
      &lt;geometry&gt;
        &lt;box size="0.3 0.2 0.15"/&gt;
      &lt;/geometry&gt;
    &lt;/collision&gt;
    &lt;inertial&gt;
      &lt;mass value="5.0"/&gt;
      &lt;inertia ixx="0.015" ixy="0" ixz="0"
               iyy="0.020" iyz="0"
               izz="0.016"/&gt;
    &lt;/inertial&gt;
  &lt;/link&gt;

  &lt;joint name="base_to_pelvis" type="fixed"&gt;
    &lt;parent link="base_link"/&gt;
    &lt;child link="pelvis"/&gt;
    &lt;origin xyz="0 0 0.8" rpy="0 0 0"/&gt;
  &lt;/joint&gt;

  &lt;!-- Torso link --&gt;
  &lt;link name="torso"&gt;
    &lt;visual&gt;
      &lt;origin xyz="0 0 0.25" rpy="0 0 0"/&gt;
      &lt;geometry&gt;
        &lt;box size="0.35 0.25 0.5"/&gt;
      &lt;/geometry&gt;
      &lt;material name="grey"/&gt;
    &lt;/visual&gt;
    &lt;collision&gt;
      &lt;origin xyz="0 0 0.25" rpy="0 0 0"/&gt;
      &lt;geometry&gt;
        &lt;box size="0.35 0.25 0.5"/&gt;
      &lt;/geometry&gt;
    &lt;/collision&gt;
    &lt;inertial&gt;
      &lt;origin xyz="0 0 0.25" rpy="0 0 0"/&gt;
      &lt;mass value="10.0"/&gt;
      &lt;inertia ixx="0.25" ixy="0" ixz="0"
               iyy="0.30" iyz="0"
               izz="0.12"/&gt;
    &lt;/inertial&gt;
  &lt;/link&gt;

  &lt;joint name="torso_joint" type="fixed"&gt;
    &lt;parent link="pelvis"/&gt;
    &lt;child link="torso"/&gt;
    &lt;origin xyz="0 0 0.1" rpy="0 0 0"/&gt;
  &lt;/joint&gt;

  &lt;!-- Left leg chain --&gt;
  &lt;link name="left_thigh"&gt;
    &lt;visual&gt;
      &lt;origin xyz="0 0 -0.2" rpy="0 0 0"/&gt;
      &lt;geometry&gt;
        &lt;cylinder radius="0.05" length="0.4"/&gt;
      &lt;/geometry&gt;
      &lt;material name="grey"/&gt;
    &lt;/visual&gt;
    &lt;collision&gt;
      &lt;origin xyz="0 0 -0.2" rpy="0 0 0"/&gt;
      &lt;geometry&gt;
        &lt;cylinder radius="0.05" length="0.4"/&gt;
      &lt;/geometry&gt;
    &lt;/collision&gt;
    &lt;inertial&gt;
      &lt;origin xyz="0 0 -0.2" rpy="0 0 0"/&gt;
      &lt;mass value="3.0"/&gt;
      &lt;inertia ixx="0.042" ixy="0" ixz="0"
               iyy="0.042" iyz="0"
               izz="0.004"/&gt;
    &lt;/inertial&gt;
  &lt;/link&gt;

  &lt;joint name="left_hip_pitch" type="revolute"&gt;
    &lt;parent link="pelvis"/&gt;
    &lt;child link="left_thigh"/&gt;
    &lt;origin xyz="0 0.1 -0.05" rpy="0 0 0"/&gt;
    &lt;axis xyz="0 1 0"/&gt;
    &lt;limit lower="-1.57" upper="1.57" effort="150" velocity="1.5"/&gt;
    &lt;dynamics damping="1.0" friction="0.2"/&gt;
  &lt;/joint&gt;

  &lt;link name="left_shin"&gt;
    &lt;visual&gt;
      &lt;origin xyz="0 0 -0.2" rpy="0 0 0"/&gt;
      &lt;geometry&gt;
        &lt;cylinder radius="0.04" length="0.4"/&gt;
      &lt;/geometry&gt;
      &lt;material name="grey"/&gt;
    &lt;/visual&gt;
    &lt;collision&gt;
      &lt;origin xyz="0 0 -0.2" rpy="0 0 0"/&gt;
      &lt;geometry&gt;
        &lt;cylinder radius="0.04" length="0.4"/&gt;
      &lt;/geometry&gt;
    &lt;/collision&gt;
    &lt;inertial&gt;
      &lt;origin xyz="0 0 -0.2" rpy="0 0 0"/&gt;
      &lt;mass value="2.0"/&gt;
      &lt;inertia ixx="0.028" ixy="0" ixz="0"
               iyy="0.028" iyz="0"
               izz="0.002"/&gt;
    &lt;/inertial&gt;
  &lt;/link&gt;

  &lt;joint name="left_knee" type="revolute"&gt;
    &lt;parent link="left_thigh"/&gt;
    &lt;child link="left_shin"/&gt;
    &lt;origin xyz="0 0 -0.4" rpy="0 0 0"/&gt;
    &lt;axis xyz="0 1 0"/&gt;
    &lt;limit lower="0" upper="2.35" effort="150" velocity="2.0"/&gt;
    &lt;dynamics damping="0.5" friction="0.1"/&gt;
  &lt;/joint&gt;

  &lt;link name="left_foot"&gt;
    &lt;visual&gt;
      &lt;origin xyz="0.05 0 -0.025" rpy="0 0 0"/&gt;
      &lt;geometry&gt;
        &lt;box size="0.2 0.1 0.05"/&gt;
      &lt;/geometry&gt;
      &lt;material name="grey"/&gt;
    &lt;/visual&gt;
    &lt;collision&gt;
      &lt;origin xyz="0.05 0 -0.025" rpy="0 0 0"/&gt;
      &lt;geometry&gt;
        &lt;box size="0.2 0.1 0.05"/&gt;
      &lt;/geometry&gt;
    &lt;/collision&gt;
    &lt;inertial&gt;
      &lt;origin xyz="0.05 0 -0.025" rpy="0 0 0"/&gt;
      &lt;mass value="1.0"/&gt;
      &lt;inertia ixx="0.001" ixy="0" ixz="0"
               iyy="0.004" iyz="0"
               izz="0.004"/&gt;
    &lt;/inertial&gt;
  &lt;/link&gt;

  &lt;joint name="left_ankle_pitch" type="revolute"&gt;
    &lt;parent link="left_shin"/&gt;
    &lt;child link="left_foot"/&gt;
    &lt;origin xyz="0 0 -0.4" rpy="0 0 0"/&gt;
    &lt;axis xyz="0 1 0"/&gt;
    &lt;limit lower="-0.785" upper="0.785" effort="100" velocity="2.0"/&gt;
    &lt;dynamics damping="0.5" friction="0.1"/&gt;
  &lt;/joint&gt;

  &lt;!-- Right leg chain (mirrored) --&gt;
  &lt;link name="right_thigh"&gt;
    &lt;visual&gt;
      &lt;origin xyz="0 0 -0.2" rpy="0 0 0"/&gt;
      &lt;geometry&gt;
        &lt;cylinder radius="0.05" length="0.4"/&gt;
      &lt;/geometry&gt;
      &lt;material name="grey"/&gt;
    &lt;/visual&gt;
    &lt;collision&gt;
      &lt;origin xyz="0 0 -0.2" rpy="0 0 0"/&gt;
      &lt;geometry&gt;
        &lt;cylinder radius="0.05" length="0.4"/&gt;
      &lt;/geometry&gt;
    &lt;/collision&gt;
    &lt;inertial&gt;
      &lt;origin xyz="0 0 -0.2" rpy="0 0 0"/&gt;
      &lt;mass value="3.0"/&gt;
      &lt;inertia ixx="0.042" ixy="0" ixz="0"
               iyy="0.042" iyz="0"
               izz="0.004"/&gt;
    &lt;/inertial&gt;
  &lt;/link&gt;

  &lt;joint name="right_hip_pitch" type="revolute"&gt;
    &lt;parent link="pelvis"/&gt;
    &lt;child link="right_thigh"/&gt;
    &lt;origin xyz="0 -0.1 -0.05" rpy="0 0 0"/&gt;
    &lt;axis xyz="0 1 0"/&gt;
    &lt;limit lower="-1.57" upper="1.57" effort="150" velocity="1.5"/&gt;
    &lt;dynamics damping="1.0" friction="0.2"/&gt;
  &lt;/joint&gt;

  &lt;link name="right_shin"&gt;
    &lt;visual&gt;
      &lt;origin xyz="0 0 -0.2" rpy="0 0 0"/&gt;
      &lt;geometry&gt;
        &lt;cylinder radius="0.04" length="0.4"/&gt;
      &lt;/geometry&gt;
      &lt;material name="grey"/&gt;
    &lt;/visual&gt;
    &lt;collision&gt;
      &lt;origin xyz="0 0 -0.2" rpy="0 0 0"/&gt;
      &lt;geometry&gt;
        &lt;cylinder radius="0.04" length="0.4"/&gt;
      &lt;/geometry&gt;
    &lt;/collision&gt;
    &lt;inertial&gt;
      &lt;origin xyz="0 0 -0.2" rpy="0 0 0"/&gt;
      &lt;mass value="2.0"/&gt;
      &lt;inertia ixx="0.028" ixy="0" ixz="0"
               iyy="0.028" iyz="0"
               izz="0.002"/&gt;
    &lt;/inertial&gt;
  &lt;/link&gt;

  &lt;joint name="right_knee" type="revolute"&gt;
    &lt;parent link="right_thigh"/&gt;
    &lt;child link="right_shin"/&gt;
    &lt;origin xyz="0 0 -0.4" rpy="0 0 0"/&gt;
    &lt;axis xyz="0 1 0"/&gt;
    &lt;limit lower="0" upper="2.35" effort="150" velocity="2.0"/&gt;
    &lt;dynamics damping="0.5" friction="0.1"/&gt;
  &lt;/joint&gt;

  &lt;link name="right_foot"&gt;
    &lt;visual&gt;
      &lt;origin xyz="0.05 0 -0.025" rpy="0 0 0"/&gt;
      &lt;geometry&gt;
        &lt;box size="0.2 0.1 0.05"/&gt;
      &lt;/geometry&gt;
      &lt;material name="grey"/&gt;
    &lt;/visual&gt;
    &lt;collision&gt;
      &lt;origin xyz="0.05 0 -0.025" rpy="0 0 0"/&gt;
      &lt;geometry&gt;
        &lt;box size="0.2 0.1 0.05"/&gt;
      &lt;/geometry&gt;
    &lt;/collision&gt;
    &lt;inertial&gt;
      &lt;origin xyz="0.05 0 -0.025" rpy="0 0 0"/&gt;
      &lt;mass value="1.0"/&gt;
      &lt;inertia ixx="0.001" ixy="0" ixz="0"
               iyy="0.004" iyz="0"
               izz="0.004"/&gt;
    &lt;/inertial&gt;
  &lt;/link&gt;

  &lt;joint name="right_ankle_pitch" type="revolute"&gt;
    &lt;parent link="right_shin"/&gt;
    &lt;child link="right_foot"/&gt;
    &lt;origin xyz="0 0 -0.4" rpy="0 0 0"/&gt;
    &lt;axis xyz="0 1 0"/&gt;
    &lt;limit lower="-0.785" upper="0.785" effort="100" velocity="2.0"/&gt;
    &lt;dynamics damping="0.5" friction="0.1"/&gt;
  &lt;/joint&gt;

  &lt;!-- Gazebo foot contact properties --&gt;
  &lt;gazebo reference="left_foot"&gt;
    &lt;mu1&gt;1.0&lt;/mu1&gt;
    &lt;mu2&gt;1.0&lt;/mu2&gt;
    &lt;kp&gt;1000000.0&lt;/kp&gt;
    &lt;kd&gt;100.0&lt;/kd&gt;
    &lt;minDepth&gt;0.001&lt;/minDepth&gt;
    &lt;material&gt;Gazebo/Grey&lt;/material&gt;
  &lt;/gazebo&gt;

  &lt;gazebo reference="right_foot"&gt;
    &lt;mu1&gt;1.0&lt;/mu1&gt;
    &lt;mu2&gt;1.0&lt;/mu2&gt;
    &lt;kp&gt;1000000.0&lt;/kp&gt;
    &lt;kd&gt;100.0&lt;/kd&gt;
    &lt;minDepth&gt;0.001&lt;/minDepth&gt;
    &lt;material&gt;Gazebo/Grey&lt;/material&gt;
  &lt;/gazebo&gt;

&lt;/robot&gt;
```

**Key Implementation Points:**
- The humanoid has a total mass of ~27 kg distributed across pelvis (5 kg), torso (10 kg), thighs (6 kg), shins (4 kg), and feet (2 kg)
- Joint limits match human biomechanics: hip pitch ±90°, knee 0-135°, ankle pitch ±45°
- Foot contact properties (high friction, stiffness) prevent slippage during walking
- The CoM height is approximately 0.8 m (pelvis) + 0.35 m (torso/legs) = 0.6-0.7 m depending on joint configuration

## Common Challenges and Solutions

### Challenge 1: Unstable Simulation Due to High Contact Stiffness

**Problem**: When simulating humanoid robots in Gazebo, overly high contact stiffness (kp) values can cause numerical instability, vibration, or jittering when the feet are on the ground.

**Solution**: Reduce contact stiffness to 1e5-1e6 range and increase contact damping (kd) to 10-100. Also reduce the simulation timestep (e.g., from 1 ms to 0.5 ms) to improve numerical stability. In Gazebo, set:

```xml
&lt;gazebo reference="foot_link"&gt;
  &lt;kp&gt;100000.0&lt;/kp&gt;
  &lt;kd&gt;10.0&lt;/kd&gt;
&lt;/gazebo&gt;
```

And in your world file:
```xml
&lt;physics type="ode"&gt;
  &lt;max_step_size&gt;0.0005&lt;/max_step_size&gt;
  &lt;real_time_update_rate&gt;2000&lt;/real_time_update_rate&gt;
&lt;/physics&gt;
```

### Challenge 2: Incorrect Center of Mass Causing Balance Issues

**Problem**: The robot tips over unexpectedly during simulation because the URDF's inertial properties don't match the physical design, leading to an incorrect CoM position.

**Solution**: Use CAD software or physics simulation tools to compute accurate mass, center of mass, and inertia tensor for each link. For complex shapes, export mesh files and use tools like `meshlab` or Python libraries (`trimesh`, `pymesh`) to compute inertial properties. Verify the CoM position using the ROS 2 `robot_state_publisher` and a visualization tool like RViz, which can display the CoM marker.

Example using trimesh:
```python
import trimesh

mesh = trimesh.load('thigh_link.stl')
mesh.density = 1050  # kg/m³ (plastic density)
mass = mesh.mass
com = mesh.center_mass
inertia = mesh.moment_inertia

print(f"Mass: {mass} kg")
print(f"Center of Mass: {com}")
print(f"Inertia Tensor:\n{inertia}")
```

### Challenge 3: Self-Collision During Joint Motion

**Problem**: When the humanoid moves its limbs, links collide with each other (e.g., thigh intersects torso during hip flexion), causing unrealistic physics behavior or simulation crashes.

**Solution**: Define collision geometry that is simplified and slightly smaller than visual geometry. Add collision filtering in Gazebo to disable collision checking between adjacent links. In URDF, use separate `collision` tags with simplified shapes:

```xml
&lt;collision&gt;
  &lt;geometry&gt;
    &lt;!-- Use capsule or smaller cylinder instead of visual mesh --&gt;
    &lt;cylinder radius="0.045" length="0.38"/&gt;
  &lt;/geometry&gt;
&lt;/collision&gt;
```

And in Gazebo, disable specific collisions:
```xml
&lt;gazebo&gt;
  &lt;plugin name="disable_collisions" filename="libgazebo_ros_disable_collisions.so"&gt;
    &lt;link1&gt;thigh_left&lt;/link1&gt;
    &lt;link2&gt;torso&lt;/link2&gt;
  &lt;/plugin&gt;
&lt;/gazebo&gt;
```

## Best Practices

1. **Use Xacro for symmetry**: Humanoid robots have left/right symmetry. Define arm and leg chains as Xacro macros with a `reflect` parameter to automatically generate mirrored limbs, reducing duplication and errors.

2. **Validate inertial properties**: Always compute inertia tensors from CAD models or analytical formulas. Incorrect inertia values cause unrealistic dynamics, making it impossible to tune controllers effectively.

3. **Test joints independently**: Before simulating the full humanoid, test each joint chain (arm, leg) separately in Gazebo. Verify range of motion, check for self-collisions, and tune damping/friction parameters.

4. **Monitor CoM in real-time**: Add a ROS 2 node that subscribes to joint states, computes the CoM, and publishes it as a `PointStamped` message for visualization in RViz. This helps debug balance issues during development.

5. **Start with reduced DOF**: Begin with a simplified humanoid (e.g., 2-DOF legs, fixed torso) to develop walking controllers. Incrementally add joints as you validate each subsystem, rather than debugging a full 30-DOF humanoid from the start.

## Summary

Key takeaways from this chapter:

- Humanoid URDF models require hierarchical kinematic structures with carefully designed serial chains for limbs, accurate inertial properties for balance, and realistic joint limits and dynamics for motion control
- Kinematic chains define the parent-child relationships between links, enabling forward kinematics for end-effector pose computation and inverse kinematics for motion planning
- Joint limits (position, velocity, effort) and dynamics (damping, friction) must match hardware specifications and biomechanical constraints to ensure safe and realistic operation
- The center of mass (CoM) is the weighted average of all link masses and must remain above the support polygon for static balance or follow ZMP constraints for dynamic balance
- Balance control strategies include ankle torque adjustments for small disturbances, hip strategies for larger corrections, stepping to reposition the support polygon, and compliance to absorb impacts
- Accurate humanoid modeling is essential for VLA systems that must control bipedal robots in real-world manipulation and navigation tasks

## Further Reading

- **Official Documentation**: [ROS 2 URDF Tutorial](https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/URDF-Main.html) - Comprehensive guide to URDF syntax and conventions
- **Research Papers**:
  - Kajita et al. (2003), "Biped Walking Pattern Generation by using Preview Control of Zero-Moment Point" - Foundational ZMP-based walking controller
  - Pratt and Tedrake (2006), "Velocity-Based Stability Margins for Fast Bipedal Walking" - Capture point theory for dynamic balance
  - Sentis and Khatib (2005), "Synthesis of Whole-Body Behaviors through Hierarchical Control of Behavioral Primitives" - Whole-body control for humanoids
- **Tutorials**:
  - [Gazebo Humanoid Robot Tutorial](http://gazebosim.org/tutorials?tut=humanoid_teleoperation) - Simulating and controlling humanoid robots
  - [MoveIt 2 with Humanoids](https://moveit.picknik.ai/) - Motion planning for high-DOF humanoid arms
- **Community Resources**:
  - [Humanoids ROS Discourse](https://discourse.ros.org/c/humanoid) - Community discussions on humanoid robotics
  - [IEEE-RAS Humanoids Conference](https://humanoids.ieee-ras.org/) - Latest research and applications

## Review Questions

1. Explain the hierarchical kinematic structure of a humanoid robot URDF. Why is the pelvis typically chosen as the root link rather than the head or feet?

2. A humanoid robot has a total mass of 50 kg and a CoM height of 0.9 m. During walking, the CoM accelerates forward at 0.5 m/s². Calculate the ZMP position relative to the CoM. Will the robot remain balanced if the CoM is directly above the ankle (assume g = 9.81 m/s²)?

3. Compare the advantages and disadvantages of static balance versus dynamic balance for bipedal locomotion. In what scenarios would each approach be preferred?

4. A humanoid's knee joint has an effort limit of 150 N⋅m and a damping coefficient of 0.5 N⋅m⋅s/rad. If the knee is rotating at 1.0 rad/s and the controller commands maximum torque, what is the net torque available for acceleration?

5. Describe the three primary balance strategies (ankle, hip, stepping) and explain when each should be used based on the magnitude and direction of external disturbances.

6. Why is it important to define accurate inertia tensors for each link in a humanoid URDF? What simulation artifacts might occur if inertia values are incorrect?

7. Explain how the support polygon changes during the gait cycle of bipedal walking (double support → single support → double support). How does this affect CoM trajectory planning?

## Hands-On Exercise

**Exercise Title**: Building and Simulating a Simple Bipedal Walker

**Objective**: Create a URDF model for a minimal bipedal robot, simulate it in Gazebo, compute its center of mass, and implement a basic balance controller.

**Prerequisites**:
- Completed Chapters 5 (URDF Basics) and 8 (Sensor Modeling)
- ROS 2 Humble installed with Gazebo
- Python 3.10+ with NumPy and SciPy

**Steps**:
1. Use the provided `simple_humanoid.urdf` from the Practical Example section. Save it to `~/ros2_ws/src/my_humanoid/urdf/simple_humanoid.urdf`.

2. Create a launch file to spawn the robot in Gazebo and start `robot_state_publisher`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('my_humanoid')
    urdf_file = os.path.join(pkg_share, 'urdf', 'simple_humanoid.urdf')

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'simple_humanoid',
                      '-topic', 'robot_description'],
            output='screen'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('gazebo_ros'),
                '/launch/gazebo.launch.py'
            ])
        )
    ])
```

3. Create a ROS 2 node that subscribes to `/joint_states`, computes the center of mass, and publishes it as a `PointStamped` message for visualization:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PointStamped
from urdf_parser_py.urdf import URDF
import numpy as np

class CoMPublisher(Node):
    def __init__(self):
        super().__init__('com_publisher')
        self.subscription = self.create_subscription(
            JointState, 'joint_states', self.joint_state_callback, 10
        )
        self.com_publisher = self.create_publisher(
            PointStamped, 'center_of_mass', 10
        )

        # Load URDF and extract mass properties
        robot_desc = self.get_parameter('robot_description').value
        self.robot = URDF.from_xml_string(robot_desc)

        self.get_logger().info('CoM Publisher initialized')

    def joint_state_callback(self, msg):
        # Simplified: assume fixed pose, compute static CoM
        # (Full implementation would use TF2 to get link transforms)
        com = self.compute_static_com()

        com_msg = PointStamped()
        com_msg.header.stamp = self.get_clock().now().to_msg()
        com_msg.header.frame_id = 'base_link'
        com_msg.point.x = com[0]
        com_msg.point.y = com[1]
        com_msg.point.z = com[2]

        self.com_publisher.publish(com_msg)

    def compute_static_com(self):
        # Simplified calculation (assumes default joint positions)
        total_mass = 0.0
        weighted_sum = np.zeros(3)

        for link in self.robot.links:
            if link.inertial:
                mass = link.inertial.mass
                # For simplicity, use link origin as CoM position
                # (Real implementation uses forward kinematics)
                pos = np.array([0.0, 0.0, 0.8])  # Approximate
                weighted_sum += mass * pos
                total_mass += mass

        return weighted_sum / total_mass if total_mass &gt; 0 else np.zeros(3)
```

4. Launch the simulation: `ros2 launch my_humanoid spawn_humanoid.launch.py`

5. Visualize the robot and CoM in RViz. Add a `PointStamped` display subscribing to `/center_of_mass` to see the CoM marker.

6. Use `rqt_joint_trajectory_controller` or publish to joint command topics to move the legs and observe how the CoM changes.

**Expected Outcome**: You should see the bipedal robot standing in Gazebo with a marker in RViz showing its center of mass. As you move the joints, the CoM marker should update, demonstrating the dynamic nature of humanoid balance.

**Extension Challenges** (Optional):
- Implement a full forward kinematics solver using TF2 to compute accurate link transforms and CoM position
- Add an ankle balance controller that adjusts ankle joint torques to keep the CoM above the support polygon
- Create a simple walking gait by commanding sinusoidal joint trajectories and monitor ZMP stability

**Complete code available in**: `/static/code/vla/chapter17/`

---

**Previous Chapter**: [Visual SLAM and Navigation](/docs/module-03-isaac/week-09/ch13-vslam-nav2)
**Next Chapter**: [Manipulation Planning](/docs/module-04-vla/week-12/ch19-grasping)
**Module Overview**: [VLA Systems](/docs/module-04-vla/)
