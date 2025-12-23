---
id: ch18-joint-control
title: Joint-Level Control
sidebar_label: Joint-Level Control
sidebar_position: 19
---

# Joint-Level Control

## Learning Objectives

By the end of this chapter, you will be able to:

- Implement PID controllers for high-DOF humanoid joint tracking
- Use MoveIt for motion planning and inverse kinematics
- Coordinate whole-body motions with balance constraints
- Handle common control issues (limits, collisions, singularities)
- Tune control gains for stable, responsive humanoid behavior

## Introduction

Controlling humanoid robots presents unique challenges due to their **high degrees of freedom** (20-40 joints for full-body humanoids) and complex kinematic chains. Unlike mobile robots with 2-3 DOF or industrial arms with 6-7 DOF, humanoids require coordinated control across multiple limbs (arms, legs, torso, head) while maintaining balance and avoiding self-collision.

Humanoid joints come in two primary types:

- **Revolute joints**: Rotational (e.g., shoulder pitch/roll/yaw, knee)
- **Prismatic joints**: Linear (rare in humanoids, used in telescoping mechanisms)

Each joint can be controlled in different modes:

- **Position control**: Command target joint angle (most common for humanoids)
- **Velocity control**: Command target joint velocity (useful for compliant motions)
- **Torque control**: Command target joint torque (advanced, requires good dynamics models)

In this chapter, we'll explore PID control for joint-level tracking, integrate MoveIt for high-level motion planning, implement whole-body coordination, and handle common pitfalls in humanoid control.

## PID Control for Joint Tracking

**PID (Proportional-Integral-Derivative)** control is the foundation of joint-level tracking. Given a target joint position θ_desired and current joint position θ_actual, a PID controller computes the control command u:

**u(t) = K_p * e(t) + K_i * ∫e(t)dt + K_d * de/dt**

where:
- **e(t) = θ_desired - θ_actual** (position error)
- **K_p**: Proportional gain (immediate response to error)
- **K_i**: Integral gain (eliminates steady-state error)
- **K_d**: Derivative gain (dampens oscillations)

### Implementing a PID Controller

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import numpy as np

class HumanoidPIDController(Node):
    """PID controller for humanoid joint tracking"""

    def __init__(self):
        super().__init__('humanoid_pid_controller')

        # PID gains (per joint, tuned experimentally)
        self.num_joints = 20
        self.kp = np.array([100.0] * self.num_joints)  # Proportional
        self.ki = np.array([0.1] * self.num_joints)    # Integral
        self.kd = np.array([10.0] * self.num_joints)   # Derivative

        # State variables
        self.target_positions = np.zeros(self.num_joints)
        self.current_positions = np.zeros(self.num_joints)
        self.current_velocities = np.zeros(self.num_joints)
        self.error_integral = np.zeros(self.num_joints)
        self.prev_error = np.zeros(self.num_joints)

        self.dt = 0.01  # 100 Hz control loop

        # ROS interfaces
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )
        self.target_sub = self.create_subscription(
            Float64MultiArray, '/joint_targets', self.target_callback, 10
        )
        self.cmd_pub = self.create_publisher(
            Float64MultiArray, '/joint_commands', 10
        )

        self.create_timer(self.dt, self.control_loop)

    def joint_state_callback(self, msg):
        """Update current joint positions and velocities"""
        self.current_positions = np.array(msg.position)
        self.current_velocities = np.array(msg.velocity)

    def target_callback(self, msg):
        """Update target joint positions"""
        self.target_positions = np.array(msg.data)

    def control_loop(self):
        """PID control loop at 100 Hz"""
        # Compute error
        error = self.target_positions - self.current_positions

        # Integral term (with anti-windup)
        self.error_integral += error * self.dt
        self.error_integral = np.clip(self.error_integral, -1.0, 1.0)

        # Derivative term (use velocity feedback to avoid noise amplification)
        error_derivative = -self.current_velocities  # de/dt ≈ -v

        # PID output
        u = (
            self.kp * error +
            self.ki * self.error_integral +
            self.kd * error_derivative
        )

        # Publish commands
        cmd = Float64MultiArray()
        cmd.data = u.tolist()
        self.cmd_pub.publish(cmd)

def main():
    rclpy.init()
    node = HumanoidPIDController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Tuning PID Gains

Use the **Ziegler-Nichols method** for initial tuning:

1. Set K_i = 0, K_d = 0
2. Increase K_p until the system oscillates with constant amplitude (critical gain K_u)
3. Measure oscillation period T_u
4. Set gains:
   - K_p = 0.6 * K_u
   - K_i = 1.2 * K_u / T_u
   - K_d = 0.075 * K_u * T_u

**Practical tips**:
- **High K_p**: Fast response but prone to oscillations and overshoot
- **High K_i**: Eliminates steady-state error but can cause instability
- **High K_d**: Dampens oscillations but amplifies sensor noise

For humanoids, start with conservative gains and increase gradually while testing on the real robot.

## MoveIt Integration for Motion Planning

**MoveIt** is the standard motion planning framework for ROS 2, providing inverse kinematics (IK), collision checking, and trajectory generation. For humanoids, MoveIt enables high-level task-space control ("move hand to position X") that automatically computes joint trajectories.

### Planning Groups

Define planning groups for different body parts in `moveit_config/srdf`:

```xml
<!-- File: config/humanoid.srdf -->
<robot name="humanoid">
  <!-- Right arm group (7 DOF) -->
  <group name="right_arm">
    <joint name="r_shoulder_pitch" />
    <joint name="r_shoulder_roll" />
    <joint name="r_shoulder_yaw" />
    <joint name="r_elbow" />
    <joint name="r_wrist_pitch" />
    <joint name="r_wrist_roll" />
    <joint name="r_wrist_yaw" />
  </group>

  <!-- Left arm group -->
  <group name="left_arm">
    <joint name="l_shoulder_pitch" />
    <!-- ... -->
  </group>

  <!-- Upper body (arms + torso) -->
  <group name="upper_body">
    <joint name="torso_yaw" />
    <joint name="torso_pitch" />
    <group name="right_arm" />
    <group name="left_arm" />
  </group>
</robot>
```

### Inverse Kinematics Solvers

MoveIt supports multiple IK solvers:

| Solver | Speed | Accuracy | Notes |
|--------|-------|----------|-------|
| KDL | Fast | Good | Closed-form, good for 6-7 DOF |
| TRAC-IK | Medium | Excellent | Optimization-based, handles redundancy |
| BioIK | Slow | Excellent | Handles complex constraints |

For humanoids with redundant arms (7+ DOF), use **TRAC-IK**:

```yaml
# File: config/kinematics.yaml

right_arm:
  kinematics_solver: trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin
  kinematics_solver_search_resolution: 0.005
  kinematics_solver_timeout: 0.05
```

### Planning and Executing Trajectories

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from moveit.planning import MoveItPy
from moveit.core.robot_state import RobotState
from geometry_msgs.msg import PoseStamped

class HumanoidMotionPlanner(Node):
    """High-level motion planning with MoveIt"""

    def __init__(self):
        super().__init__('humanoid_motion_planner')

        # Initialize MoveItPy
        self.moveit = MoveItPy(node_name="moveit_py")
        self.right_arm = self.moveit.get_planning_component("right_arm")

    def plan_to_pose(self, target_pose):
        """
        Plan arm motion to target end-effector pose

        Args:
            target_pose: PoseStamped (position + orientation)

        Returns:
            Planned trajectory (RobotTrajectory)
        """
        # Set pose goal
        self.right_arm.set_goal_state(
            pose_stamped_msg=target_pose,
            pose_link="r_gripper_link"
        )

        # Plan
        plan_result = self.right_arm.plan()

        if plan_result.error_code.val != 1:
            self.get_logger().error(f'Planning failed: {plan_result.error_code}')
            return None

        self.get_logger().info('Planning succeeded')
        return plan_result.trajectory

    def execute_trajectory(self, trajectory):
        """Execute planned trajectory on real robot"""
        # Execute blocking (waits for completion)
        result = self.right_arm.execute(trajectory, blocking=True)

        if result.error_code.val == 1:
            self.get_logger().info('Execution succeeded')
        else:
            self.get_logger().error(f'Execution failed: {result.error_code}')

        return result.error_code.val == 1

    def reach_and_grasp(self, object_pose):
        """High-level task: reach to object and grasp"""
        # Approach pose (10cm above object)
        approach_pose = object_pose
        approach_pose.pose.position.z += 0.1

        # Plan and execute approach
        traj_approach = self.plan_to_pose(approach_pose)
        if traj_approach is None:
            return False

        self.execute_trajectory(traj_approach)

        # Plan and execute grasp (move down to object)
        traj_grasp = self.plan_to_pose(object_pose)
        if traj_grasp is None:
            return False

        self.execute_trajectory(traj_grasp)

        self.get_logger().info('Grasp complete')
        return True

def main():
    rclpy.init()
    node = HumanoidMotionPlanner()

    # Example: reach to target pose
    target = PoseStamped()
    target.header.frame_id = "base_link"
    target.pose.position.x = 0.5
    target.pose.position.y = 0.3
    target.pose.position.z = 1.0
    target.pose.orientation.w = 1.0

    node.reach_and_grasp(target)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Whole-Body Control

For tasks requiring coordination across multiple limbs (e.g., waving while maintaining balance), we need **whole-body control** that considers:

- **Balance constraints**: Keep center of mass (COM) above support polygon
- **Joint limits**: Respect maximum/minimum joint angles
- **Collision avoidance**: Prevent self-collision between limbs

### Task-Space vs Joint-Space Control

**Joint-space control**: Command joint angles directly

```python
# Joint-space: specify exact joint configuration
target_joints = [0.1, 0.2, -0.5, 1.0, ...]  # radians
```

**Task-space control**: Command end-effector pose, solve for joints via IK

```python
# Task-space: specify desired hand position
target_pose = PoseStamped()
target_pose.pose.position = (0.5, 0.3, 1.0)
# MoveIt computes joint angles automatically
```

Task-space is preferred for manipulation (easier to specify "grab object at X") while joint-space is used for whole-body poses (e.g., predefined gestures).

### Coordinated Arm + Torso Motion

```python
def plan_coordinated_reach(self, target_pose):
    """
    Plan reach using arm + torso for extended workspace

    Uses upper_body planning group (includes torso joints)
    """
    upper_body = self.moveit.get_planning_component("upper_body")

    # Set pose goal for right hand
    upper_body.set_goal_state(
        pose_stamped_msg=target_pose,
        pose_link="r_gripper_link"
    )

    # Add balance constraint (keep COM over feet)
    constraints = Constraints()
    constraints.name = "balance"

    position_constraint = PositionConstraint()
    position_constraint.link_name = "base_link"
    position_constraint.target_point_offset.z = 0.0  # COM height
    # ... define constraint region

    upper_body.set_path_constraints(constraints)

    # Plan (MoveIt automatically uses torso for extended reach)
    plan_result = upper_body.plan()

    return plan_result.trajectory
```

## Common Issues and Solutions

### Issue 1: Joint Limit Violations

**Problem**: Commanded joint positions exceed URDF-defined limits, causing hardware errors.

**Solution**: Implement software joint limit checking:

```python
def enforce_joint_limits(self, joint_positions):
    """Clamp joint positions to safe limits"""
    joint_limits_lower = np.array([-3.14, -1.57, -3.14, ...])  # radians
    joint_limits_upper = np.array([3.14, 1.57, 3.14, ...])

    return np.clip(joint_positions, joint_limits_lower, joint_limits_upper)
```

Add a safety margin (e.g., 5°) to avoid mechanical hard stops.

### Issue 2: Self-Collision

**Problem**: During whole-body motions, limbs collide with each other or the torso.

**Solution**: Enable MoveIt collision checking:

```python
# In SRDF, define collision pairs to disable (for adjacent links)
<disable_collisions link1="r_shoulder" link2="r_upper_arm" reason="Adjacent" />

# In planning, MoveIt automatically avoids all other collisions
plan_result = arm.plan()  # Collision-free by default
```

### Issue 3: Kinematic Singularities

**Problem**: IK solver fails near singularities (e.g., fully extended arm).

**Solution**: Add redundant DOF (7-DOF arm instead of 6) or constraint elbow position:

```python
# Constraint to keep elbow bent (avoid singularity)
joint_constraint = JointConstraint()
joint_constraint.joint_name = "r_elbow"
joint_constraint.position = 1.57  # 90° bent
joint_constraint.tolerance_above = 1.0
joint_constraint.tolerance_below = 0.5
joint_constraint.weight = 0.5

constraints.joint_constraints.append(joint_constraint)
```

### Issue 4: Oscillations and Instability

**Problem**: High PID gains cause joints to oscillate around target.

**Solution**: Reduce K_d or add low-pass filtering to derivative term:

```python
# Exponential moving average for derivative
self.error_derivative_filtered = (
    0.9 * self.error_derivative_filtered +
    0.1 * error_derivative
)

u_d = self.kd * self.error_derivative_filtered
```

## Best Practices

1. **Start with simulation**: Test control gains in Gazebo before real hardware
2. **Monitor joint torques**: Detect overloads before hardware damage
3. **Implement safety limits**: Software e-stops for unexpected motions
4. **Use feedforward control**: Compensate for gravity/inertia (advanced)
5. **Log control data**: Record errors, commands, and states for tuning

## Summary

Key takeaways from this chapter:

- **PID control** provides stable joint-level tracking for humanoid robots when properly tuned (Kp ~100, Ki ~0.1, Kd ~10)
- **MoveIt** enables high-level task-space control with automatic IK, collision checking, and trajectory generation
- **Whole-body coordination** requires balancing multiple objectives (task goals, balance, joint limits, collisions)
- **Common pitfalls** (joint limits, singularities, oscillations) have well-established solutions
- **Tuning** is iterative: start conservative in simulation, refine on real hardware

These techniques form the foundation for humanoid manipulation and will be extended in the next chapter for grasping tasks.

## Review Questions

1. What are the roles of the P, I, and D terms in a PID controller? How does each affect system response?
2. Why is TRAC-IK preferred over KDL for redundant humanoid arms?
3. Explain the difference between task-space and joint-space control. When would you use each?
4. How does MoveIt handle collision avoidance during motion planning?
5. What causes kinematic singularities, and how can they be avoided?
6. Describe a scenario where whole-body control (arm + torso) is necessary.
7. How would you debug a humanoid arm that oscillates when reaching to a target?

## Hands-On Exercises

### Exercise 1: PID Tuning

**Objective**: Tune PID gains for a simulated humanoid arm joint.

**Steps**:
1. Spawn humanoid in Gazebo
2. Implement PID controller for shoulder pitch joint
3. Use Ziegler-Nichols method to find initial gains
4. Test step response (plot position vs time)
5. Iteratively tune to achieve &lt;5% overshoot, &lt;1s settling time

**Expected Outcome**: Smooth, responsive joint tracking without oscillations.

### Exercise 2: MoveIt Motion Planning

**Objective**: Use MoveIt to plan collision-free arm motions.

**Steps**:
1. Configure MoveIt for your humanoid URDF
2. Define right_arm planning group
3. Implement plan_to_pose function
4. Command arm to reach multiple target poses
5. Visualize planned trajectories in RViz

**Expected Outcome**: Collision-free reaching motions to all target poses.

### Exercise 3: Whole-Body Waving

**Objective**: Implement a waving gesture using arm + torso coordination.

**Steps**:
1. Define upper_body planning group (arm + torso)
2. Plan a waving motion (hand moves side-to-side)
3. Add balance constraint to keep COM over feet
4. Execute on simulated humanoid
5. Verify torso compensates for arm motion

**Expected Outcome**: Smooth waving gesture without tipping over.

## Further Reading

- **Modern Robotics** (Lynch & Park): Chapter 11 on robot control
- **MoveIt Documentation**: [https://moveit.picknik.ai/](https://moveit.picknik.ai/)
- **TRAC-IK Paper**: "TRAC-IK: An Open-Source Library for Improved Solving of Generic Inverse Kinematics" (2015)
- **PID Control Tutorial**: [https://www.ni.com/en-us/innovations/white-papers/06/pid-theory-explained.html](https://www.ni.com/en-us/innovations/white-papers/06/pid-theory-explained.html)
- **ROS 2 Control**: [https://control.ros.org/](https://control.ros.org/)

---

**Previous**: [Chapter 17 - Humanoid Robot URDF and Kinematics](../week-11/ch17-humanoid-urdf.md)
**Next**: [Chapter 19 - Robotic Grasping with Modern Approaches](../week-12/ch19-grasping.md)
