---
id: ch06-nav2-basics
title: Navigation with Nav2
sidebar_label: Navigation with Nav2
sidebar_position: 2
---

# Navigation with Nav2

## Learning Objectives

By the end of this chapter, you will be able to:

- Understand the Nav2 stack architecture and its components
- Explain how costmaps represent obstacles and free space
- Describe path planning algorithms used in Nav2
- Understand behavior trees for navigation logic
- Implement recovery behaviors for navigation failures
- Configure AMCL for robot localization

## Introduction

Autonomous navigation is one of the most challenging and essential capabilities for mobile robots. **Nav2 (Navigation 2)** is the ROS 2 navigation framework that enables robots to safely navigate from point A to point B while avoiding obstacles, localizing themselves, and recovering from failures.

**The navigation challenge**: A mobile robot must:
- **Know where it is** (localization)
- **Know where to go** (goal specification)
- **Plan a path** (global and local planning)
- **Execute the path** (trajectory following)
- **Avoid obstacles** (dynamic and static)
- **Recover from failures** (stuck, lost, blocked)

Nav2 provides a complete, production-ready solution for these problems, building on decades of research and real-world deployments. It's used in warehouses, hospitals, hotels, factories, and research labs worldwide.

This chapter introduces the core concepts of Nav2 and shows how to use its powerful navigation capabilities.

## 1. Nav2 Stack Overview

### Architecture

Nav2 follows a **layered architecture** with multiple specialized servers:

```
┌─────────────────────────────────────────────┐
│           Behavior Tree Navigator           │  (Orchestration)
└────────┬────────────────────────────────────┘
         │
    ┌────▼────┐  ┌──────────┐  ┌──────────┐
    │ Planner │  │Controller│  │Recoveries│
    │ Server  │  │  Server  │  │  Server  │
    └────┬────┘  └────┬─────┘  └────┬─────┘
         │            │              │
    ┌────▼────────────▼──────────────▼──────┐
    │          Costmap 2D                    │  (World representation)
    └────┬─────────────────────────────┬─────┘
         │                             │
    ┌────▼─────┐                  ┌────▼─────┐
    │  Sensor  │                  │   AMCL   │  (Localization)
    │  Data    │                  │          │
    └──────────┘                  └──────────┘
```

### Key Components

**1. Planner Server**
- Computes **global path** from start to goal
- Uses graph search algorithms (A*, Dijkstra, Theta*, Smac Planner)
- Runs periodically or on-demand
- Considers static map and inflated obstacles

**2. Controller Server**
- Follows global path with **local trajectory**
- Avoids dynamic obstacles
- Uses DWA (Dynamic Window Approach), TEB, MPPI, RPP
- Runs at high frequency (10-20 Hz)

**3. Recoveries Server**
- Executes recovery behaviors when stuck
- Includes: rotate in place, back up, wait, clear costmap
- Prevents robot from getting permanently stuck

**4. Behavior Tree Navigator**
- Coordinates all servers
- Implements navigation logic as behavior trees
- Customizable for different applications

**5. Costmap 2D**
- Represents environment as occupancy grid
- Fuses sensor data (LiDAR, depth cameras)
- Inflates obstacles for safety
- Global and local costmaps

**6. AMCL (Adaptive Monte Carlo Localization)**
- Estimates robot pose on known map
- Uses particle filter algorithm
- Fuses odometry and laser scans

### Communication Pattern

```python
from nav2_simple_commander.robot_navigator import BasicNavigator
import rclpy

class NavigationExample:
    def __init__(self):
        rclpy.init()
        self.navigator = BasicNavigator()

        # Wait for Nav2 to be ready
        self.navigator.waitUntilNav2Active()

    def navigate_to_pose(self, x, y, theta):
        """Navigate to a goal pose"""
        from geometry_msgs.msg import PoseStamped

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.orientation.z = math.sin(theta / 2.0)
        goal_pose.pose.orientation.w = math.cos(theta / 2.0)

        # Send goal
        self.navigator.goToPose(goal_pose)

        # Monitor progress
        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            print(f'Distance remaining: {feedback.distance_remaining:.2f}m')
            time.sleep(0.5)

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Navigation succeeded!')
        else:
            print(f'Navigation failed: {result}')
```

## 2. Costmaps and Planning

### Costmap Representation

A **costmap** is a 2D occupancy grid where each cell has a cost value (0-255):

- **0**: Free space (safe to traverse)
- **1-127**: Low cost (prefer to avoid)
- **128-252**: High cost (obstacles nearby)
- **253**: Inscribed inflated obstacle
- **254**: Lethal obstacle (collision)
- **255**: Unknown space

### Costmap Layers

Costmaps are built from multiple layers:

**1. Static Layer**
- Loads from pre-built map (from SLAM)
- Represents walls, furniture, fixed obstacles
- Doesn't change during runtime

**2. Obstacle Layer**
- Adds dynamic obstacles from sensors
- Uses LiDAR scans, depth cameras
- Clears after obstacles move away

**3. Inflation Layer**
- Expands obstacles by robot radius
- Creates safety buffer
- Exponential cost decay from obstacles

**4. Voxel Layer (optional)**
- 3D representation (for stairs, overhangs)
- Clears obstacles above ground

**Example costmap configuration:**

```yaml
# global_costmap_params.yaml
global_costmap:
  global_frame: map
  robot_base_frame: base_link
  update_frequency: 1.0
  publish_frequency: 1.0
  resolution: 0.05  # 5cm cells

  static_layer:
    plugin: "nav2_costmap_2d::StaticLayer"
    map_subscribe_transient_local: True

  obstacle_layer:
    plugin: "nav2_costmap_2d::ObstacleLayer"
    observation_sources: scan
    scan:
      topic: /scan
      max_obstacle_height: 2.0
      clearing: True
      marking: True

  inflation_layer:
    plugin: "nav2_costmap_2d::InflationLayer"
    cost_scaling_factor: 3.0
    inflation_radius: 0.55  # Robot radius + safety margin
```

### Path Planning Algorithms

**1. NavFn Planner (Dijkstra variant)**
- Guaranteed shortest path
- Slower but thorough
- Good for simple environments

**2. Smac Planner 2D (State Lattice)**
- Considers robot kinematics
- Smoother paths for car-like robots
- Supports Ackermann steering

**3. Smac Planner Hybrid-A***
- Best for complex environments
- Considers robot shape and turning radius
- More computationally expensive

**Choosing a planner:**
```yaml
# planner_server.yaml
planner_server:
  ros__parameters:
    expected_planner_frequency: 1.0
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"  # or "nav2_smac_planner/SmacPlanner2D"
      tolerance: 0.5
      use_astar: true  # A* (true) vs Dijkstra (false)
```

### Global vs Local Costmaps

**Global Costmap:**
- Covers entire known map
- Low update frequency (1 Hz)
- Used for long-range planning
- Large footprint in memory

**Local Costmap:**
- Small window around robot (e.g., 5m x 5m)
- High update frequency (5-10 Hz)
- Used for local obstacle avoidance
- Real-time dynamic obstacle handling

## 3. Behavior Trees

### What Are Behavior Trees?

Behavior trees (BTs) are a hierarchical structure for decision-making. In Nav2, they orchestrate the navigation pipeline.

**Key node types:**

1. **Sequence**: Executes children left-to-right until one fails
2. **Fallback (Selector)**: Tries children until one succeeds
3. **Parallel**: Executes multiple children simultaneously
4. **Decorator**: Modifies child behavior (retry, timeout, precondition)

### Default Navigation Behavior Tree

```xml
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <RecoveryNode number_of_retries="3">
      <PipelineSequence>
        <!-- Compute path -->
        <ComputePathToPose goal="{goal}"/>

        <!-- Follow path -->
        <FollowPath path="{path}"/>
      </PipelineSequence>

      <!-- Recovery behaviors if navigation fails -->
      <Fallback>
        <ClearEntireCostmap/>
        <Spin spin_dist="1.57"/>  <!-- 90 degrees -->
        <Wait wait_duration="5"/>
        <BackUp backup_dist="0.3" backup_speed="0.1"/>
      </Fallback>
    </RecoveryNode>
  </BehaviorTree>
</root>
```

### Custom Behavior Tree Example

```xml
<!-- Navigate with battery check -->
<root>
  <BehaviorTree ID="NavigateWithBatteryCheck">
    <Sequence>
      <!-- Pre-check: Battery OK? -->
      <IsBatteryOK min_battery="20.0"/>

      <!-- Main navigation -->
      <RecoveryNode number_of_retries="2">
        <PipelineSequence>
          <ComputePathToPose goal="{goal}"/>
          <FollowPath path="{path}"/>
        </PipelineSequence>
        <RecoveryFallback>
          <Spin/>
          <BackUp/>
        </RecoveryFallback>
      </RecoveryNode>

      <!-- Post-check: Send completion message -->
      <PublishNavigationComplete/>
    </Sequence>
  </BehaviorTree>
</root>
```

## 4. Recovery Behaviors

### Why Recovery Behaviors?

Robots get stuck due to:
- Dead ends
- Dynamic obstacles (people, doors)
- Sensor noise
- Localization errors
- Unexpected obstacles

Recovery behaviors provide graceful failure handling.

### Built-in Recoveries

**1. Spin**
- Rotate in place (360° or specified angle)
- Helps relocalize (AMCL)
- Finds alternate path after rotating

```yaml
# recoveries_server.yaml
recoveries_server:
  ros__parameters:
    costmap_topic: local_costmap/costmap_raw
    footprint_topic: local_costmap/published_footprint
    recovery_plugins: ["spin", "backup", "wait"]

    spin:
      plugin: "nav2_recoveries/Spin"
      simulate_ahead_time: 2.0

    backup:
      plugin: "nav2_recoveries/BackUp"
      simulate_ahead_time: 2.0

    wait:
      plugin: "nav2_recoveries/Wait"
```

**2. Back Up**
- Reverse along current heading
- Useful when robot is too close to obstacle
- Configurable distance and speed

**3. Wait**
- Pause for specified duration
- Allows dynamic obstacles to clear
- Useful in crowded environments

**4. Clear Costmap**
- Removes sensor artifacts
- Helps with false detections
- Use sparingly (can be dangerous)

### Custom Recovery Behavior

```python
from nav2_msgs.action import BackUp
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

class ManualRecovery(Node):
    def __init__(self):
        super().__init__('manual_recovery')
        self.backup_client = ActionClient(self, BackUp, 'backup')

    def execute_recovery(self):
        """Back up 0.3m at 0.1 m/s"""
        goal = BackUp.Goal()
        goal.target.x = -0.3  # Negative = backward
        goal.speed = 0.1

        self.backup_client.wait_for_server()
        future = self.backup_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)

        if future.result().accepted:
            self.get_logger().info('Recovery initiated')
        else:
            self.get_logger().error('Recovery rejected')
```

## 5. Localization (AMCL)

### Adaptive Monte Carlo Localization

AMCL estimates robot pose using:
- **Laser scan** matching against known map
- **Odometry** for motion model
- **Particle filter** for probabilistic estimation

**How it works:**

1. **Initialization**: Scatter particles across map (or near known pose)
2. **Prediction**: Move particles based on odometry
3. **Update**: Weight particles by laser scan match quality
4. **Resampling**: Replace low-weight particles with high-weight copies
5. **Pose estimate**: Weighted average of particles

### AMCL Configuration

```yaml
# amcl.yaml
amcl:
  ros__parameters:
    # Particle filter
    min_particles: 500
    max_particles: 2000

    # Odometry model (differential drive)
    odom_model_type: "diff-corrected"
    odom_alpha1: 0.2  # Rotation noise from rotation
    odom_alpha2: 0.2  # Rotation noise from translation
    odom_alpha3: 0.2  # Translation noise from translation
    odom_alpha4: 0.2  # Translation noise from rotation

    # Laser model
    laser_model_type: "likelihood_field"
    laser_max_range: 12.0
    laser_min_range: 0.1
    laser_likelihood_max_dist: 2.0

    # Update rates
    update_min_d: 0.2  # Min translation before update (m)
    update_min_a: 0.5  # Min rotation before update (rad)
    resample_interval: 1

    # Initial pose
    set_initial_pose: true
    initial_pose:
      x: 0.0
      y: 0.0
      z: 0.0
      yaw: 0.0
```

### Using AMCL

```python
from geometry_msgs.msg import PoseWithCovarianceStamped
import rclpy
from rclpy.node import Node

class LocalizationMonitor(Node):
    def __init__(self):
        super().__init__('localization_monitor')

        # Subscribe to AMCL pose
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10
        )

        # Publish initial pose (optional)
        self.init_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10
        )

    def pose_callback(self, msg):
        """Monitor localization quality"""
        # Extract pose
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Extract covariance (uncertainty)
        cov_x = msg.pose.covariance[0]   # Variance in x
        cov_y = msg.pose.covariance[7]   # Variance in y
        cov_yaw = msg.pose.covariance[35]  # Variance in yaw

        # Check localization quality
        if cov_x > 0.5 or cov_y > 0.5:
            self.get_logger().warn('High localization uncertainty!')

    def set_initial_pose(self, x, y, yaw):
        """Manually set robot's initial pose"""
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        # Set covariance (initial uncertainty)
        msg.pose.covariance[0] = 0.5  # x
        msg.pose.covariance[7] = 0.5  # y
        msg.pose.covariance[35] = 0.1  # yaw

        self.init_pose_pub.publish(msg)
```

### Debugging Localization

**Common issues:**

1. **Robot kidnapped (teleported)**: AMCL can't recover
   - Solution: Use global localization (`request_nomotion_update`)

2. **High uncertainty**: Covariance keeps growing
   - Solution: Check laser scan quality, reduce odometry noise parameters

3. **False localization**: Robot thinks it's somewhere else
   - Solution: More particles, better laser scan, distinctive features in map

## Summary

Key takeaways:

- **Nav2** provides production-ready autonomous navigation for ROS 2
- **Costmaps** represent obstacles with multiple layers (static, obstacle, inflation)
- **Planners** compute global paths using graph search algorithms
- **Controllers** follow paths while avoiding dynamic obstacles
- **Behavior trees** orchestrate the navigation pipeline with recovery logic
- **AMCL** localizes the robot on known maps using particle filters
- **Recovery behaviors** handle navigation failures gracefully

**Navigation pipeline:**
1. AMCL estimates robot pose
2. Planner computes global path
3. Controller follows path locally
4. If stuck → Execute recovery behaviors
5. Repeat until goal reached

## Review Questions

1. What is the difference between a global costmap and a local costmap?
2. Why does Nav2 use both a planner server and a controller server?
3. How does the inflation layer improve navigation safety?
4. What is the purpose of behavior trees in Nav2?
5. How does AMCL estimate robot pose?
6. When would you use a recovery behavior instead of replanning?
7. What are the advantages of using Smac Planner over NavFn Planner?

## Hands-on Exercises

### Exercise 1: Costmap Visualization
- Launch Nav2 with TurtleBot3 simulation
- Visualize global and local costmaps in RViz
- Observe how costmaps update as robot moves
- Add dynamic obstacles and watch inflation

### Exercise 2: Custom Behavior Tree
Create a behavior tree that:
- Checks battery level before navigating
- Navigates to charging station if battery low
- Uses recovery behaviors if path blocked
- Sends notification when goal reached

### Exercise 3: AMCL Tuning
- Run AMCL with default parameters
- Measure localization accuracy (error from ground truth)
- Adjust particle count, noise parameters
- Compare before/after localization quality

## Further Reading

- [Nav2 Official Documentation](https://navigation.ros.org/)
- [Behavior Trees in Robotics](https://arxiv.org/abs/1709.00084)
- [AMCL Algorithm](http://robots.stanford.edu/papers/fox.aaai99.pdf)
- [Nav2 Tutorials](https://navigation.ros.org/tutorials/index.html)

---

**Next Chapter**: [Module 2: Gazebo Physics Simulation →](../../module-02-digital-twin/week-06/ch07-gazebo-physics.md)

**Previous Chapter**: [← URDF Robot Models](ch05-urdf-models.md)
