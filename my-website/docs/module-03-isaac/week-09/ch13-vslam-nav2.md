---
id: ch13-vslam-nav2
title: Visual SLAM and Navigation
sidebar_label: Visual SLAM and Navigation
sidebar_position: 14
---

# Visual SLAM and Navigation

## Learning Objectives

By the end of this chapter, you will be able to:

- Understand Visual SLAM algorithms (ORB-SLAM3, RTAB-Map)
- Implement visual odometry for camera-based localization
- Leverage loop closure for drift correction
- Integrate Visual SLAM with ROS 2 Nav2 navigation stack
- Apply different mapping strategies for autonomous navigation

## Introduction

LiDAR-based SLAM works great in structured environments, but what about robots that only have cameras? Indoor drones, handheld devices, and cost-sensitive robots rely entirely on vision. **How do we enable robots to navigate using only cameras, without expensive LiDAR sensors?**

This is where **Visual SLAM (V-SLAM)** becomes essential. V-SLAM uses cameras to:
- **Localize**: Estimate robot pose from visual features
- **Map**: Build 3D maps of the environment
- **Navigate**: Plan paths through mapped spaces

**Why Visual SLAM matters:**
- ✅ **Cost-effective**: Cameras are 10-100x cheaper than LiDAR
- ✅ **Rich information**: Color, texture, object recognition
- ✅ **Omnidirectional**: Fish-eye or 360° cameras see everywhere
- ✅ **Versatile**: Works indoors, outdoors, underwater

**Challenges:**
- ❌ Computationally intensive (feature extraction, matching)
- ❌ Sensitive to lighting changes
- ❌ Struggles with texture-less surfaces
- ❌ Scale ambiguity with monocular cameras

**Real-world applications:**
- **AR/VR headsets**: Meta Quest, HoloLens use V-SLAM for tracking
- **Drones**: DJI drones use visual-inertial SLAM for indoor flight
- **Warehouse robots**: Amazon robots combine V-SLAM with LiDAR
- **Autonomous cars**: Teslas use vision-based localization

This chapter teaches you how to implement Visual SLAM and integrate it with ROS 2's Nav2 navigation stack.

## 1. VSLAM Algorithms (ORB-SLAM)

### What is Visual SLAM?

**Visual SLAM** estimates camera pose and builds a map simultaneously using:
- **Feature detection**: Find distinctive points in images
- **Feature matching**: Match points across frames
- **Pose estimation**: Calculate camera movement
- **Map building**: Triangulate 3D positions of features
- **Loop closure**: Detect revisited locations

### ORB-SLAM3

**ORB-SLAM3** is the state-of-the-art open-source V-SLAM system supporting:
- Monocular, stereo, RGB-D, and multi-camera setups
- Visual-inertial SLAM (camera + IMU)
- Real-time operation (30 FPS)
- Loop closure and map optimization

**Pipeline:**
```
1. Feature Extraction (ORB features)
   ↓
2. Initial Pose Estimation (motion model)
   ↓
3. Local Map Tracking
   ↓
4. New Keyframe Decision
   ↓
5. Local Mapping (bundle adjustment)
   ↓
6. Loop Closure Detection
   ↓
7. Global Pose Graph Optimization
```

### ORB Features

**ORB (Oriented FAST and Rotated BRIEF)** features are:
- **Fast to compute**: Real-time performance
- **Rotation invariant**: Works even if camera rotates
- **Binary descriptors**: Efficient matching

```python
import cv2

# Detect ORB features in image
orb = cv2.ORB_create(nfeatures=1000)
keypoints, descriptors = orb.detectAndCompute(image, None)

# Draw keypoints
image_with_keypoints = cv2.drawKeypoints(
    image, keypoints, None, color=(0, 255, 0)
)
```

### Installing ORB-SLAM3 with ROS 2

```bash
# Clone ORB-SLAM3
cd ~/workspaces/ros2_ws/src
git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git

# Build ORB-SLAM3
cd ORB_SLAM3
chmod +x build.sh
./build.sh

# Clone ROS 2 wrapper
cd ~/workspaces/ros2_ws/src
git clone https://github.com/zang09/ORB_SLAM3_ROS2.git

# Build
cd ~/workspaces/ros2_ws
colcon build --packages-select orb_slam3_ros2
```

### Running ORB-SLAM3

```bash
# RGB-D mode (with depth camera)
ros2 run orb_slam3_ros2 rgbd \
  --ros-args \
  -p voc_file:=/path/to/ORBvoc.txt \
  -p settings_file:=/path/to/RealSense_D435i.yaml \
  -r /camera/color/image_raw:=/camera/rgb \
  -r /camera/depth/image_raw:=/camera/depth

# Stereo mode
ros2 run orb_slam3_ros2 stereo \
  --ros-args \
  -p voc_file:=/path/to/ORBvoc.txt \
  -p settings_file:=/path/to/Stereo.yaml
```

**Published topics:**
- `/orb_slam3/camera_pose` (geometry_msgs/PoseStamped)
- `/orb_slam3/map_points` (sensor_msgs/PointCloud2)
- `/orb_slam3/tracking_state` (std_msgs/String)

## 2. Visual Odometry

### What is Visual Odometry?

**Visual odometry (VO)** tracks camera movement between consecutive frames without building a global map.

**Difference from SLAM:**
- **Visual Odometry**: Local tracking only, no loop closure
- **Visual SLAM**: Global map + loop closure

### Stereo Visual Odometry

Stereo cameras provide depth directly, enabling accurate VO:

```python
import cv2
import numpy as np

class StereoVisualOdometry:
    def __init__(self):
        # Stereo matcher
        self.stereo = cv2.StereoBM_create(numDisparities=16*6, blockSize=11)

        # Feature detector
        self.detector = cv2.ORB_create()

        # Previous frame data
        self.prev_keypoints = None
        self.prev_descriptors = None
        self.prev_points_3d = None

    def process_frame(self, left_image, right_image, camera_matrix, baseline):
        # Compute disparity
        disparity = self.stereo.compute(left_image, right_image)

        # Detect features in left image
        keypoints, descriptors = self.detector.detectAndCompute(left_image, None)

        # Compute 3D points from disparity
        points_3d = self.triangulate_points(
            keypoints, disparity, camera_matrix, baseline
        )

        # Match with previous frame
        if self.prev_keypoints is not None:
            matches = self.match_features(descriptors, self.prev_descriptors)

            # Estimate transformation
            R, t = self.estimate_motion(
                self.prev_points_3d[matches[:, 0]],
                points_3d[matches[:, 1]]
            )

            return R, t

        # Store for next frame
        self.prev_keypoints = keypoints
        self.prev_descriptors = descriptors
        self.prev_points_3d = points_3d

        return np.eye(3), np.zeros(3)

    def match_features(self, desc1, desc2):
        # Brute-force matcher
        bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        matches = bf.match(desc1, desc2)

        # Sort by distance
        matches = sorted(matches, key=lambda x: x.distance)

        # Return indices
        return np.array([[m.queryIdx, m.trainIdx] for m in matches[:100]])

    def estimate_motion(self, points_3d_prev, points_3d_curr):
        # Use RANSAC with PnP
        success, rvec, tvec, inliers = cv2.solvePnPRansac(
            points_3d_prev,
            points_3d_curr,
            camera_matrix,
            None
        )

        R, _ = cv2.Rodrigues(rvec)
        return R, tvec
```

### ROS 2 Integration

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge

class VisualOdometryNode(Node):
    def __init__(self):
        super().__init__('visual_odometry')

        self.bridge = CvBridge()
        self.vo = StereoVisualOdometry()

        self.left_sub = self.create_subscription(
            Image, '/camera/left/image_raw', self.left_callback, 10
        )
        self.right_sub = self.create_subscription(
            Image, '/camera/right/image_raw', self.right_callback, 10
        )

        self.pose_pub = self.create_publisher(PoseStamped, '/vo/pose', 10)

        self.cumulative_pose = np.eye(4)  # 4x4 transformation matrix

    def process_stereo_pair(self, left_img, right_img):
        R, t = self.vo.process_frame(left_img, right_img, K, baseline)

        # Update cumulative pose
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = t.flatten()

        self.cumulative_pose = self.cumulative_pose @ T

        # Publish pose
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'odom'

        # Extract position and orientation
        pose_msg.pose.position.x = float(self.cumulative_pose[0, 3])
        pose_msg.pose.position.y = float(self.cumulative_pose[1, 3])
        pose_msg.pose.position.z = float(self.cumulative_pose[2, 3])

        # Convert rotation matrix to quaternion
        quat = self.rotation_matrix_to_quaternion(self.cumulative_pose[:3, :3])
        pose_msg.pose.orientation.x = quat[0]
        pose_msg.pose.orientation.y = quat[1]
        pose_msg.pose.orientation.z = quat[2]
        pose_msg.pose.orientation.w = quat[3]

        self.pose_pub.publish(pose_msg)
```

## 3. Loop Closure

### What is Loop Closure?

**Loop closure** detects when the robot revisits a previously mapped location and corrects accumulated drift.

**Why needed:**
- Visual odometry accumulates drift over time
- Without loop closure, maps become increasingly inconsistent
- Loop closure enforces global consistency

### DBoW2 Place Recognition

**DBoW2 (Bag of Words)** is a popular algorithm for detecting loop closures:

1. **Build vocabulary**: Cluster ORB descriptors into visual "words"
2. **Image representation**: Each image = histogram of visual words
3. **Similarity search**: Compare current image to database
4. **Loop detection**: High similarity = loop closure

### Pose Graph Optimization

When a loop is detected:

```
1. Add loop closure constraint between poses
   ↓
2. Build pose graph
   Nodes = keyframe poses
   Edges = odometry + loop closures
   ↓
3. Optimize graph (g2o, Ceres)
   Minimize reprojection error
   ↓
4. Update all pose estimates
   ↓
5. Rebuild map with corrected poses
```

### Loop Closure Example

```python
class LoopClosureDetector:
    def __init__(self, vocabulary_file):
        # Load pre-trained vocabulary
        self.vocabulary = self.load_vocabulary(vocabulary_file)

        # Database of keyframe descriptors
        self.keyframe_database = []

    def detect_loop(self, current_descriptors, current_pose, threshold=0.75):
        # Compute BoW vector for current frame
        current_bow = self.compute_bow_vector(current_descriptors)

        # Search database for similar frames
        best_match_score = 0
        best_match_idx = -1

        for idx, (bow_vec, pose) in enumerate(self.keyframe_database):
            score = self.compute_similarity(current_bow, bow_vec)

            if score &gt; best_match_score:
                best_match_score = score
                best_match_idx = idx

        # Check if loop closure detected
        if best_match_score &gt; threshold:
            loop_pose = self.keyframe_database[best_match_idx][1]
            return True, loop_pose

        # Add current frame to database
        self.keyframe_database.append((current_bow, current_pose))

        return False, None

    def optimize_pose_graph(self, loop_from, loop_to):
        # Use g2o or Ceres to optimize
        # Minimize error across all poses and loop closures
        pass
```

## 4. Nav2 Integration

### Connecting V-SLAM to Nav2

**Nav2** (ROS 2 Navigation) requires:
- **Map**: 2D occupancy grid
- **Localization**: Robot pose in map frame
- **TF transforms**: `map` → `odom` → `base_link`

### Converting V-SLAM Output to Nav2

```python
from nav_msgs.msg import OccupancyGrid
import numpy as np

class VSLAMToNav2Bridge(Node):
    def __init__(self):
        super().__init__('vslam_nav2_bridge')

        # Subscribe to V-SLAM outputs
        self.pose_sub = self.create_subscription(
            PoseStamped, '/orb_slam3/camera_pose', self.pose_callback, 10
        )
        self.points_sub = self.create_subscription(
            PointCloud2, '/orb_slam3/map_points', self.points_callback, 10
        )

        # Publish Nav2 inputs
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', 10)

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

    def pose_callback(self, msg):
        # Broadcast map → odom transform
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'

        # V-SLAM pose becomes map → base_link
        # Compute map → odom by subtracting odom → base_link
        t.transform.translation.x = msg.pose.position.x
        t.transform.translation.y = msg.pose.position.y
        t.transform.translation.z = msg.pose.position.z
        t.transform.rotation = msg.pose.orientation

        self.tf_broadcaster.sendTransform(t)

    def points_callback(self, msg):
        # Convert 3D point cloud to 2D occupancy grid
        occupancy_grid = self.project_points_to_grid(msg)
        self.map_pub.publish(occupancy_grid)

    def project_points_to_grid(self, point_cloud):
        # Project 3D points to 2D grid
        grid = OccupancyGrid()
        grid.header.frame_id = 'map'
        grid.info.resolution = 0.05  # 5 cm per cell
        grid.info.width = 400  # 20m wide
        grid.info.height = 400
        grid.data = [0] * (400 * 400)  # Unknown

        # Parse point cloud and mark occupied cells
        for point in pc2.read_points(point_cloud):
            x, y, z = point[:3]

            # Project to grid
            grid_x = int((x - grid.info.origin.position.x) / grid.info.resolution)
            grid_y = int((y - grid.info.origin.position.y) / grid.info.resolution)

            if 0 &lt;= grid_x &lt; grid.info.width and 0 &lt;= grid_y &lt; grid.info.height:
                idx = grid_y * grid.info.width + grid_x
                grid.data[idx] = 100  # Occupied

        return grid
```

### Launching V-SLAM + Nav2

```python
# launch/vslam_nav2.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # ORB-SLAM3
        Node(
            package='orb_slam3_ros2',
            executable='rgbd',
            name='orb_slam3',
            parameters=[{
                'voc_file': '/path/to/ORBvoc.txt',
                'settings_file': '/path/to/settings.yaml'
            }]
        ),

        # V-SLAM to Nav2 bridge
        Node(
            package='my_package',
            executable='vslam_nav2_bridge',
            name='vslam_nav2_bridge'
        ),

        # Nav2
        Node(
            package='nav2_bringup',
            executable='navigation_launch.py',
            name='nav2'
        )
    ])
```

## 5. Mapping Strategies

### 2D vs 3D Maps

| Map Type | Use Case | Storage | Nav2 Compatible |
|----------|----------|---------|-----------------|
| **2D Occupancy Grid** | Mobile robots | Efficient | ✅ Yes |
| **2.5D Height Map** | Rough terrain | Moderate | Partial |
| **3D Voxel Map** | Drones, 3D planning | Large | ❌ No |
| **Point Cloud** | Visualization | Very large | ❌ No |

### Octomap for 3D Mapping

```bash
# Install OctoMap
sudo apt install ros-humble-octomap ros-humble-octomap-server

# Run OctoMap server
ros2 run octomap_server octomap_server_node \
  --ros-args \
  -r cloud_in:=/camera/depth/points \
  -p resolution:=0.05 \
  -p frame_id:=map
```

**Published topics:**
- `/octomap_binary` - Compressed 3D occupancy map
- `/octomap_full` - Full 3D occupancy map

### RTAB-Map (Real-Time Appearance-Based Mapping)

RTAB-Map combines V-SLAM with powerful mapping:

```bash
# Install RTAB-Map
sudo apt install ros-humble-rtabmap-ros

# Launch RTAB-Map
ros2 launch rtabmap_ros rtabmap.launch.py \
  rgb_topic:=/camera/rgb \
  depth_topic:=/camera/depth \
  camera_info_topic:=/camera/camera_info \
  frame_id:=base_link \
  approx_sync:=true
```

**Advantages:**
- Long-term SLAM (handles large environments)
- Loop closure out of the box
- 3D and 2D map outputs
- Works with RGB-D, stereo, or LiDAR

## Summary

Key takeaways from this chapter:

- **Visual SLAM** uses cameras for localization and mapping without LiDAR
- **ORB-SLAM3** is the state-of-the-art V-SLAM framework (stereo, RGB-D, monocular)
- **Visual odometry** tracks camera motion frame-to-frame
- **Loop closure** detects revisited locations and corrects drift
- **Nav2 integration** requires converting 3D V-SLAM maps to 2D occupancy grids

**Best practices:**
- Use stereo or RGB-D cameras (avoid monocular scale ambiguity)
- Combine V-SLAM with IMU for robust tracking (visual-inertial SLAM)
- Enable loop closure for long-term operation
- Use RTAB-Map for production systems (more stable than ORB-SLAM3)
- Fuse V-SLAM with wheel odometry using sensor fusion

## Review Questions

1. What is the difference between Visual Odometry and Visual SLAM?
2. Why do stereo cameras provide better V-SLAM than monocular?
3. What problem does loop closure solve?
4. How do you convert 3D V-SLAM point clouds to 2D Nav2 maps?
5. What are ORB features and why are they used in V-SLAM?
6. When would you use RTAB-Map instead of ORB-SLAM3?

## Hands-on Exercises

### Exercise 1: Run ORB-SLAM3
- Install ORB-SLAM3 and ROS 2 wrapper
- Record a dataset with RGB-D camera
- Run ORB-SLAM3 on recorded data
- Visualize trajectory and map points in RViz

### Exercise 2: Visual Odometry Implementation
- Implement stereo visual odometry
- Subscribe to stereo camera topics
- Publish estimated pose on `/vo/pose`
- Compare against ground truth

### Exercise 3: V-SLAM + Nav2 Integration
- Launch RTAB-Map with RGB-D camera
- Convert RTAB-Map output to occupancy grid
- Launch Nav2 navigation stack
- Command robot to navigate autonomously

## Further Reading

- [ORB-SLAM3 Paper](https://arxiv.org/abs/2007.11898)
- [RTAB-Map Documentation](http://introlab.github.io/rtabmap/)
- [Visual SLAM Tutorial](https://github.com/gaoxiang12/slambook-en)
- [DBoW2 Place Recognition](https://github.com/dorian3d/DBoW2)

---

**Next Chapter**: [Reinforcement Learning for Robots →](../week-10/ch14-rl-robots.md)

**Previous Chapter**: [← ROS 2 + Isaac Sim Integration](../week-08/ch12-ros2-isaac.md)
