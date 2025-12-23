---
id: ch14-perception
title: Perception Pipelines
sidebar_label: Perception Pipelines
sidebar_position: 15
---

# Perception Pipelines

## Learning Objectives

By the end of this chapter, you will be able to:

- Build end-to-end perception pipelines for object detection and segmentation
- Use state-of-the-art models (YOLOv8, RT-DETR, Segformer) in ROS 2
- Process RGB-D data for 3D object localization
- Integrate perception with Nav2 for dynamic obstacle avoidance
- Optimize pipelines for real-time performance (&lt;100ms latency)

## Introduction

**Perception** is the ability of a robot to interpret sensor data and build a semantic understanding of its environment. While sensors like cameras and LiDAR provide raw data (images, point clouds), perception algorithms transform this data into actionable information: "There is a person 2 meters ahead," "The table is 0.8 meters high," or "The doorway is clear for navigation."

For Physical AI systems, robust perception is critical for safe and effective operation. A warehouse robot must detect pallets, forklifts, and people in real-time; a domestic assistant robot must recognize objects for manipulation; an outdoor autonomous vehicle must segment drivable surfaces from obstacles. These tasks require **sensor fusion** (combining camera, depth, and LiDAR data), **real-time processing** (latency &lt;100ms for 10 Hz control loops), and **reliability** (high precision and recall to avoid false positives and missed detections).

Common perception tasks in robotics include:

- **Object Detection**: Identifying and localizing objects with 2D bounding boxes (e.g., "person at (320, 240) with 80x120 box")
- **Semantic Segmentation**: Classifying every pixel in an image (e.g., road, sidewalk, obstacle)
- **3D Object Localization**: Estimating 3D position and orientation from RGB-D or stereo cameras
- **Scene Understanding**: Recognizing higher-level concepts like "doorway," "staircase," or "cluttered workspace"

In this chapter, we'll build production-ready perception pipelines using modern deep learning models, integrate them with ROS 2, and deploy them for real-time navigation and manipulation tasks.

## Object Detection Pipeline

**Object detection** identifies objects of interest in images and returns their 2D bounding boxes and class labels. For robotics, we prioritize **real-time models** that run at 30+ FPS on edge devices (NVIDIA Jetson) while maintaining high accuracy.

### YOLOv8 and RT-DETR

Two state-of-the-art architectures dominate real-time object detection:

1. **YOLOv8** (You Only Look Once v8): Single-stage detector with ~20-80 FPS on Jetson Orin, mAP 50-53% on COCO. Best for general-purpose detection.
2. **RT-DETR** (Real-Time Detection Transformer): Transformer-based detector with dynamic anchor generation, ~30-60 FPS, mAP 53-56%. Better accuracy but slightly slower.

Both models support:
- **Pre-trained weights** on COCO (80 classes: person, car, chair, etc.)
- **Custom training** via transfer learning
- **TensorRT optimization** for 2-3x speedup on NVIDIA GPUs
- **Export to ONNX** for cross-platform deployment

### ROS 2 Integration

Here's a complete YOLOv8 detection node for ROS 2:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge
from ultralytics import YOLO
import numpy as np

class YOLOv8DetectorNode(Node):
    """Real-time object detection using YOLOv8"""

    def __init__(self):
        super().__init__('yolov8_detector')

        # Parameters
        self.declare_parameter('model_path', 'yolov8n.pt')  # nano model
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('classes_of_interest', [0])  # 0 = person

        model_path = self.get_parameter('model_path').value
        self.conf_threshold = self.get_parameter('confidence_threshold').value
        self.classes = self.get_parameter('classes_of_interest').value

        # Load YOLO model
        self.model = YOLO(model_path)
        self.get_logger().info(f'Loaded YOLOv8 model: {model_path}')

        # ROS interfaces
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(
            Image, '/camera/color/image_raw', self.image_callback, 10
        )
        self.detection_pub = self.create_publisher(
            Detection2DArray, '/detections', 10
        )

    def image_callback(self, msg):
        """Process incoming image and publish detections"""
        # Convert ROS Image to OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')

        # Run YOLOv8 inference
        results = self.model(
            cv_image,
            conf=self.conf_threshold,
            classes=self.classes,  # Filter to classes of interest
            verbose=False
        )[0]

        # Convert to Detection2DArray
        detection_array = Detection2DArray()
        detection_array.header = msg.header

        for box in results.boxes:
            detection = Detection2D()

            # Bounding box (center x, y, width, height)
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
            detection.bbox.center.position.x = float((x1 + x2) / 2)
            detection.bbox.center.position.y = float((y1 + y2) / 2)
            detection.bbox.size_x = float(x2 - x1)
            detection.bbox.size_y = float(y2 - y1)

            # Class and confidence
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = str(int(box.cls[0]))
            hypothesis.hypothesis.score = float(box.conf[0])
            detection.results.append(hypothesis)

            detection_array.detections.append(detection)

        self.detection_pub.publish(detection_array)
        self.get_logger().info(
            f'Published {len(detection_array.detections)} detections'
        )

def main():
    rclpy.init()
    node = YOLOv8DetectorNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Class Filtering for Robotics

For robotics applications, we typically filter detections to relevant classes:

- **Navigation**: person, chair, table, door
- **Warehouse**: pallet, forklift, box, person
- **Domestic**: bottle, cup, book, remote

This reduces false positives and improves performance by skipping irrelevant classes during inference.

## Semantic Segmentation

**Semantic segmentation** assigns a class label to every pixel in an image, providing dense scene understanding. For mobile robots, segmentation is used to:

- Identify **free space** (drivable/walkable regions)
- Detect **obstacles** (walls, furniture, people)
- Recognize **terrain types** (grass, pavement, gravel)

### Segformer and DeepLabV3

Two popular segmentation architectures:

1. **Segformer**: Transformer-based, ~40-60 FPS on Jetson Orin, mIoU 50-55% on ADE20K. Excellent for outdoor navigation.
2. **DeepLabV3+**: CNN-based with atrous convolution, ~30-50 FPS, mIoU 45-50%. Better for indoor scenes.

### Free Space Detection for Navigation

Here's a segmentation node that publishes a binary occupancy grid for free space:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import OccupancyGrid
from cv_bridge import CvBridge
import torch
from transformers import SegformerForSemanticSegmentation, SegformerImageProcessor
import numpy as np

class FreeSpaceDetector(Node):
    """Semantic segmentation for free space detection"""

    def __init__(self):
        super().__init__('free_space_detector')

        # Load Segformer model
        self.processor = SegformerImageProcessor.from_pretrained(
            "nvidia/segformer-b0-finetuned-ade-512-512"
        )
        self.model = SegformerForSemanticSegmentation.from_pretrained(
            "nvidia/segformer-b0-finetuned-ade-512-512"
        )
        self.model.eval()

        # Free space class IDs (road=6, sidewalk=11, floor=3)
        self.free_space_classes = [3, 6, 11]

        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(
            Image, '/camera/color/image_raw', self.image_callback, 10
        )
        self.grid_pub = self.create_publisher(
            OccupancyGrid, '/free_space_grid', 10
        )

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')

        # Preprocess and run inference
        inputs = self.processor(images=cv_image, return_tensors="pt")
        with torch.no_grad():
            outputs = self.model(**inputs)
            logits = outputs.logits  # Shape: (1, num_classes, H, W)

        # Upsample to original size
        seg_map = torch.nn.functional.interpolate(
            logits,
            size=cv_image.shape[:2],
            mode='bilinear',
            align_corners=False
        ).argmax(dim=1)[0].cpu().numpy()

        # Create binary free space mask
        free_space_mask = np.isin(seg_map, self.free_space_classes).astype(np.uint8)

        # Convert to OccupancyGrid (0=free, 100=occupied, -1=unknown)
        grid = OccupancyGrid()
        grid.header = msg.header
        grid.info.resolution = 0.05  # 5cm per pixel
        grid.info.width = free_space_mask.shape[1]
        grid.info.height = free_space_mask.shape[0]
        grid.data = ((1 - free_space_mask) * 100).flatten().tolist()

        self.grid_pub.publish(grid)

def main():
    rclpy.init()
    node = FreeSpaceDetector()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 3D Perception from Depth

RGB-D cameras (e.g., Intel RealSense, Azure Kinect) provide both color images and depth maps, enabling **3D object localization**. This is critical for manipulation tasks where the robot needs to know not just "where is the object in the image" but "where is the object in 3D space relative to the robot."

### Point Cloud Generation

Given an RGB-D image, we can generate a 3D point cloud:

```python
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
import struct

def rgbd_to_pointcloud(rgb_image, depth_image, camera_intrinsics):
    """
    Convert RGB-D image to point cloud

    Args:
        rgb_image: (H, W, 3) numpy array
        depth_image: (H, W) numpy array in millimeters
        camera_intrinsics: dict with fx, fy, cx, cy
    """
    fx, fy, cx, cy = camera_intrinsics['fx'], camera_intrinsics['fy'], \
                     camera_intrinsics['cx'], camera_intrinsics['cy']

    H, W = depth_image.shape
    points = []

    for v in range(H):
        for u in range(W):
            z = depth_image[v, u] / 1000.0  # Convert mm to meters

            if z == 0:  # Invalid depth
                continue

            # Backproject to 3D
            x = (u - cx) * z / fx
            y = (v - cy) * z / fy

            r, g, b = rgb_image[v, u]
            rgb_packed = struct.unpack('f', struct.pack('I', (r << 16 | g << 8 | b)))[0]

            points.append([x, y, z, rgb_packed])

    return np.array(points)
```

### 3D Bounding Box Estimation

Combining 2D detection with depth, we can estimate 3D bounding boxes:

```python
def estimate_3d_bbox(detection_2d, depth_image, camera_intrinsics):
    """
    Estimate 3D bounding box from 2D detection and depth

    Returns:
        center_3d: (x, y, z) in meters
        size_3d: (width, height, depth) in meters
    """
    # Extract 2D bbox
    x_center = int(detection_2d.bbox.center.position.x)
    y_center = int(detection_2d.bbox.center.position.y)
    width = int(detection_2d.bbox.size_x)
    height = int(detection_2d.bbox.size_y)

    # Get depth at bbox center (median for robustness)
    x1 = max(0, x_center - width // 2)
    x2 = min(depth_image.shape[1], x_center + width // 2)
    y1 = max(0, y_center - height // 2)
    y2 = min(depth_image.shape[0], y_center + height // 2)

    depth_patch = depth_image[y1:y2, x1:x2]
    depth_patch = depth_patch[depth_patch > 0]  # Remove invalid

    if len(depth_patch) == 0:
        return None

    z = np.median(depth_patch) / 1000.0  # mm to meters

    # Backproject center to 3D
    fx, fy, cx, cy = camera_intrinsics['fx'], camera_intrinsics['fy'], \
                     camera_intrinsics['cx'], camera_intrinsics['cy']
    x = (x_center - cx) * z / fx
    y = (y_center - cy) * z / fy

    # Estimate 3D size (rough approximation)
    width_3d = (width / fx) * z
    height_3d = (height / fy) * z
    depth_3d = 0.3  # Assume 30cm depth for person/object

    return {
        'center': (x, y, z),
        'size': (width_3d, height_3d, depth_3d)
    }
```

### 6D Pose Estimation

For manipulation, we often need the full **6D pose** (position + orientation) of objects. Methods include:

- **DOPE** (Deep Object Pose Estimation): CNN-based, works on RGB images
- **PVNet**: Keypoint-based 6D pose estimation
- **FoundationPose**: Foundation model for 6D pose (novel objects)

These are integrated in Isaac ROS (`isaac_ros_dope`) for GPU acceleration.

## Perception for Navigation

Perception pipelines must integrate with the Nav2 navigation stack to enable dynamic obstacle avoidance and environment-aware planning.

### Dynamic Obstacle Detection

Use object detection to identify moving obstacles (people, vehicles) and publish them to Nav2's costmap:

```python
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PointStamped

class DynamicObstaclePublisher(Node):
    """Publish detected people as dynamic obstacles"""

    def __init__(self):
        super().__init__('dynamic_obstacle_publisher')

        self.detection_sub = self.create_subscription(
            Detection2DArray, '/detections', self.detection_callback, 10
        )
        self.costmap_pub = self.create_publisher(
            OccupancyGrid, '/dynamic_obstacles_costmap', 10
        )

    def detection_callback(self, msg):
        # Create costmap with detected obstacles
        costmap = OccupancyGrid()
        costmap.header = msg.header
        costmap.info.resolution = 0.05  # 5cm resolution
        costmap.info.width = 200  # 10m x 10m map
        costmap.info.height = 200
        costmap.data = [0] * (200 * 200)  # Initialize as free

        for detection in msg.detections:
            # Mark detected person as occupied in costmap
            # (This is simplified; in practice, use 3D position)
            x_pixel = int(detection.bbox.center.position.x / costmap.info.resolution)
            y_pixel = int(detection.bbox.center.position.y / costmap.info.resolution)

            # Inflate obstacle (5x5 grid)
            for dx in range(-2, 3):
                for dy in range(-2, 3):
                    idx = (y_pixel + dy) * 200 + (x_pixel + dx)
                    if 0 <= idx < len(costmap.data):
                        costmap.data[idx] = 100  # Occupied

        self.costmap_pub.publish(costmap)
```

### Person Tracking and Following

For assistive robots, tracking a designated person is a common task:

```python
class PersonTracker(Node):
    """Track and follow a designated person"""

    def __init__(self):
        super().__init__('person_tracker')
        self.target_person_id = None
        self.last_detection_time = None

        self.detection_sub = self.create_subscription(
            Detection2DArray, '/detections', self.track_callback, 10
        )
        self.cmd_vel_pub = self.create_publisher(
            Twist, '/cmd_vel', 10
        )

    def track_callback(self, msg):
        if not msg.detections:
            return

        # Find closest person (simple heuristic)
        closest_detection = min(
            msg.detections,
            key=lambda d: d.bbox.size_x * d.bbox.size_y  # Largest = closest
        )

        # Compute tracking command (proportional control)
        image_center = 320  # Assuming 640x480 camera
        x_center = closest_detection.bbox.center.position.x
        error = x_center - image_center

        cmd = Twist()
        cmd.linear.x = 0.3  # Move forward at 0.3 m/s
        cmd.angular.z = -0.002 * error  # Turn to center person

        self.cmd_vel_pub.publish(cmd)
```

## Real-Time Performance Optimization

To achieve &lt;100ms latency for real-time control:

1. **Use GPU acceleration**: Run models on NVIDIA Jetson or GPUs
2. **Model optimization**: Convert to TensorRT (2-3x speedup)
3. **Resolution trade-offs**: Use 640x480 instead of 1920x1080 (4x faster)
4. **Batch processing**: If latency allows, batch multiple frames
5. **Asynchronous inference**: Run perception in separate thread from control

## Best Practices

1. **Validate models on real data**: Test on robot-collected images, not just benchmarks
2. **Monitor inference time**: Log latency for each pipeline stage
3. **Use confidence thresholds**: Filter low-confidence detections to reduce false positives
4. **Fuse multiple sensors**: Combine camera + LiDAR for robust obstacle detection
5. **Implement fallback behaviors**: If perception fails, stop safely

## Summary

Key takeaways from this chapter:

- **Object detection** with YOLOv8/RT-DETR enables real-time identification of objects at 30-80 FPS
- **Semantic segmentation** provides dense scene understanding for free space detection and terrain classification
- **3D perception** from RGB-D cameras enables accurate object localization for manipulation
- **Integration with Nav2** allows perception-based dynamic obstacle avoidance
- **Real-time optimization** through GPU acceleration and TensorRT is essential for high-frequency control loops

These perception pipelines form the foundation for intelligent Physical AI systems that can understand and interact with complex, dynamic environments.

## Review Questions

1. What are the key differences between YOLOv8 and RT-DETR, and when would you choose one over the other?
2. How does semantic segmentation differ from object detection, and what are the use cases for each in robotics?
3. Explain the process of generating a 3D point cloud from an RGB-D image using camera intrinsics.
4. Why is 6D pose estimation more challenging than 3D bounding box estimation?
5. Describe how you would integrate a person detection pipeline with Nav2 to enable dynamic obstacle avoidance.
6. What are the trade-offs between using higher resolution images vs. faster inference in real-time perception?
7. How would you debug a perception pipeline that has high latency (>200ms)?

## Hands-On Exercises

### Exercise 1: YOLOv8 Object Detection

**Objective**: Deploy YOLOv8 on a ROS 2 robot and visualize detections in RViz.

**Steps**:
1. Install Ultralytics YOLOv8: `pip install ultralytics`
2. Implement the YOLOv8DetectorNode from this chapter
3. Run the node with a camera feed: `ros2 run perception yolov8_detector`
4. Visualize detections in RViz using `vision_msgs/Detection2DArray`

**Expected Outcome**: Real-time object detection at 30+ FPS with bounding boxes displayed.

### Exercise 2: Free Space Segmentation

**Objective**: Build a semantic segmentation pipeline to detect drivable surfaces.

**Steps**:
1. Fine-tune Segformer on custom indoor/outdoor dataset
2. Implement the FreeSpaceDetector node
3. Integrate with Nav2's costmap_2d for free space updates
4. Test navigation in cluttered environment

**Expected Outcome**: Robot successfully navigates using segmentation-based free space detection.

### Exercise 3: 3D Object Localization

**Objective**: Use RGB-D camera to estimate 3D positions of objects.

**Steps**:
1. Set up Intel RealSense D435 camera in ROS 2
2. Combine YOLOv8 detections with depth images
3. Implement `estimate_3d_bbox` function from this chapter
4. Publish 3D bounding boxes as `visualization_msgs/MarkerArray` in RViz

**Expected Outcome**: Accurate 3D localization of objects within 5cm error.

## Further Reading

- **YOLOv8 Documentation**: [https://docs.ultralytics.com/](https://docs.ultralytics.com/)
- **RT-DETR Paper**: "DETRs Beat YOLOs on Real-time Object Detection" (2023)
- **Segformer Paper**: "SegFormer: Simple and Efficient Design for Semantic Segmentation with Transformers" (2021)
- **DOPE (6D Pose)**: [https://github.com/NVlabs/Deep_Object_Pose](https://github.com/NVlabs/Deep_Object_Pose)
- **ROS 2 Perception Tutorials**: [https://docs.ros.org/en/humble/Tutorials/Advanced/Computer-Vision.html](https://docs.ros.org/en/humble/Tutorials/Advanced/Computer-Vision.html)

---

**Previous**: [Chapter 13 - Visual SLAM and Nav2](ch13-vslam-nav2.md)
**Next**: [Chapter 15 - Reinforcement Learning and Sim-to-Real](../week-10/ch15-rl-sim2real.md)
