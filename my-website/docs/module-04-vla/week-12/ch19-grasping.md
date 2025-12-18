---
id: ch19-grasping
title: Robotic Grasping
sidebar_label: Robotic Grasping
sidebar_position: 20
---

# Robotic Grasping

## Learning Objectives

By the end of this chapter, you will be able to:

- Design and implement grasp planning algorithms that generate stable grasps for diverse object geometries
- Analyze force closure properties to ensure grasps can resist external disturbances
- Integrate tactile sensing for feedback-driven manipulation and grasp adjustment
- Implement manipulation primitives for pick-and-place, reorientation, and assembly tasks
- Apply learning-based approaches to improve grasping performance through data-driven methods

## Introduction

Robotic grasping is the fundamental skill that enables robots to physically interact with their environment, manipulate objects, and perform useful tasks. From industrial assembly lines to household assistance, the ability to reliably grasp and manipulate objects of varying shapes, sizes, materials, and weights is essential for practical robot deployment. Grasping involves complex interactions between perception, planning, control, and mechanical design, making it one of the most challenging problems in robotics.

A successful grasp must satisfy multiple constraints simultaneously: the gripper must make contact with the object at appropriate locations, apply sufficient force to prevent slipping, avoid collisions with the environment, and enable subsequent manipulation actions. These requirements are complicated by uncertainties in object pose estimation, surface friction properties, and sensor noise. Classical approaches to grasping rely on analytical models of contact mechanics and optimization-based planning, while modern learning-based methods leverage large datasets and neural networks to discover effective grasping strategies.

This chapter builds on URDF modeling (Chapter 17) and manipulation concepts to provide a comprehensive treatment of robotic grasping. We'll cover grasp planning algorithms that search for optimal contact configurations, force closure analysis that ensures stability, tactile sensing for closed-loop control, manipulation primitives that compose complex behaviors, and learning-based approaches that improve through experience. These techniques are critical for Vision-Language-Action (VLA) models that must translate high-level commands into precise manipulation actions.

## 1. Grasp Planning Algorithms

### Analytical Grasp Synthesis

Grasp planning algorithms determine where and how a gripper should make contact with an object to achieve a stable grasp. Analytical approaches model the object geometry, gripper kinematics, and contact physics to compute candidate grasps. A common method is to sample points on the object surface, compute surface normals, and evaluate grasp quality metrics such as force closure, distance from center of mass, and collision avoidance.

For a parallel-jaw gripper, the grasp planning problem simplifies to finding two opposing contact points on the object surface such that:
1. The line connecting the contacts passes through or near the object's center of mass
2. The contact normals are anti-parallel (opposite directions)
3. The gripper width accommodates the object
4. No collisions occur with the object or environment

Example grasp sampling for a cylinder:

```python
import numpy as np

class GraspSampler:
    def __init__(self, gripper_width_min=0.02, gripper_width_max=0.12):
        self.gripper_width_min = gripper_width_min
        self.gripper_width_max = gripper_width_max

    def sample_cylinder_grasps(self, radius, height, num_samples=100):
        """
        Sample parallel-jaw grasps around a cylinder.

        Args:
            radius: Cylinder radius (m)
            height: Cylinder height (m)
            num_samples: Number of grasp candidates to generate

        Returns:
            List of grasp poses (position, orientation, width)
        """
        grasps = []

        for i in range(num_samples):
            # Random approach angle around cylinder
            theta = np.random.uniform(0, 2 * np.pi)

            # Random height along cylinder
            z = np.random.uniform(-height / 2, height / 2)

            # Grasp position on cylinder surface
            x = radius * np.cos(theta)
            y = radius * np.sin(theta)
            position = np.array([x, y, z])

            # Grasp orientation (gripper z-axis points toward cylinder center)
            approach_vector = -np.array([x, y, 0]) / np.linalg.norm([x, y, 0])
            # Binormal (perpendicular to approach and world z)
            binormal = np.array([0, 0, 1])
            # Normal (completes right-handed frame)
            normal = np.cross(approach_vector, binormal)

            # Rotation matrix from gripper frame to world frame
            rotation = np.column_stack([normal, binormal, approach_vector])

            # Gripper width (diameter of cylinder)
            width = 2 * radius

            if self.gripper_width_min <= width <= self.gripper_width_max:
                grasps.append({
                    'position': position,
                    'rotation': rotation,
                    'width': width,
                    'score': self.evaluate_grasp(position, rotation, width)
                })

        return sorted(grasps, key=lambda g: g['score'], reverse=True)

    def evaluate_grasp(self, position, rotation, width):
        """
        Score a grasp based on heuristics.
        Higher score is better.
        """
        # Prefer grasps near vertical axis (z-axis)
        vertical_alignment = abs(position[2])

        # Prefer wider grasps (more stable)
        width_score = width / self.gripper_width_max

        # Combine scores (weights can be tuned)
        return 0.5 * vertical_alignment + 0.5 * width_score
```

This sampler generates grasp candidates by randomly placing the gripper around the cylinder and evaluating each grasp based on stability heuristics. More sophisticated methods use volumetric representations (voxel grids, point clouds) or shape primitives (boxes, cylinders, spheres) to handle arbitrary object geometries.

### Sampling-Based Grasp Planning

For complex objects, sampling-based methods generate many candidate grasps and rank them using quality metrics. A popular approach is GraspIt!, which represents objects as triangle meshes and searches for stable grasps by:
1. Placing gripper at random poses relative to the object
2. Closing gripper fingers until contact is made
3. Computing grasp quality (force closure, epsilon quality)
4. Iteratively refining grasp pose using optimization

Example sampling-based grasp planner interface:

```python
from scipy.spatial.transform import Rotation

class SamplingGraspPlanner:
    def __init__(self, object_mesh, gripper_model):
        self.object_mesh = object_mesh
        self.gripper_model = gripper_model

    def plan_grasps(self, num_samples=1000, top_k=10):
        """
        Generate and rank grasp candidates.

        Args:
            num_samples: Number of grasps to sample
            top_k: Number of top grasps to return

        Returns:
            List of top-ranked grasps
        """
        candidates = []

        for _ in range(num_samples):
            # Sample random grasp pose
            grasp_pose = self.sample_grasp_pose()

            # Check collision
            if self.check_collision(grasp_pose):
                continue

            # Compute contacts
            contacts = self.compute_contacts(grasp_pose)

            if len(contacts) < 2:
                continue  # Need at least 2 contacts

            # Evaluate grasp quality
            quality = self.evaluate_force_closure(contacts)

            candidates.append({
                'pose': grasp_pose,
                'contacts': contacts,
                'quality': quality
            })

        # Return top-k grasps
        candidates.sort(key=lambda g: g['quality'], reverse=True)
        return candidates[:top_k]

    def sample_grasp_pose(self):
        """Sample random SE(3) grasp pose near object."""
        # Random position near object center of mass
        com = self.object_mesh.center_mass
        position = com + np.random.randn(3) * 0.05  # 5cm std dev

        # Random orientation
        rotation = Rotation.random().as_matrix()

        return {'position': position, 'rotation': rotation}

    def check_collision(self, grasp_pose):
        """Check if gripper collides with object at given pose."""
        # Transform gripper mesh to grasp pose
        gripper_mesh = self.gripper_model.transform(
            grasp_pose['position'],
            grasp_pose['rotation']
        )
        # Check intersection with object mesh
        return self.object_mesh.intersects(gripper_mesh)

    def compute_contacts(self, grasp_pose):
        """Compute contact points between gripper and object."""
        # Simulate gripper closure and find contact points
        # (Simplified: returns contact positions and normals)
        contacts = []
        # ... contact detection logic ...
        return contacts

    def evaluate_force_closure(self, contacts):
        """
        Compute force closure quality metric.
        Returns value in [0, 1], higher is better.
        """
        # Simplified force closure check (see next section for details)
        if len(contacts) < 3:
            return 0.0  # Need at least 3 contacts for 3D force closure

        # Compute grasp matrix and evaluate rank
        G = self.compute_grasp_matrix(contacts)
        # Compute epsilon quality (largest perturbation wrench that can be resisted)
        epsilon = self.compute_epsilon_quality(G)
        return epsilon
```

Sampling-based planners are flexible and can handle arbitrary object geometries, but they require many samples to find high-quality grasps. Modern GPU-accelerated simulators like NVIDIA Isaac Sim can evaluate thousands of grasp candidates in parallel, enabling real-time grasp planning.

### Database-Driven Grasp Planning

An alternative to analytical planning is to use pre-computed grasp databases. These databases store successful grasps for common object categories (mugs, bottles, boxes), indexed by shape descriptors. At runtime, the planner:
1. Computes shape descriptor for the target object
2. Retrieves similar objects from the database
3. Transfers grasps from database objects to the target
4. Refines grasps using local optimization

This approach is fast and leverages prior successful grasps, but requires a large, diverse database and may not generalize to novel object categories.

## 2. Force Closure Analysis

### Contact Mechanics and Friction Cones

A grasp is in force closure if it can resist arbitrary external wrenches (forces and torques) applied to the object. This requires that the contact forces can balance any disturbance. For a single contact point with Coulomb friction, the feasible contact forces lie within a friction cone:

**f ∈ FC = \{f : ||f\_tangent|| ≤ μ f\_normal, f\_normal ≥ 0\}**

where μ is the coefficient of friction, f_normal is the force component along the surface normal, and f_tangent is the tangential component. For hard contact (no friction), the cone degenerates to a ray along the normal. For soft contact with friction, the cone opens with angle α = arctan(μ).

Example friction cone computation:

```python
def compute_friction_cone(contact_normal, mu=0.5, num_edges=4):
    """
    Compute friction cone basis vectors.

    Args:
        contact_normal: 3D unit vector normal to contact surface
        mu: Coefficient of friction
        num_edges: Number of cone edge vectors to generate

    Returns:
        Array of friction cone edge vectors (num_edges, 3)
    """
    # Find two orthogonal tangent vectors
    if abs(contact_normal[2]) < 0.9:
        tangent1 = np.cross(contact_normal, np.array([0, 0, 1]))
    else:
        tangent1 = np.cross(contact_normal, np.array([1, 0, 0]))
    tangent1 /= np.linalg.norm(tangent1)
    tangent2 = np.cross(contact_normal, tangent1)
    tangent2 /= np.linalg.norm(tangent2)

    # Generate cone edge vectors
    cone_edges = []
    for i in range(num_edges):
        angle = 2 * np.pi * i / num_edges
        tangent = np.cos(angle) * tangent1 + np.sin(angle) * tangent2
        edge = contact_normal + mu * tangent
        edge /= np.linalg.norm(edge)
        cone_edges.append(edge)

    return np.array(cone_edges)
```

For a grasp with n contact points, the combined friction cone is the Minkowski sum of individual cones. A grasp achieves force closure if the origin of the wrench space lies in the interior of the convex hull of the friction cone wrenches.

### Grasp Matrix and Wrench Space

The grasp matrix G maps contact forces to object wrenches (6D force-torque vectors). For n contact points with position r_i and friction cone edges f_ij, the grasp matrix is:

**G = [f_11, ..., f_1k, f_21, ..., f_nk]**
**  = [f_ij]**
**    [r_i × f_ij]**

Each column represents a unit wrench generated by a friction cone edge. The grasp achieves force closure if the origin is in the interior of the convex hull of G's columns, which can be tested using linear programming or computational geometry algorithms.

Example grasp matrix computation:

```python
def compute_grasp_matrix(contacts, mu=0.5, num_edges=4):
    """
    Compute grasp matrix from contact points.

    Args:
        contacts: List of contact dicts with 'position' and 'normal' keys
        mu: Coefficient of friction
        num_edges: Number of friction cone edges per contact

    Returns:
        Grasp matrix G (6, n*num_edges)
    """
    G_columns = []

    for contact in contacts:
        position = contact['position']
        normal = contact['normal']

        # Compute friction cone for this contact
        cone_edges = compute_friction_cone(normal, mu, num_edges)

        for edge in cone_edges:
            # Force component
            force = edge

            # Torque component (r × f)
            torque = np.cross(position, force)

            # Wrench (6D: force + torque)
            wrench = np.concatenate([force, torque])
            G_columns.append(wrench)

    return np.column_stack(G_columns)


def check_force_closure(G, tolerance=1e-6):
    """
    Check if grasp matrix G represents a force closure grasp.

    Args:
        G: Grasp matrix (6, m) where m = n_contacts * n_edges
        tolerance: Numerical tolerance for interior point test

    Returns:
        Boolean indicating force closure
    """
    from scipy.optimize import linprog

    # Check if origin is in interior of convex hull of G's columns
    # Formulate as LP: min c^T λ subject to G λ = 0, λ ≥ 0, sum(λ) = 1
    # If optimal objective is close to zero, origin is in convex hull

    m = G.shape[1]
    c = np.zeros(m)  # Dummy objective

    # Equality constraint: G λ = 0 (origin is in span)
    A_eq = G
    b_eq = np.zeros(6)

    # Inequality constraint: λ ≥ 0, sum(λ) = 1
    A_ub = -np.eye(m)
    b_ub = np.zeros(m)

    # Additional constraint: sum(λ) = 1
    A_eq = np.vstack([A_eq, np.ones(m)])
    b_eq = np.append(b_eq, 1)

    result = linprog(c, A_ub=A_ub, b_ub=b_ub, A_eq=A_eq, b_eq=b_eq, method='highs')

    return result.success and np.linalg.norm(G @ result.x) < tolerance
```

This force closure test checks whether the origin lies in the convex hull of the grasp matrix columns, indicating that any external wrench can be balanced by contact forces within the friction cones.

### Grasp Quality Metrics

Beyond binary force closure, quality metrics quantify how robust a grasp is to disturbances. Common metrics include:

1. **Epsilon quality (ε)**: Largest magnitude of uniform wrench that can be resisted. Computed as the distance from the origin to the boundary of the grasp wrench space.

2. **Volume quality**: Volume of the grasp wrench space, normalized by a reference ellipsoid. Larger volume indicates more disturbances can be resisted.

3. **Ferrari-Canny metric**: Minimum effort to generate a unit wrench in any direction. Related to the smallest singular value of the grasp matrix.

Example epsilon quality computation:

```python
def compute_epsilon_quality(G):
    """
    Compute epsilon quality (largest uniform wrench that can be resisted).

    Args:
        G: Grasp matrix (6, m)

    Returns:
        Epsilon quality (scalar)
    """
    from scipy.spatial import ConvexHull

    # Normalize grasp matrix columns to unit length
    G_normalized = G / np.linalg.norm(G, axis=0, keepdims=True)

    # Compute convex hull of normalized columns
    hull = ConvexHull(G_normalized.T)

    # Epsilon is distance from origin to closest facet
    epsilon = np.inf
    for eq in hull.equations:
        # Facet equation: a^T x + b = 0 where ||a|| = 1
        a = eq[:-1]
        b = eq[-1]
        # Distance from origin to facet
        distance = abs(b)
        epsilon = min(epsilon, distance)

    return epsilon
```

Higher epsilon quality indicates a more robust grasp that can resist larger disturbances before the object slips.

## 3. Tactile Sensing

### Tactile Sensor Technologies

Tactile sensors provide local force, pressure, or deformation measurements at the gripper-object interface. They enable feedback-driven manipulation, slip detection, and adaptive grasping. Common tactile sensor technologies include:

1. **Resistive sensors**: Measure resistance change under pressure (e.g., piezoresistive, force-sensing resistors). Simple and low-cost but limited spatial resolution.

2. **Capacitive sensors**: Measure capacitance change due to deformation. Higher resolution than resistive sensors, used in GelSight and similar optical tactile sensors.

3. **Optical sensors**: Camera-based sensors that track surface deformation using markers or illumination patterns. Very high resolution (submillimeter) and can measure 3D deformation fields. Examples: GelSight, TacTip, DIGIT.

4. **Piezoelectric sensors**: Measure dynamic force changes. Good for vibration and slip detection but not static forces.

For robotic grasping, optical tactile sensors like GelSight provide rich information about contact geometry, forces, and slip:

```python
import numpy as np
import cv2

class GelSightSensor:
    def __init__(self, camera_id=0, resolution=(640, 480)):
        self.cap = cv2.VideoCapture(camera_id)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, resolution[0])
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, resolution[1])
        self.reference_frame = None

    def capture_reference(self):
        """Capture reference image (no contact)."""
        ret, frame = self.cap.read()
        if ret:
            self.reference_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        return ret

    def get_contact_image(self):
        """Capture current contact image."""
        ret, frame = self.cap.read()
        if ret:
            return cv2.cvtColor(frame, cv2.COLOR_BGR_GRAY)
        return None

    def compute_depth_map(self, contact_image):
        """
        Compute depth map from contact image.

        Args:
            contact_image: Grayscale contact image

        Returns:
            Depth map (normalized displacement field)
        """
        if self.reference_frame is None:
            raise ValueError("Reference frame not captured")

        # Compute difference from reference
        diff = cv2.absdiff(contact_image, self.reference_frame)

        # Apply Gaussian blur to reduce noise
        diff_blurred = cv2.GaussianBlur(diff, (5, 5), 0)

        # Normalize to [0, 1]
        depth_map = diff_blurred / 255.0

        return depth_map

    def detect_slip(self, depth_map_current, depth_map_previous, threshold=0.05):
        """
        Detect slip by comparing consecutive depth maps.

        Args:
            depth_map_current: Current depth map
            depth_map_previous: Previous depth map
            threshold: Motion threshold for slip detection

        Returns:
            Boolean indicating slip detected
        """
        # Compute optical flow between depth maps
        flow = cv2.calcOpticalFlowFarneback(
            (depth_map_previous * 255).astype(np.uint8),
            (depth_map_current * 255).astype(np.uint8),
            None, 0.5, 3, 15, 3, 5, 1.2, 0
        )

        # Compute flow magnitude
        flow_magnitude = np.linalg.norm(flow, axis=2)

        # Slip detected if mean flow exceeds threshold
        mean_flow = np.mean(flow_magnitude)
        return mean_flow > threshold
```

This GelSight interface captures depth maps from contact deformation and detects slip by tracking optical flow between consecutive frames. Slip detection enables reactive behaviors like increasing grip force or adjusting grasp pose.

### Force/Torque Sensing

Wrist-mounted force/torque (F/T) sensors measure net forces and torques applied to the gripper. Unlike tactile sensors, F/T sensors provide global measurements but don't resolve individual contact locations. They are useful for detecting contact events, measuring object weight, and impedance control.

Example F/T sensor integration:

```python
from geometry_msgs.msg import WrenchStamped

class ForceTorqueSensor:
    def __init__(self, node):
        self.node = node
        self.wrench = None
        self.bias = np.zeros(6)  # Force and torque bias

        # Subscribe to F/T sensor topic
        self.subscriber = node.create_subscription(
            WrenchStamped,
            '/ft_sensor/wrench',
            self.wrench_callback,
            10
        )

    def wrench_callback(self, msg):
        """Store latest wrench measurement."""
        force = np.array([msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z])
        torque = np.array([msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z])
        self.wrench = np.concatenate([force, torque]) - self.bias

    def calibrate(self, num_samples=100):
        """
        Calibrate sensor by averaging measurements (no contact).
        """
        measurements = []
        rate = self.node.create_rate(100)  # 100 Hz

        for _ in range(num_samples):
            if self.wrench is not None:
                measurements.append(self.wrench + self.bias)
            rate.sleep()

        self.bias = np.mean(measurements, axis=0)
        self.node.get_logger().info(f'F/T sensor calibrated with bias: {self.bias}')

    def detect_contact(self, force_threshold=1.0):
        """
        Detect contact based on force magnitude.

        Args:
            force_threshold: Minimum force (N) to consider contact

        Returns:
            Boolean indicating contact detected
        """
        if self.wrench is None:
            return False

        force_magnitude = np.linalg.norm(self.wrench[:3])
        return force_magnitude > force_threshold
```

F/T sensors are commonly used in compliant manipulation strategies where the robot adjusts its motion based on sensed forces, enabling gentle contact and robust insertion tasks.

## 4. Manipulation Primitives

### Pick and Place

Pick-and-place is the fundamental manipulation primitive consisting of four phases:
1. **Approach**: Move gripper to pre-grasp pose above object
2. **Grasp**: Close gripper and secure object
3. **Lift**: Raise object to clear workspace
4. **Place**: Move to target location and release

Example pick-and-place implementation:

```python
class PickAndPlaceController:
    def __init__(self, move_group, gripper):
        self.move_group = move_group  # MoveIt move group
        self.gripper = gripper  # Gripper controller

    def pick(self, grasp_pose, approach_distance=0.1):
        """
        Execute pick primitive.

        Args:
            grasp_pose: Target grasp pose (SE(3))
            approach_distance: Distance to retract before approach (m)

        Returns:
            Success boolean
        """
        # 1. Move to pre-grasp pose (approach from above)
        pre_grasp_pose = self.compute_pre_grasp_pose(grasp_pose, approach_distance)
        if not self.move_group.go(pre_grasp_pose, wait=True):
            return False

        # 2. Open gripper
        self.gripper.open()

        # 3. Approach grasp pose
        if not self.move_group.go(grasp_pose, wait=True):
            return False

        # 4. Close gripper
        self.gripper.close()

        # 5. Lift object
        lift_pose = self.compute_lift_pose(grasp_pose, lift_distance=0.15)
        if not self.move_group.go(lift_pose, wait=True):
            self.gripper.open()  # Release if lift fails
            return False

        return True

    def place(self, target_pose, approach_distance=0.1):
        """
        Execute place primitive.

        Args:
            target_pose: Target placement pose (SE(3))
            approach_distance: Distance to approach before placing (m)

        Returns:
            Success boolean
        """
        # 1. Move to pre-place pose
        pre_place_pose = self.compute_pre_grasp_pose(target_pose, approach_distance)
        if not self.move_group.go(pre_place_pose, wait=True):
            return False

        # 2. Lower to place pose
        if not self.move_group.go(target_pose, wait=True):
            return False

        # 3. Open gripper (release object)
        self.gripper.open()

        # 4. Retract
        retract_pose = self.compute_pre_grasp_pose(target_pose, approach_distance)
        self.move_group.go(retract_pose, wait=True)

        return True

    def compute_pre_grasp_pose(self, grasp_pose, distance):
        """Compute pre-grasp pose by retracting along approach direction."""
        approach_vector = grasp_pose['rotation'][:, 2]  # z-axis of gripper
        pre_grasp_position = grasp_pose['position'] - distance * approach_vector
        return {'position': pre_grasp_position, 'rotation': grasp_pose['rotation']}

    def compute_lift_pose(self, grasp_pose, lift_distance):
        """Compute lift pose by moving vertically upward."""
        lift_position = grasp_pose['position'] + np.array([0, 0, lift_distance])
        return {'position': lift_position, 'rotation': grasp_pose['rotation']}
```

This controller sequences motion primitives (move, open, close, lift) to perform reliable pick-and-place operations. Robustness can be improved with force feedback, slip detection, and error recovery strategies.

### In-Hand Manipulation

In-hand manipulation reorients an object within the gripper without placing it down. Techniques include:

1. **Finger gaiting**: Alternately releasing and re-grasping with individual fingers to walk around the object
2. **Pivoting**: Rotating object about a fixed contact point
3. **Rolling**: Moving object by coordinating finger joint motions

Example pivoting controller:

```python
class PivotController:
    def __init__(self, gripper):
        self.gripper = gripper

    def pivot(self, angle, pivot_finger='left'):
        """
        Pivot object by rotating about one contact point.

        Args:
            angle: Desired rotation angle (radians)
            pivot_finger: Which finger to keep fixed ('left' or 'right')

        Returns:
            Success boolean
        """
        # 1. Reduce grip force on non-pivot finger
        if pivot_finger == 'left':
            self.gripper.set_finger_force('left', high_force=True)
            self.gripper.set_finger_force('right', high_force=False)
        else:
            self.gripper.set_finger_force('left', high_force=False)
            self.gripper.set_finger_force('right', high_force=True)

        # 2. Move non-pivot finger to create rotation
        if pivot_finger == 'left':
            finger_displacement = self.compute_arc_displacement(angle, radius=0.05)
            self.gripper.move_finger('right', displacement=finger_displacement)
        else:
            finger_displacement = self.compute_arc_displacement(angle, radius=0.05)
            self.gripper.move_finger('left', displacement=finger_displacement)

        # 3. Restore equal grip force
        self.gripper.set_finger_force('left', high_force=True)
        self.gripper.set_finger_force('right', high_force=True)

        return True

    def compute_arc_displacement(self, angle, radius):
        """Compute arc length for given angle and radius."""
        return angle * radius
```

In-hand manipulation enables dexterous tasks like screwdriver manipulation, where the object must be continuously reoriented during use.

## 5. Learning-Based Grasping

### Data-Driven Grasp Synthesis

Learning-based methods train neural networks to predict grasp success directly from sensor data (images, point clouds), bypassing analytical models. A common approach is to train a convolutional neural network (CNN) to predict grasp quality for candidate grasps in a depth image.

Example CNN-based grasp detector:

```python
import torch
import torch.nn as nn

class GraspQualityCNN(nn.Module):
    def __init__(self, input_channels=1):
        super().__init__()

        self.conv_layers = nn.Sequential(
            nn.Conv2d(input_channels, 64, kernel_size=5, padding=2),
            nn.ReLU(),
            nn.MaxPool2d(2),
            nn.Conv2d(64, 128, kernel_size=5, padding=2),
            nn.ReLU(),
            nn.MaxPool2d(2),
            nn.Conv2d(128, 256, kernel_size=3, padding=1),
            nn.ReLU(),
            nn.MaxPool2d(2),
        )

        self.fc_layers = nn.Sequential(
            nn.Linear(256 * 8 * 8, 512),
            nn.ReLU(),
            nn.Dropout(0.5),
            nn.Linear(512, 1),
            nn.Sigmoid()  # Output grasp quality in [0, 1]
        )

    def forward(self, depth_image):
        """
        Predict grasp quality from depth image patch.

        Args:
            depth_image: (B, 1, 64, 64) depth patches around grasp candidates

        Returns:
            Grasp quality scores (B,)
        """
        features = self.conv_layers(depth_image)
        features_flat = features.view(features.size(0), -1)
        quality = self.fc_layers(features_flat)
        return quality.squeeze()


class GraspNetworkTrainer:
    def __init__(self, model, device='cuda'):
        self.model = model.to(device)
        self.device = device
        self.optimizer = torch.optim.Adam(model.parameters(), lr=1e-4)
        self.criterion = nn.BCELoss()  # Binary cross-entropy for success/failure

    def train_epoch(self, dataloader):
        """Train for one epoch."""
        self.model.train()
        total_loss = 0.0

        for depth_patches, labels in dataloader:
            depth_patches = depth_patches.to(self.device)
            labels = labels.to(self.device)

            self.optimizer.zero_grad()
            predictions = self.model(depth_patches)
            loss = self.criterion(predictions, labels)
            loss.backward()
            self.optimizer.step()

            total_loss += loss.item()

        return total_loss / len(dataloader)

    def evaluate(self, dataloader):
        """Evaluate on validation set."""
        self.model.eval()
        correct = 0
        total = 0

        with torch.no_grad():
            for depth_patches, labels in dataloader:
                depth_patches = depth_patches.to(self.device)
                labels = labels.to(self.device)

                predictions = self.model(depth_patches)
                predicted_labels = (predictions > 0.5).float()
                correct += (predicted_labels == labels).sum().item()
                total += labels.size(0)

        return correct / total
```

The network is trained on thousands of grasp attempts labeled with success/failure. At test time, candidate grasps are evaluated by extracting depth patches and feeding them through the network to predict success probability.

### Reinforcement Learning for Grasping

Reinforcement learning (RL) enables robots to learn grasping policies through trial and error. The agent receives rewards for successful grasps and penalties for failures, gradually improving its policy. Deep Q-Networks (DQN) and Proximal Policy Optimization (PPO) are popular RL algorithms for grasping.

Example RL grasp policy structure:

```python
class GraspPolicyNetwork(nn.Module):
    def __init__(self, input_dim, action_dim):
        super().__init__()
        self.network = nn.Sequential(
            nn.Linear(input_dim, 256),
            nn.ReLU(),
            nn.Linear(256, 256),
            nn.ReLU(),
            nn.Linear(256, action_dim)
        )

    def forward(self, state):
        """
        Compute action values for given state.

        Args:
            state: Robot state (joint angles, gripper pose, object features)

        Returns:
            Action values (Q-values for DQN or action logits for PPO)
        """
        return self.network(state)


class GraspEnvironment:
    def __init__(self, simulator):
        self.simulator = simulator
        self.object = None

    def reset(self):
        """Reset environment and return initial state."""
        self.object = self.simulator.spawn_random_object()
        robot_state = self.simulator.get_robot_state()
        object_state = self.simulator.get_object_state(self.object)
        return np.concatenate([robot_state, object_state])

    def step(self, action):
        """
        Execute action and return next state, reward, done.

        Args:
            action: Gripper pose (x, y, z, roll, pitch, yaw)

        Returns:
            next_state, reward, done, info
        """
        # Execute grasp attempt
        success = self.simulator.attempt_grasp(action)

        # Compute reward
        if success:
            reward = 1.0
            done = True
        else:
            reward = -0.1
            done = False

        next_state = self.get_state()
        return next_state, reward, done, {}
```

RL-based grasping can learn complex, non-intuitive strategies and generalize to novel objects, but requires many training episodes (often millions) and careful reward shaping.

### Vision-Language-Action Models for Grasping

Modern VLA models like RT-1, RT-2, and PaLM-E integrate vision, language understanding, and action prediction to enable natural language-commanded grasping. These models are trained on large-scale datasets of robot demonstrations with language annotations, learning to map instructions like "pick up the red cup" to grasp actions.

VLA pipeline:
1. **Vision encoder**: Processes RGB-D images to extract visual features
2. **Language encoder**: Encodes task description ("pick up the red cup")
3. **Fusion module**: Combines vision and language features
4. **Action decoder**: Predicts gripper pose and control commands

Example VLA interface for grasping:

```python
class VLAGraspController:
    def __init__(self, vla_model, camera, robot_controller):
        self.vla_model = vla_model
        self.camera = camera
        self.robot_controller = robot_controller

    def execute_language_command(self, command):
        """
        Execute grasp based on natural language command.

        Args:
            command: Natural language instruction (e.g., "pick up the blue bottle")

        Returns:
            Success boolean
        """
        # 1. Capture current scene
        rgb_image = self.camera.get_rgb()
        depth_image = self.camera.get_depth()

        # 2. Query VLA model for grasp action
        action = self.vla_model.predict(
            rgb=rgb_image,
            depth=depth_image,
            instruction=command
        )

        # 3. Execute predicted grasp
        grasp_pose = {
            'position': action['position'],
            'rotation': action['rotation'],
            'width': action['gripper_width']
        }

        success = self.robot_controller.execute_grasp(grasp_pose)
        return success
```

VLA models enable intuitive, language-based robot control and can generalize to new tasks through few-shot learning or fine-tuning.

## Practical Example

Let's implement a complete grasping pipeline that integrates perception, planning, and execution:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Image
from geometry_msgs.msg import PoseStamped
import numpy as np

class GraspingPipeline(Node):
    def __init__(self):
        super().__init__('grasping_pipeline')

        # Subscribers
        self.depth_sub = self.create_subscription(
            Image, '/camera/depth/image', self.depth_callback, 10
        )

        # Publishers
        self.grasp_pub = self.create_publisher(
            PoseStamped, '/grasp_pose', 10
        )

        # Grasp planner
        self.planner = SamplingGraspPlanner(object_mesh=None, gripper_model=None)

        # Latest depth image
        self.depth_image = None

        self.get_logger().info('Grasping pipeline initialized')

    def depth_callback(self, msg):
        """Store latest depth image."""
        # Convert ROS Image to numpy array
        self.depth_image = np.frombuffer(msg.data, dtype=np.float32).reshape(
            (msg.height, msg.width)
        )

    def segment_object(self, depth_image):
        """
        Segment object from depth image.

        Returns:
            Object point cloud
        """
        # Simple segmentation: cluster points within distance range
        mask = (depth_image > 0.3) & (depth_image < 1.5)  # 30cm to 150cm
        object_points = np.column_stack(np.where(mask))
        return object_points

    def plan_grasp(self, object_points):
        """
        Plan grasp for segmented object.

        Returns:
            Best grasp pose
        """
        # Compute object bounding box
        min_coords = np.min(object_points, axis=0)
        max_coords = np.max(object_points, axis=0)
        center = (min_coords + max_coords) / 2
        size = max_coords - min_coords

        # Simple heuristic: grasp from top, centered
        grasp_pose = PoseStamped()
        grasp_pose.header.frame_id = 'camera_depth_optical_frame'
        grasp_pose.header.stamp = self.get_clock().now().to_msg()
        grasp_pose.pose.position.x = center[1]  # Column (x in camera frame)
        grasp_pose.pose.position.y = center[0]  # Row (y in camera frame)
        grasp_pose.pose.position.z = min_coords[2] - 0.05  # 5cm above object
        grasp_pose.pose.orientation.w = 1.0  # No rotation

        return grasp_pose

    def execute_grasp_pipeline(self):
        """Run complete grasping pipeline."""
        if self.depth_image is None:
            self.get_logger().warn('No depth image available')
            return

        # 1. Segment object
        object_points = self.segment_object(self.depth_image)
        self.get_logger().info(f'Segmented {len(object_points)} object points')

        # 2. Plan grasp
        grasp_pose = self.plan_grasp(object_points)
        self.get_logger().info(f'Planned grasp at position {grasp_pose.pose.position}')

        # 3. Publish grasp pose
        self.grasp_pub.publish(grasp_pose)


def main():
    rclpy.init()
    node = GraspingPipeline()

    # Run pipeline every 2 seconds
    timer = node.create_timer(2.0, node.execute_grasp_pipeline)

    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Key Implementation Points:**
- The pipeline subscribes to depth images, segments objects, plans grasps, and publishes grasp poses for execution
- Object segmentation uses simple depth thresholding; production systems use learned segmentation models
- Grasp planning uses a top-down heuristic; can be replaced with sampling-based or learning-based planners
- The grasp pose is published for a separate motion controller to execute via MoveIt or similar

## Common Challenges and Solutions

### Challenge 1: Grasp Failure Due to Perception Errors

**Problem**: Incorrect object pose estimation from noisy depth images or occlusions leads to grasp failures where the gripper misses the object or collides with surfaces.

**Solution**: Integrate multi-view perception by capturing images from multiple camera viewpoints and fusing them into a complete 3D model. Use learned segmentation and pose estimation networks (e.g., DOPE, PVN3D) that are robust to partial occlusions. Implement grasp verification by checking tactile sensor readings after grasp execution—if no contact is detected, retry with adjusted pose.

### Challenge 2: Slip During Manipulation

**Problem**: Objects slip from the gripper during lifting or transport due to insufficient grip force, smooth surfaces, or unexpected disturbances.

**Solution**: Integrate tactile sensors (GelSight, force/torque sensors) to detect slip in real-time. When slip is detected, immediately increase grip force or adjust grasp pose. Implement closed-loop grip force control that maintains minimum force needed to prevent slip without crushing the object. Use compliant grippers (underactuated, soft robotics) that conform to object shape and distribute forces more evenly.

### Challenge 3: Grasping Novel Objects

**Problem**: Grasp planners trained on specific object datasets fail when encountering novel object geometries, materials, or weights not seen during training.

**Solution**: Use shape-based grasp planning that decomposes objects into primitive shapes (boxes, cylinders, spheres) and applies pre-computed grasps for each primitive. Train grasp networks on highly diverse synthetic datasets generated in simulation (domain randomization). Implement few-shot learning where the robot quickly adapts to new objects after a small number of grasp attempts, updating its model based on success/failure feedback.

## Best Practices

1. **Start with analytical methods before learning**: Implement analytical grasp planning (force closure, contact mechanics) first to understand the fundamentals. Learning-based methods can then augment or replace analytical components where they struggle.

2. **Use simulation for rapid iteration**: Develop and test grasping algorithms in physics simulators (Gazebo, Isaac Sim) where thousands of grasp attempts can be evaluated quickly without hardware wear. Transfer learned policies to real robots using sim-to-real techniques.

3. **Implement grasp recovery strategies**: Not all grasps succeed on the first attempt. Design recovery behaviors: re-grasp after failure, adjust approach angle, or switch to alternative grasp candidates. Robustness comes from graceful failure handling.

4. **Tune grip force carefully**: Too little force causes slips; too much crushes delicate objects. Use force feedback to modulate grip force dynamically based on object properties (estimated weight, compliance) and task requirements.

5. **Collect diverse training data**: If using learning-based approaches, ensure training datasets include diverse objects, poses, lighting conditions, and backgrounds. Data augmentation and domain randomization improve generalization.

## Summary

Key takeaways from this chapter:

- Robotic grasping requires integrating perception (object detection, pose estimation), planning (grasp synthesis, force closure analysis), sensing (tactile, force/torque), and control (manipulation primitives) into a cohesive system
- Analytical grasp planning methods use contact mechanics, friction cone models, and force closure tests to generate provably stable grasps, while learning-based approaches discover effective strategies from data
- Tactile and force/torque sensors provide critical feedback for slip detection, grip force control, and adaptive manipulation, enabling robust interaction with uncertain environments
- Manipulation primitives like pick-and-place and in-hand manipulation decompose complex tasks into reusable building blocks that can be sequenced and parameterized for diverse applications
- Modern Vision-Language-Action models enable natural language-commanded grasping and can generalize to novel objects and tasks through large-scale pre-training and fine-tuning
- Successful grasping systems require careful integration of multiple components, simulation-based development, and robust error recovery strategies

## Further Reading

- **Official Documentation**:
  - [MoveIt 2 Grasping Pipeline](https://moveit.picknik.ai/main/doc/examples/pick_place/pick_place_tutorial.html) - End-to-end grasp planning with collision checking
  - [GelSight Sensor Documentation](http://gelsight.csail.mit.edu/) - Optical tactile sensing technology
- **Research Papers**:
  - Miller & Allen (2004), "GraspIt!: A Versatile Simulator for Robotic Grasping" - Foundational grasp simulation platform
  - Mahler et al. (2017), "Dex-Net 2.0: Deep Learning to Plan Robust Grasps" - CNN-based grasp quality prediction
  - Levine et al. (2018), "Learning Hand-Eye Coordination for Robotic Grasping with Deep Learning" - End-to-end learning from pixels
  - Brohan et al. (2023), "RT-2: Vision-Language-Action Models" - VLA for robotic manipulation
- **Tutorials**:
  - [ROS 2 Pick and Place Tutorial](https://docs.ros.org/en/humble/Tutorials/Advanced/Pick-and-Place.html) - Implementing manipulation primitives
  - [PyBullet Grasp Simulation](https://pybullet.org/wordpress/) - Physics-based grasp testing
- **Community Resources**:
  - [RoboGrasp Community](https://discourse.ros.org/c/manipulation) - Discussion forum for manipulation
  - [Grasp Pose Detection Benchmark](https://github.com/atenpas/gpd) - Standard datasets and evaluation

## Review Questions

1. Explain the difference between force closure and form closure for robotic grasps. Why is force closure more commonly used in practice?

2. A parallel-jaw gripper with coefficient of friction μ = 0.5 grasps a box at two opposing faces. What is the minimum normal force required at each contact to support a 2 kg object against gravity? (Assume g = 9.81 m/s²)

3. Compare sampling-based grasp planning with learning-based approaches. In what scenarios would you prefer one over the other?

4. Describe how optical tactile sensors like GelSight can detect slip. What information from the sensor is used, and how is it processed?

5. Explain the pick-and-place primitive phases (approach, grasp, lift, place). Why is it important to include approach and lift phases rather than directly moving to the grasp and place poses?

6. A grasp quality metric returns ε = 0.05 for candidate grasp A and ε = 0.15 for candidate grasp B. Which grasp is more robust to external disturbances? If both grasps achieve force closure, why might you still prefer grasp B?

7. Describe how Vision-Language-Action (VLA) models enable natural language-commanded grasping. What are the key components of a VLA pipeline, and how do they work together?

## Hands-On Exercise

**Exercise Title**: Implementing a Grasp Planner for Cylinder Objects

**Objective**: Develop a grasp planning system that generates and evaluates parallel-jaw grasps for cylindrical objects using depth images.

**Prerequisites**:
- Completed previous chapters on perception, URDF modeling, and simulation
- ROS 2 Humble installed
- Python 3.10+ with NumPy, SciPy, OpenCV

**Steps**:

1. Set up a simulated environment with Gazebo containing cylindrical objects (bottles, cans) and a parallel-jaw gripper robot.

2. Implement the `GraspSampler` class from Section 1 to generate candidate grasps for cylinders. Test with various cylinder dimensions (radius 2-5cm, height 10-30cm).

3. Implement force closure checking using the `compute_grasp_matrix` and `check_force_closure` functions from Section 2. Verify that top-down grasps on cylinders achieve force closure with μ ≥ 0.3.

4. Create a ROS 2 node that:
   - Subscribes to depth images from a simulated camera
   - Segments cylindrical objects using RANSAC cylinder fitting
   - Generates grasp candidates using your sampler
   - Evaluates candidates using force closure and quality metrics
   - Publishes the best grasp pose to a visualization topic

5. Visualize the grasp candidates in RViz using markers. Color-code grasps by quality (green = high, yellow = medium, red = low).

6. Test your planner by commanding the robot to execute the top-ranked grasp. Measure success rate over 20 trials with randomly placed cylinders.

**Expected Outcome**: Your grasp planner should consistently generate force-closure grasps for cylinders with 80%+ success rate in simulation. Top-ranked grasps should approach from the side (diametric grasp) rather than the top, as these typically have higher quality metrics.

**Extension Challenges** (Optional):
- Extend the planner to handle boxes by implementing a box grasp sampler
- Integrate tactile sensing simulation to detect slip during lifting
- Implement grasp refinement using gradient-based optimization to improve initial samples

**Complete code available in**: `/static/code/vla/chapter19/`

---

**Previous Chapter**: [Humanoid Robot Modeling](/docs/module-04-vla/week-11/ch17-humanoid-urdf)
**Next Chapter**: [Voice Interfaces with Whisper](/docs/module-04-vla/week-13/ch21-whisper-voice)
**Module Overview**: [VLA Systems](/docs/module-04-vla/)
