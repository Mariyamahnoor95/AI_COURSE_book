#!/usr/bin/env python3
"""Generate all textbook chapter template files"""

from pathlib import Path

BASE_DIR = Path("/Users/noori/docusorus_project/hackathon1_book/my-website/docs")

# Define ALL chapters
CHAPTERS = [
    # Foundations Week 2
    ("foundations/week-02", "ch02-sensors.md", "Sensors in Physical AI Systems", ["Vision sensors (RGB, depth)", "LiDAR and range sensing", "IMUs and inertial sensing", "Proprioceptive sensors", "Sensor fusion strategies"]),
    ("foundations/week-02", "ch03-actuators-robotics-arch.md", "Actuators and Robotics Architectures", ["Electric motors and servos", "Hydraulic and pneumatic systems", "Control system hierarchies", "Robot platforms and architectures", "Safety and redundancy"]),

    # Module 1 Week 3-5
    ("module-01-ros2/week-03", "ch02-services-actions.md", "ROS 2 Services and Actions", ["Service servers and clients", "Synchronous request-response", "Action servers and clients", "Long-running task management", "Feedback and cancellation"]),
    ("module-01-ros2/week-04", "ch03-python-rclpy.md", "Python Programming with rclpy", ["rclpy API overview", "Creating custom nodes", "Message handling patterns", "Quality of Service", "Best practices"]),
    ("module-01-ros2/week-04", "ch04-tf2-transforms.md", "Coordinate Transforms with TF2", ["TF2 library fundamentals", "Coordinate frames", "Transform broadcasting", "Transform lookup", "Static vs dynamic transforms"]),
    ("module-01-ros2/week-05", "ch05-urdf-models.md", "Robot Modeling with URDF", ["URDF syntax and structure", "Links and joints", "Visual and collision geometry", "Gazebo integration", "Xacro for modularity"]),
    ("module-01-ros2/week-05", "ch06-nav2-basics.md", "Navigation with Nav2", ["Nav2 stack overview", "Costmaps and planning", "Behavior trees", "Recovery behaviors", "Localization (AMCL)"]),

    # Module 2 Week 6-7
    ("module-02-digital-twin/week-06", "ch07-gazebo-physics.md", "Gazebo Physics Simulation", ["Gazebo architecture", "Physics engines (ODE, Bullet)", "World files and models", "Sensor plugins", "Contact and collision"]),
    ("module-02-digital-twin/week-06", "ch08-sensor-modeling.md", "Sensor Modeling in Simulation", ["Camera simulation", "LiDAR and ray tracing", "IMU and noise models", "Ground truth data", "Sim-to-real considerations"]),
    ("module-02-digital-twin/week-07", "ch09-unity-viz.md", "Unity for Robot Visualization", ["Unity fundamentals", "ROS-Unity bridge", "3D visualization", "User interfaces", "Real-time rendering"]),
    ("module-02-digital-twin/week-07", "ch10-digital-twin.md", "Digital Twin Concepts", ["Digital twin architecture", "State synchronization", "Bi-directional communication", "Predictive analytics", "Applications in robotics"]),

    # Module 3 Week 8-10
    ("module-03-isaac/week-08", "ch11-isaac-sim.md", "NVIDIA Isaac Sim", ["Isaac Sim installation", "RTX rendering", "PhysX 5.0 physics", "Synthetic data generation", "Replicator for data"]),
    ("module-03-isaac/week-08", "ch12-isaac-ros.md", "Isaac ROS Perception", ["Isaac ROS packages", "GPU-accelerated nodes", "Perception pipelines", "AprilTag detection", "Performance optimization"]),
    ("module-03-isaac/week-09", "ch13-vslam-nav2.md", "Visual SLAM and Navigation", ["VSLAM algorithms (ORB-SLAM)", "Visual odometry", "Loop closure", "Nav2 integration", "Mapping strategies"]),
    ("module-03-isaac/week-09", "ch14-perception.md", "Perception Pipelines", ["Object detection (YOLO, RT-DETR)", "Semantic segmentation", "3D bounding boxes", "Point cloud processing", "Real-time constraints"]),
    ("module-03-isaac/week-10", "ch15-rl-sim2real.md", "Reinforcement Learning and Sim-to-Real", ["RL fundamentals (PPO, SAC)", "Domain randomization", "System identification", "Transfer learning", "Reality gap analysis"]),
    ("module-03-isaac/week-10", "ch16-sensor-fusion.md", "Multi-Sensor Fusion", ["Kalman filtering (EKF, UKF)", "Sensor fusion architectures", "Data association", "Uncertainty propagation", "Robust estimation"]),

    # Module 4 Week 11-13
    ("module-04-vla/week-11", "ch17-humanoid-urdf.md", "Humanoid Robot Modeling", ["Humanoid URDF structure", "Kinematic chains", "Joint limits and dynamics", "Center of mass", "Balance constraints"]),
    ("module-04-vla/week-11", "ch18-joint-control.md", "Joint-Level Control", ["PID control", "Trajectory generation", "Impedance control", "Force control", "Compliant manipulation"]),
    ("module-04-vla/week-12", "ch19-grasping.md", "Robotic Grasping", ["Grasp planning algorithms", "Force closure analysis", "Tactile sensing", "Manipulation primitives", "Learning-based grasping"]),
    ("module-04-vla/week-12", "ch20-walking-gaits.md", "Bipedal Walking and Gaits", ["Gait patterns", "Zero moment point (ZMP)", "Walking controllers", "Dynamic balance", "Push recovery"]),
    ("module-04-vla/week-13", "ch21-whisper-voice.md", "Voice Interfaces with Whisper", ["Whisper API", "Speech recognition", "Voice command parsing", "Natural language understanding", "Multimodal interaction"]),
    ("module-04-vla/week-13", "ch22-llm-planning.md", "LLM-Based Task Planning", ["LLM integration", "Prompt engineering", "Task decomposition", "High-level reasoning", "Grounding language to actions"]),
    ("module-04-vla/week-13", "ch23-vla-integration.md", "Vision-Language-Action Integration", ["VLA model architecture", "RT-2 and PaLM-E", "Multimodal learning", "End-to-end policies", "Future research directions"]),
]

TEMPLATE = """---
id: {chapter_id}
title: {title}
sidebar_label: {sidebar_label}
sidebar_position: {position}
---

# {title}

:::caution Content In Progress
This chapter template provides structure for technical writers and subject matter experts to fill in detailed content.
:::

## Learning Objectives

By the end of this chapter, you will be able to:

{learning_objectives}

## Introduction

TODO: Write 2-3 paragraphs introducing {title_lower} and its importance in Physical AI and robotics systems.

Key questions to address:
- What problem does this technology solve?
- Why is it important for Physical AI systems?
- How does it build on previous chapters?

{topic_sections}

## Practical Example

TODO: Provide a hands-on code example or walkthrough demonstrating the key concepts of {title_lower}.

```python
# TODO: Add code example demonstrating {title_lower}
import rclpy
from rclpy.node import Node

class Example{class_name}Node(Node):
    def __init__(self):
        super().__init__('example_{node_name}')
        # TODO: Implement example
        self.get_logger().info('{title} node initialized')

def main():
    rclpy.init()
    node = Example{class_name}Node()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Key Implementation Points:**
- TODO: Highlight important implementation detail 1
- TODO: Highlight important implementation detail 2
- TODO: Explain common pitfalls and how to avoid them

## Common Challenges and Solutions

### Challenge 1: TODO Title

**Problem**: TODO: Describe a common challenge students face

**Solution**: TODO: Provide practical solution with explanation

### Challenge 2: TODO Title

**Problem**: TODO: Describe another common challenge

**Solution**: TODO: Provide practical solution with explanation

## Best Practices

1. **TODO Practice 1**: Brief explanation
2. **TODO Practice 2**: Brief explanation
3. **TODO Practice 3**: Brief explanation
4. **TODO Practice 4**: Brief explanation

## Summary

Key takeaways from this chapter:

- TODO: Key point 1 about {title_lower}
- TODO: Key point 2 about {title_lower}
- TODO: Key point 3 about {title_lower}
- TODO: How this knowledge applies to Physical AI systems

## Further Reading

- **Official Documentation**: TODO: Link to relevant official docs
- **Research Papers**: TODO: List 2-3 foundational papers
- **Tutorials**: TODO: Link to external learning resources
- **Community Resources**: TODO: Forums, discussion groups

## Review Questions

1. TODO: Conceptual question about main topic
2. TODO: Application question requiring analysis
3. TODO: Comparison question (compare two approaches)
4. TODO: Implementation question
5. TODO: Problem-solving scenario question

## Hands-On Exercise

**Exercise Title**: TODO: Descriptive exercise name

**Objective**: TODO: What should students learn from this exercise?

**Prerequisites**:
- Completed previous chapters
- ROS 2 environment set up
- TODO: Any specific tools or packages

**Steps**:
1. TODO: Step-by-step instruction 1
2. TODO: Step-by-step instruction 2
3. TODO: Step-by-step instruction 3
4. TODO: Test and verify results

**Expected Outcome**: TODO: Describe what successful completion looks like

**Extension Challenges** (Optional):
- TODO: Advanced variation 1
- TODO: Advanced variation 2

**Complete code available in**: `/static/code/{module_code}/chapter{chapter_num}/`

---

**Previous Chapter**: TODO: Link to previous chapter
**Next Chapter**: TODO: Link to next chapter
**Module Overview**: TODO: Link to module index
"""

def sanitize_for_class_name(title):
    """Convert title to PascalCase class name"""
    words = title.replace("-", " ").replace(":", "").split()
    return "".join(w.capitalize() for w in words if w.lower() not in ['and', 'with', 'the', 'in', 'for'])

def sanitize_for_node_name(title):
    """Convert title to snake_case node name"""
    words = title.replace("-", " ").replace(":", "").lower().split()
    return "_".join(w for w in words if w not in ['and', 'with', 'the', 'in', 'for'])

def extract_module_code(path):
    """Extract module code from path"""
    if "foundations" in path:
        return "foundations"
    elif "module-01" in path:
        return "ros2"
    elif "module-02" in path:
        return "digital-twin"
    elif "module-03" in path:
        return "isaac"
    elif "module-04" in path:
        return "vla"
    return "misc"

def main():
    created = []
    skipped = []

    for idx, (path, filename, title, topics) in enumerate(CHAPTERS, start=1):
        file_path = BASE_DIR / path / filename

        # Skip if exists
        if file_path.exists():
            skipped.append(str(file_path))
            continue

        # Ensure directory exists
        file_path.parent.mkdir(parents=True, exist_ok=True)

        # Generate learning objectives
        objectives = "\n".join(f"- Understand and apply concepts related to {topic.lower()}" for topic in topics)

        # Generate topic sections
        sections = ""
        for i, topic in enumerate(topics, start=1):
            sections += f"""## {i}. {topic}

TODO: Write 3-4 paragraphs explaining {topic.lower()} in detail.

### Key Concepts

- **TODO Concept 1**: Brief explanation
- **TODO Concept 2**: Brief explanation
- **TODO Concept 3**: Brief explanation

### Implementation Details

TODO: Provide technical details about implementing {topic.lower()}.

"""

        # Extract chapter number from filename
        chapter_num = filename.split("-")[0].replace("ch", "").replace(".md", "")

        # Generate content
        content = TEMPLATE.format(
            chapter_id=f"{path}/{filename.replace('.md', '')}",
            title=title,
            sidebar_label=title if len(title) <= 25 else title[:22] + "...",
            position=idx,
            learning_objectives=objectives,
            title_lower=title.lower(),
            topic_sections=sections,
            class_name=sanitize_for_class_name(title),
            node_name=sanitize_for_node_name(title),
            module_code=extract_module_code(path),
            chapter_num=chapter_num
        )

        # Write file
        with open(file_path, 'w') as f:
            f.write(content)

        created.append(str(file_path))

    # Summary
    print(f"\n✅ Created {len(created)} new chapter template files")
    print(f"⏭️  Skipped {len(skipped)} existing files")
    print(f"\n📊 Total chapters: {len(CHAPTERS) + 2}")  # +2 for the sample chapters already created
    print("   - 2 complete sample chapters")
    print(f"   - {len(created)} template chapters for content authors")

if __name__ == "__main__":
    main()
