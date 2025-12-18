#!/usr/bin/env python3
"""
Script to generate template chapter files for the Physical AI textbook
"""

import os
from pathlib import Path

# Base directory
BASE_DIR = Path("/Users/noori/docusorus_project/hackathon1_book/my-website/docs")

# Chapter definitions
CHAPTERS = {
    "foundations/week-01": [
        {
            "file": "ch01-embodied-intelligence.md",
            "title": "Embodied Intelligence and Sensor-Motor Loops",
            "topics": ["Embodied cognition", "Sensor-motor coordination", "Feedback loops", "Real-time control"]
        }
    ],
    "foundations/week-02": [
        {
            "file": "ch02-sensors.md",
            "title": "Sensors in Physical AI Systems",
            "topics": ["Vision sensors", "LiDAR", "IMUs", "Proprioceptive sensors", "Sensor fusion"]
        },
        {
            "file": "ch03-actuators-robotics-arch.md",
            "title": "Actuators and Robotics Architectures",
            "topics": ["Motors and actuators", "Control systems", "Robotic architectures", "Hardware platforms"]
        }
    ],
    "module-01-ros2/week-03": [
        {
            "file": "ch02-services-actions.md",
            "title": "ROS 2 Services and Actions",
            "topics": ["Service servers and clients", "Action servers", "Long-running tasks", "Request-response patterns"]
        }
    ],
    "module-01-ros2/week-04": [
        {
            "file": "ch03-python-rclpy.md",
            "title": "Python Programming with rclpy",
            "topics": ["rclpy API", "Creating custom nodes", "Working with messages", "Best practices"]
        },
        {
            "file": "ch04-tf2-transforms.md",
            "title": "Coordinate Transforms with TF2",
            "topics": ["TF2 library", "Coordinate frames", "Transform broadcasting", "Transform listening"]
        }
    ],
    "module-01-ros2/week-05": [
        {
            "file": "ch05-urdf-models.md",
            "title": "Robot Modeling with URDF",
            "topics": ["URDF syntax", "Robot description", "Joints and links", "Gazebo integration"]
        },
        {
            "file": "ch06-nav2-basics.md",
            "title": "Navigation with Nav2",
            "topics": ["Nav2 stack", "Path planning", "Localization", "Obstacle avoidance"]
        }
    ],
    "module-02-digital-twin/week-06": [
        {
            "file": "ch07-gazebo-physics.md",
            "title": "Gazebo Physics Simulation",
            "topics": ["Gazebo architecture", "Physics engines", "World files", "Sensor plugins"]
        },
        {
            "file": "ch08-sensor-modeling.md",
            "title": "Sensor Modeling in Simulation",
            "topics": ["Camera models", "LiDAR simulation", "IMU modeling", "Noise and uncertainty"]
        }
    ],
    "module-02-digital-twin/week-07": [
        {
            "file": "ch09-unity-viz.md",
            "title": "Unity for Robot Visualization",
            "topics": ["Unity basics", "ROS-Unity integration", "3D visualization", "User interfaces"]
        },
        {
            "file": "ch10-digital-twin.md",
            "title": "Digital Twin Concepts and Implementation",
            "topics": ["Digital twin architecture", "Real-time synchronization", "State estimation", "Applications"]
        }
    ],
    "module-03-isaac/week-08": [
        {
            "file": "ch11-isaac-sim.md",
            "title": "NVIDIA Isaac Sim",
            "topics": ["Isaac Sim setup", "RTX rendering", "PhysX physics", "Synthetic data generation"]
        },
        {
            "file": "ch12-isaac-ros.md",
            "title": "Isaac ROS Perception",
            "topics": ["Isaac ROS packages", "GPU acceleration", "Perception pipelines", "Performance optimization"]
        }
    ],
    "module-03-isaac/week-09": [
        {
            "file": "ch13-vslam-nav2.md",
            "title": "Visual SLAM and Navigation",
            "topics": ["VSLAM algorithms", "Map building", "Localization", "Nav2 integration"]
        },
        {
            "file": "ch14-perception.md",
            "title": "Perception Pipelines",
            "topics": ["Object detection", "Segmentation", "3D perception", "Real-time processing"]
        }
    ],
    "module-03-isaac/week-10": [
        {
            "file": "ch15-rl-sim2real.md",
            "title": "Reinforcement Learning and Sim-to-Real",
            "topics": ["RL basics", "Domain randomization", "Transfer learning", "Sim-to-real gap"]
        },
        {
            "file": "ch16-sensor-fusion.md",
            "title": "Multi-Sensor Fusion",
            "topics": ["Kalman filtering", "Sensor fusion architectures", "Data association", "Uncertainty handling"]
        }
    ],
    "module-04-vla/week-11": [
        {
            "file": "ch17-humanoid-urdf.md",
            "title": "Humanoid Robot Modeling",
            "topics": ["Humanoid URDF", "Kinematic chains", "Joint controllers", "Balance and stability"]
        },
        {
            "file": "ch18-joint-control.md",
            "title": "Joint-Level Control",
            "topics": ["PID controllers", "Trajectory generation", "Impedance control", "Force control"]
        }
    ],
    "module-04-vla/week-12": [
        {
            "file": "ch19-grasping.md",
            "title": "Robotic Grasping",
            "topics": ["Grasp planning", "Force closure", "Tactile sensing", "Manipulation primitives"]
        },
        {
            "file": "ch20-walking-gaits.md",
            "title": "Bipedal Walking and Gaits",
            "topics": ["Gait patterns", "Zero moment point", "Walking controllers", "Dynamic balance"]
        }
    ],
    "module-04-vla/week-13": [
        {
            "file": "ch21-whisper-voice.md",
            "title": "Voice Interfaces with Whisper",
            "topics": ["Speech recognition", "Whisper API", "Voice commands", "Natural language understanding"]
        },
        {
            "file": "ch22-llm-planning.md",
            "title": "LLM-Based Task Planning",
            "topics": ["LLM integration", "Task decomposition", "High-level planning", "Reasoning"]
        },
        {
            "file": "ch23-vla-integration.md",
            "title": "Vision-Language-Action Integration",
            "topics": ["VLA models", "Multimodal learning", "End-to-end systems", "Future directions"]
        }
    ]
}

TEMPLATE = """---
id: {chapter_id}
title: {title}
sidebar_label: {sidebar_label}
sidebar_position: {position}
---

# {title}

:::caution Content In Progress
This chapter is currently under development. The content outline below provides the structure for technical writers and subject matter experts.
:::

## Learning Objectives

By the end of this chapter, you will be able to:

- TODO: Define specific learning objective 1
- TODO: Define specific learning objective 2
- TODO: Define specific learning objective 3
- TODO: Define specific learning objective 4

## Introduction

TODO: Write 2-3 paragraphs introducing the chapter topic and its importance in Physical AI/Robotics.

Key questions to address:
- What problem does this technology solve?
- Why is it important for Physical AI systems?
- How does it relate to previous chapters?

## Main Topics

{topic_sections}

## Practical Example

TODO: Provide a hands-on code example or walkthrough demonstrating the concepts.

```python
# TODO: Add code example
pass
```

**Key Points:**
- TODO: Highlight important implementation details
- TODO: Explain common pitfalls
- TODO: Provide best practices

## Common Challenges and Solutions

### Challenge 1: TODO Title

**Problem**: TODO: Describe the challenge

**Solution**: TODO: Explain the solution

### Challenge 2: TODO Title

**Problem**: TODO: Describe the challenge

**Solution**: TODO: Explain the solution

## Summary

TODO: Write a concise summary (3-5 bullet points) of key takeaways from this chapter.

- Key point 1
- Key point 2
- Key point 3

## Further Reading

- **Papers**: TODO: List 2-3 relevant academic papers
- **Documentation**: TODO: Link to official documentation
- **Tutorials**: TODO: Link to external tutorials or resources

## Review Questions

1. TODO: Conceptual question about main topic
2. TODO: Application question
3. TODO: Comparison question
4. TODO: Implementation question
5. TODO: Problem-solving question

## Hands-On Exercise

TODO: Design a practical exercise that reinforces the chapter concepts.

**Exercise**: TODO: Exercise title

**Objective**: TODO: What should students learn?

**Steps**:
1. TODO: Step 1
2. TODO: Step 2
3. TODO: Step 3

**Expected Outcome**: TODO: What should the result look like?

---

**Next Chapter**: TODO: Link to next chapter
**Previous Chapter**: TODO: Link to previous chapter
"""


def create_chapter_templates():
    """Generate all template chapter files"""
    created_files = []

    for path, chapters in CHAPTERS.items():
        dir_path = BASE_DIR / path
        dir_path.mkdir(parents=True, exist_ok=True)

        for idx, chapter_info in enumerate(chapters, start=1):
            file_path = dir_path / chapter_info["file"]

            # Skip if file already exists
            if file_path.exists():
                print(f"⏭️  Skipping existing file: {file_path}")
                continue

            # Generate topic sections
            topic_sections = ""
            for i, topic in enumerate(chapter_info["topics"], start=1):
                topic_sections += f"""### {i}. {topic}

TODO: Write 2-3 paragraphs explaining {topic.lower()}.

**Key Concepts:**
- TODO: Concept 1
- TODO: Concept 2
- TODO: Concept 3

"""

            # Generate chapter ID
            chapter_id = f"{path}/{chapter_info['file'].replace('.md', '')}"

            # Generate sidebar label (shorter version of title)
            title_parts = chapter_info["title"].split()
            if len(title_parts) > 4:
                sidebar_label = " ".join(title_parts[:4]) + "..."
            else:
                sidebar_label = chapter_info["title"]

            # Fill template
            content = TEMPLATE.format(
                chapter_id=chapter_id,
                title=chapter_info["title"],
                sidebar_label=sidebar_label,
                position=idx,
                topic_sections=topic_sections
            )

            # Write file
            with open(file_path, 'w') as f:
                f.write(content)

            created_files.append(str(file_path))
            print(f"✅ Created: {file_path}")

    print(f"\n🎉 Created {len(created_files)} template chapter files!")
    return created_files


if __name__ == "__main__":
    create_chapter_templates()
