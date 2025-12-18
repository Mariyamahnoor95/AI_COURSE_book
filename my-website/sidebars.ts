import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

/**
 * Sidebar configuration for Physical AI & Humanoid Robotics textbook
 * Structure follows ADR-0002: Nested categories with module → week → chapter hierarchy
 */
const sidebars: SidebarsConfig = {
  textbookSidebar: [
    {
      type: 'doc',
      id: 'intro',
      label: 'Course Overview',
    },
    {
      type: 'category',
      label: 'Foundations (Weeks 1-2)',
      collapsible: true,
      collapsed: false,
      link: {
        type: 'generated-index',
        title: 'Foundations',
        description: 'Introduction to Physical AI, embodied intelligence, sensors, and actuators',
        slug: '/foundations',
      },
      items: [
        {
          type: 'category',
          label: 'Week 1',
          collapsible: true,
          items: [
            'foundations/week-01/ch00-intro-physical-ai',
            'foundations/week-01/ch01-embodied-intelligence',
          ],
        },
        {
          type: 'category',
          label: 'Week 2',
          collapsible: true,
          items: [
            'foundations/week-02/ch02-sensors',
            'foundations/week-02/ch03-actuators-robotics-arch',
          ],
        },
      ],
    },
    {
      type: 'category',
      label: 'Module 1: ROS 2 (Weeks 3-5)',
      collapsible: true,
      collapsed: true,
      link: {
        type: 'generated-index',
        title: 'Module 1: ROS 2',
        description: 'Robot Operating System 2: nodes, topics, services, Python APIs, and navigation',
        slug: '/module-01-ros2',
      },
      items: [
        {
          type: 'category',
          label: 'Week 3',
          collapsible: true,
          items: [
            'module-01-ros2/week-03/ch01-nodes-topics',
            'module-01-ros2/week-03/ch02-services-actions',
          ],
        },
        {
          type: 'category',
          label: 'Week 4',
          collapsible: true,
          items: [
            'module-01-ros2/week-04/ch03-python-rclpy',
            'module-01-ros2/week-04/ch04-tf2-transforms',
          ],
        },
        {
          type: 'category',
          label: 'Week 5',
          collapsible: true,
          items: [
            'module-01-ros2/week-05/ch05-urdf-models',
            'module-01-ros2/week-05/ch06-nav2-basics',
          ],
        },
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Digital Twin (Weeks 6-7)',
      collapsible: true,
      collapsed: true,
      link: {
        type: 'generated-index',
        title: 'Module 2: Digital Twin Simulation',
        description: 'Gazebo physics simulation, Unity visualization, and digital twin concepts',
        slug: '/module-02-digital-twin',
      },
      items: [
        {
          type: 'category',
          label: 'Week 6',
          collapsible: true,
          items: [
            'module-02-digital-twin/week-06/ch07-gazebo-physics',
            'module-02-digital-twin/week-06/ch08-sensor-modeling',
          ],
        },
        {
          type: 'category',
          label: 'Week 7',
          collapsible: true,
          items: [
            'module-02-digital-twin/week-07/ch09-unity-viz',
            'module-02-digital-twin/week-07/ch10-digital-twin',
          ],
        },
      ],
    },
    {
      type: 'category',
      label: 'Module 3: NVIDIA Isaac (Weeks 8-10)',
      collapsible: true,
      collapsed: true,
      link: {
        type: 'generated-index',
        title: 'Module 3: NVIDIA Isaac Platform',
        description: 'Isaac Sim, Isaac ROS, VSLAM, perception, and reinforcement learning',
        slug: '/module-03-isaac',
      },
      items: [
        {
          type: 'category',
          label: 'Week 8',
          collapsible: true,
          items: [
            'module-03-isaac/week-08/ch11-isaac-sim',
            'module-03-isaac/week-08/ch12-isaac-ros',
          ],
        },
        {
          type: 'category',
          label: 'Week 9',
          collapsible: true,
          items: [
            'module-03-isaac/week-09/ch13-vslam-nav2',
            'module-03-isaac/week-09/ch14-perception',
          ],
        },
        {
          type: 'category',
          label: 'Week 10',
          collapsible: true,
          items: [
            'module-03-isaac/week-10/ch15-rl-sim2real',
            'module-03-isaac/week-10/ch16-sensor-fusion',
          ],
        },
      ],
    },
    {
      type: 'category',
      label: 'Module 4: VLA Integration (Weeks 11-13)',
      collapsible: true,
      collapsed: true,
      link: {
        type: 'generated-index',
        title: 'Module 4: Vision-Language-Action Integration',
        description: 'Humanoid robotics, grasping, walking, voice interfaces, and VLA models',
        slug: '/module-04-vla',
      },
      items: [
        {
          type: 'category',
          label: 'Week 11',
          collapsible: true,
          items: [
            'module-04-vla/week-11/ch17-humanoid-urdf',
            'module-04-vla/week-11/ch18-joint-control',
          ],
        },
        {
          type: 'category',
          label: 'Week 12',
          collapsible: true,
          items: [
            'module-04-vla/week-12/ch19-grasping',
            'module-04-vla/week-12/ch20-walking-gaits',
          ],
        },
        {
          type: 'category',
          label: 'Week 13',
          collapsible: true,
          items: [
            'module-04-vla/week-13/ch21-whisper-voice',
            'module-04-vla/week-13/ch22-llm-planning',
            'module-04-vla/week-13/ch23-vla-integration',
          ],
        },
      ],
    },
  ],
};

export default sidebars;
