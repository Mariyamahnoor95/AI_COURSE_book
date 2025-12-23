---
id: ch09-unity-viz
title: Unity for Robot Visualization
sidebar_label: Unity Visualization
sidebar_position: 1
---

# Unity for Robot Visualization

## Learning Objectives

By the end of this chapter, you will be able to:

- Understand Unity game engine fundamentals for robotics visualization
- Set up and configure the ROS-Unity bridge for real-time communication
- Create 3D visualizations of robot models and sensor data
- Build interactive user interfaces for robot control and monitoring
- Implement real-time rendering for smooth visualization

## Introduction

While Gazebo excels at physics simulation, **Unity** provides superior visualization, user interface design, and cross-platform deployment. Unity is a professional game engine used to create immersive 3D experiences, and it's increasingly adopted in robotics for:

**Why Unity for robotics?**
- **Photorealistic rendering**: High-quality graphics for presentations and demos
- **Rich UI toolkit**: Build intuitive control panels and dashboards
- **Cross-platform**: Deploy to Windows, Mac, Linux, mobile, VR/AR
- **Performance**: Optimized 3D rendering at 60+ FPS
- **Asset ecosystem**: Thousands of 3D models, materials, and effects

**Unity vs Gazebo:**

| Feature | Gazebo | Unity |
|---------|--------|-------|
| **Physics simulation** | Excellent | Good (PhysX) |
| **Visualization** | Basic | Photorealistic |
| **UI/UX** | Limited | Professional |
| **Rendering performance** | Moderate | Excellent |
| **Ease of use** | Steep learning curve | More accessible |
| **ROS integration** | Native | via ROS-Unity bridge |

**Common use cases:**
- Digital twin visualization for remote monitoring
- Training data generation for computer vision
- VR/AR teleoperation interfaces
- Customer demos and presentations
- Synthetic data generation for ML

This chapter teaches how to use Unity alongside ROS 2 for advanced robot visualization.

## 1. Unity Fundamentals

### Unity Editor Overview

Unity's editor is organized into several key panels:

**1. Hierarchy**: Tree view of all GameObjects in the scene
**2. Scene View**: 3D viewport for positioning objects
**3. Game View**: Runtime preview of what the camera sees
**4. Inspector**: Properties panel for selected GameObject
**5. Project**: File browser for assets (models, scripts, materials)
**6. Console**: Debug logs and error messages

### GameObjects and Components

Unity uses an **Entity-Component-System (ECS)** architecture:

**GameObject**: A container for components (like a ROS 2 node)
**Component**: Modular functionality attached to GameObjects

**Common components:**
- **Transform**: Position, rotation, scale (every GameObject has this)
- **MeshRenderer**: Displays 3D model
- **Collider**: Physics collision detection
- **Rigidbody**: Physics simulation (gravity, forces)
- **Script**: Custom behavior (C#)

**Example hierarchy for a robot:**
```
Robot (GameObject)
├── Transform
├── MeshRenderer (robot body)
├── Rigidbody (physics)
└── RobotController (custom script)
    ├── LeftWheel (child GameObject)
    │   ├── Transform
    │   └── MeshRenderer
    └── RightWheel (child GameObject)
        ├── Transform
        └── MeshRenderer
```

### Coordinate Systems

**Unity vs ROS coordinate systems differ:**

| Axis | Unity | ROS |
|------|-------|-----|
| **Forward** | +Z | +X |
| **Right** | +X | -Y |
| **Up** | +Y | +Z |

**Conversion required when bridging Unity ↔ ROS:**

```csharp
// Unity to ROS
Vector3 UnityToROS(Vector3 unityPos)
{
    return new Vector3(
        unityPos.z,  // Unity Z → ROS X (forward)
        -unityPos.x, // Unity X → ROS -Y (right)
        unityPos.y   // Unity Y → ROS Z (up)
    );
}

// ROS to Unity
Vector3 ROSToUnity(Vector3 rosPos)
{
    return new Vector3(
        -rosPos.y,   // ROS -Y → Unity X
        rosPos.z,    // ROS Z → Unity Y
        rosPos.x     // ROS X → Unity Z
    );
}
```

### Materials and Rendering

**Materials** define how surfaces appear:

**Standard shader properties:**
- **Albedo**: Base color/texture
- **Metallic**: How metallic the surface is (0-1)
- **Smoothness**: How smooth/rough (0-1)
- **Normal Map**: Surface detail without adding geometry
- **Emission**: Self-illuminating surfaces

**Example: Creating a robot material**
```csharp
// C# script to create material programmatically
using UnityEngine;

public class RobotMaterial : MonoBehaviour
{
    void Start()
    {
        // Get renderer component
        Renderer renderer = GetComponent<Renderer>();

        // Create new material
        Material robotMat = new Material(Shader.Find("Standard"));

        // Set properties
        robotMat.color = Color.gray;
        robotMat.SetFloat("_Metallic", 0.5f);
        robotMat.SetFloat("_Glossiness", 0.7f);

        // Apply material
        renderer.material = robotMat;
    }
}
```

## 2. ROS-Unity Bridge

### Installation and Setup

The **ROS-Unity Bridge** enables bidirectional communication between ROS 2 and Unity.

**Architecture:**
```
ROS 2 (Python/C++)  ←→  ROS-Unity Bridge  ←→  Unity (C#)
     (Robot)              (TCP/WebSocket)        (Visualization)
```

**Installation steps:**

1. **Unity side**: Install ROS-TCP-Connector package
   ```
   Window → Package Manager → Add package from git URL
   https://github.com/Unity-Technologies/ROS-TCP-Connector.git
   ```

2. **ROS side**: Install ROS-TCP-Endpoint
   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/Unity-Technologies/ROS-TCP-Endpoint
   cd ~/ros2_ws
   colcon build --packages-select ros_tcp_endpoint
   ```

3. **Configure Unity**:
   - Create GameObject with `ROSConnection` component
   - Set ROS IP address (e.g., `192.168.1.100`)
   - Set port (default: `10000`)

4. **Launch ROS endpoint**:
   ```bash
   ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0
   ```

### Publishing from Unity to ROS

**Example: Send Unity sensor data to ROS**

```csharp
using RosMessageTypes.Sensor;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

public class CameraPublisher : MonoBehaviour
{
    private ROSConnection ros;
    private string topicName = "unity/camera";

    void Start()
    {
        // Get ROS connection
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<ImageMsg>(topicName);
    }

    void Update()
    {
        // Publish at 10 Hz
        if (Time.frameCount % 6 == 0)
        {
            ImageMsg msg = new ImageMsg
            {
                header = new HeaderMsg
                {
                    stamp = new TimeMsg
                    {
                        sec = (int)Time.time,
                        nanosec = (uint)((Time.time % 1) * 1e9)
                    },
                    frame_id = "camera_frame"
                },
                height = 480,
                width = 640,
                encoding = "rgb8"
            };

            ros.Publish(topicName, msg);
        }
    }
}
```

### Subscribing in Unity to ROS

**Example: Receive ROS Twist messages to control Unity robot**

```csharp
using RosMessageTypes.Geometry;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

public class TwistSubscriber : MonoBehaviour
{
    private ROSConnection ros;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<TwistMsg>("cmd_vel", ReceiveTwist);
    }

    void ReceiveTwist(TwistMsg twist)
    {
        // Convert ROS Twist to Unity movement
        float linearX = (float)twist.linear.x;
        float angularZ = (float)twist.angular.z;

        // Apply to robot (simplified)
        transform.Translate(0, 0, linearX * Time.deltaTime);
        transform.Rotate(0, angularZ * Mathf.Rad2Deg * Time.deltaTime, 0);
    }
}
```

### Service Calls from Unity

```csharp
using RosMessageTypes.Std;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class ServiceCaller : MonoBehaviour
{
    private ROSConnection ros;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
    }

    public void CallResetService()
    {
        // Create request
        EmptyRequest request = new EmptyRequest();

        // Send service call
        ros.SendServiceMessage<EmptyResponse>(
            "reset_simulation",
            request,
            OnServiceResponse
        );
    }

    void OnServiceResponse(EmptyResponse response)
    {
        Debug.Log("Service call completed");
    }
}
```

## 3. 3D Visualization

### Importing Robot Models

**URDF to Unity conversion:**

1. **Use URDF Importer package**:
   ```
   Window → Package Manager → URDF Importer
   ```

2. **Import URDF**:
   ```
   Assets → Import Robot from URDF
   ```

3. **Unity creates**:
   - GameObjects for each link
   - ArticulationBody for joints (physics)
   - MeshRenderers for visual models

**Alternative: Import from FBX/OBJ**
- Export from Blender/CAD tools
- Drag into Unity Assets folder
- Maintains materials and textures

### Visualizing Sensor Data

**LiDAR Point Cloud Visualization:**

```csharp
using RosMessageTypes.Sensor;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

public class LidarVisualizer : MonoBehaviour
{
    private ParticleSystem particles;
    private ParticleSystem.Particle[] points;

    void Start()
    {
        // Create particle system for point cloud
        particles = gameObject.AddComponent<ParticleSystem>();
        var main = particles.main;
        main.loop = false;
        main.playOnAwake = false;
        main.maxParticles = 10000;

        // Subscribe to LaserScan
        ROSConnection.GetOrCreateInstance()
            .Subscribe<LaserScanMsg>("scan", VisualizeScan);
    }

    void VisualizeScan(LaserScanMsg scan)
    {
        int numPoints = scan.ranges.Length;
        points = new ParticleSystem.Particle[numPoints];

        for (int i = 0; i < numPoints; i++)
        {
            float range = scan.ranges[i];
            float angle = scan.angle_min + i * scan.angle_increment;

            // Convert polar to Cartesian (Unity coordinates)
            Vector3 position = new Vector3(
                range * Mathf.Cos(angle),
                0,
                range * Mathf.Sin(angle)
            );

            points[i].position = position;
            points[i].startSize = 0.05f;
            points[i].startColor = Color.red;
        }

        particles.SetParticles(points, numPoints);
    }
}
```

**Camera Feed Visualization:**

```csharp
using RosMessageTypes.Sensor;
using UnityEngine;
using UnityEngine.UI;

public class CameraFeedVisualizer : MonoBehaviour
{
    public RawImage displayImage;
    private Texture2D texture;

    void Start()
    {
        texture = new Texture2D(640, 480, TextureFormat.RGB24, false);
        displayImage.texture = texture;

        ROSConnection.GetOrCreateInstance()
            .Subscribe<ImageMsg>("camera/image", UpdateImage);
    }

    void UpdateImage(ImageMsg msg)
    {
        // Load image data into texture
        texture.LoadRawTextureData(msg.data);
        texture.Apply();
    }
}
```

### Trajectory Visualization

```csharp
using UnityEngine;

public class PathVisualizer : MonoBehaviour
{
    private LineRenderer lineRenderer;
    private List<Vector3> pathPoints = new List<Vector3>();

    void Start()
    {
        lineRenderer = gameObject.AddComponent<LineRenderer>();
        lineRenderer.startWidth = 0.1f;
        lineRenderer.endWidth = 0.1f;
        lineRenderer.material = new Material(Shader.Find("Sprites/Default"));
        lineRenderer.startColor = Color.green;
        lineRenderer.endColor = Color.yellow;
    }

    public void AddPathPoint(Vector3 point)
    {
        pathPoints.Add(point);
        lineRenderer.positionCount = pathPoints.Count;
        lineRenderer.SetPositions(pathPoints.ToArray());
    }

    public void ClearPath()
    {
        pathPoints.Clear();
        lineRenderer.positionCount = 0;
    }
}
```

## 4. User Interfaces

### Unity UI (uGUI) Basics

Unity provides a powerful **Canvas-based UI system**:

**UI Hierarchy:**
```
Canvas (screen space)
├── Panel (background)
│   ├── Text (labels)
│   ├── Button (controls)
│   ├── Slider (adjustable values)
│   └── InputField (text entry)
```

### Robot Control Panel Example

```csharp
using UnityEngine;
using UnityEngine.UI;

public class RobotControlPanel : MonoBehaviour
{
    public Slider speedSlider;
    public Text speedText;
    public Button startButton;
    public Button stopButton;
    public Text statusText;

    private bool robotRunning = false;

    void Start()
    {
        // Configure slider
        speedSlider.minValue = 0f;
        speedSlider.maxValue = 2.0f;
        speedSlider.value = 1.0f;
        speedSlider.onValueChanged.AddListener(OnSpeedChanged);

        // Configure buttons
        startButton.onClick.AddListener(StartRobot);
        stopButton.onClick.AddListener(StopRobot);

        UpdateStatus("Ready");
    }

    void OnSpeedChanged(float value)
    {
        speedText.text = $"Speed: {value:F2} m/s";
        // Send to ROS via bridge
        SendSpeedCommand(value);
    }

    void StartRobot()
    {
        robotRunning = true;
        UpdateStatus("Running");
        // Publish ROS start command
    }

    void StopRobot()
    {
        robotRunning = false;
        UpdateStatus("Stopped");
        // Publish ROS stop command
    }

    void UpdateStatus(string message)
    {
        statusText.text = $"Status: {message}";
        statusText.color = robotRunning ? Color.green : Color.red;
    }

    void SendSpeedCommand(float speed)
    {
        // Publish Twist message via ROS-Unity bridge
        var twist = new TwistMsg();
        twist.linear.x = speed;
        ROSConnection.GetOrCreateInstance().Publish("cmd_vel", twist);
    }
}
```

### Telemetry Dashboard

```csharp
using UnityEngine;
using UnityEngine.UI;

public class TelemetryDashboard : MonoBehaviour
{
    public Text batteryText;
    public Text velocityText;
    public Text positionText;
    public Image batteryBar;

    void Start()
    {
        // Subscribe to robot telemetry
        ROSConnection.GetOrCreateInstance()
            .Subscribe<BatteryStateMsg>("battery_state", UpdateBattery);
        ROSConnection.GetOrCreateInstance()
            .Subscribe<TwistMsg>("odom", UpdateVelocity);
    }

    void UpdateBattery(BatteryStateMsg msg)
    {
        float percentage = msg.percentage;
        batteryText.text = $"Battery: {percentage:F1}%";

        // Update battery bar color
        batteryBar.fillAmount = percentage / 100f;
        batteryBar.color = percentage > 20 ? Color.green : Color.red;
    }

    void UpdateVelocity(TwistMsg msg)
    {
        float speed = Mathf.Sqrt(
            (float)(msg.linear.x * msg.linear.x +
                    msg.linear.y * msg.linear.y)
        );
        velocityText.text = $"Speed: {speed:F2} m/s";
    }
}
```

## 5. Real-Time Rendering

### Frame Rate Optimization

**Target: 60 FPS minimum for smooth visualization**

**Optimization techniques:**

1. **Level of Detail (LOD)**:
   ```csharp
   // Simplify geometry at distance
   LODGroup lodGroup = gameObject.AddComponent<LODGroup>();
   ```

2. **Occlusion Culling**: Don't render hidden objects
   ```
   Window → Rendering → Occlusion Culling
   ```

3. **Reduce draw calls**: Batch similar materials
4. **Use GPU instancing**: For repeated objects
5. **Limit shadow casters**: Expensive feature

### Camera Setup for Robotics

```csharp
using UnityEngine;

public class RobotCamera : MonoBehaviour
{
    public Transform robotTransform;
    public float followDistance = 5f;
    public float followHeight = 3f;
    public float rotationSpeed = 5f;

    void LateUpdate()
    {
        // Follow robot smoothly
        Vector3 targetPosition = robotTransform.position
            - robotTransform.forward * followDistance
            + Vector3.up * followHeight;

        transform.position = Vector3.Lerp(
            transform.position,
            targetPosition,
            Time.deltaTime * rotationSpeed
        );

        // Look at robot
        transform.LookAt(robotTransform.position + Vector3.up);
    }
}
```

### Lighting for Clarity

**Best practices for robot visualization:**

**1. Directional Light** (sun):
- Illuminates entire scene
- Shadows for depth perception

**2. Fill Lights**:
- Reduce harsh shadows
- Use lower intensity

**3. Skybox**:
- Ambient lighting
- Reflections on metallic surfaces

**Example lighting setup:**
```csharp
public class LightingSetup : MonoBehaviour
{
    void Start()
    {
        // Main directional light
        GameObject sun = new GameObject("Directional Light");
        Light sunLight = sun.AddComponent<Light>();
        sunLight.type = LightType.Directional;
        sunLight.intensity = 1.0f;
        sunLight.shadows = LightShadows.Soft;
        sun.transform.rotation = Quaternion.Euler(50, -30, 0);

        // Ambient lighting
        RenderSettings.ambientMode = UnityEngine.Rendering.AmbientMode.Skybox;
        RenderSettings.ambientIntensity = 0.5f;
    }
}
```

## Summary

Key takeaways:

- **Unity** provides superior visualization and UI capabilities for robotics
- **ROS-Unity Bridge** enables real-time communication between ROS 2 and Unity
- **Coordinate system conversion** is required (Unity Y-up vs ROS Z-up)
- **GameObjects and Components** form Unity's entity-component architecture
- **3D visualization** supports robot models, sensor data, and trajectories
- **UI toolkit** enables professional control panels and dashboards
- **Real-time rendering** requires optimization for 60+ FPS

**Unity vs Gazebo use cases:**
- **Use Gazebo**: Physics simulation, testing control algorithms
- **Use Unity**: Visualization, demos, UI development, VR/AR, synthetic data
- **Use both**: Gazebo for sim, Unity for visualization (via bridge)

## Review Questions

1. What are the main advantages of Unity over Gazebo for robot visualization?
2. How do you convert coordinates from Unity to ROS coordinate systems?
3. What is the purpose of the ROS-Unity Bridge?
4. How would you visualize a LiDAR point cloud in Unity?
5. What techniques can improve rendering performance in Unity?
6. When would you choose Unity over Gazebo for a robotics project?
7. How do you subscribe to ROS topics from Unity C# scripts?

## Hands-on Exercises

### Exercise 1: Basic Unity Scene
- Create Unity project
- Import robot URDF
- Set up ROS-Unity bridge
- Subscribe to `/cmd_vel` and move robot in Unity

### Exercise 2: Sensor Visualization
- Add camera to robot
- Publish camera feed from Unity to ROS
- Subscribe to LiDAR data and visualize as point cloud
- Display both visualizations in UI

### Exercise 3: Control Dashboard
- Create UI canvas with control panel
- Add speed slider that publishes Twist messages
- Display robot telemetry (battery, position, velocity)
- Implement start/stop buttons

## Further Reading

- [Unity-Robotics-Hub Documentation](https://github.com/Unity-Technologies/Unity-Robotics-Hub)
- [ROS-Unity Bridge Tutorial](https://github.com/Unity-Technologies/ROS-TCP-Connector)
- [Unity Manual](https://docs.unity3d.com/Manual/index.html)
- [Unity Scripting Reference](https://docs.unity3d.com/ScriptReference/)

---

**Next Chapter**: [Digital Twin Concepts →](ch10-digital-twin.md)

**Previous Chapter**: [← Sensor Modeling in Simulation](../week-06/ch08-sensor-modeling.md)
