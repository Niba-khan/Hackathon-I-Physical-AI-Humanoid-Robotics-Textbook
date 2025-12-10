# Chapter 3: Unity High-Fidelity Rendering & Interaction - Realistic Rendering, Human-Robot Interaction Scenes, Gazebo-Unity Bridging

## Objectives
- Understand Unity's capabilities for high-fidelity robotics visualization
- Create realistic rendering scenarios for humanoid robots
- Implement human-robot interaction scenes in Unity
- Explore Gazebo-Unity bridging strategies for complete digital twin functionality

## Introduction to Unity for Robotics

Unity is a powerful real-time 3D development platform that provides high-fidelity rendering capabilities essential for digital twin applications in robotics. For humanoid robotics specifically, Unity offers:

- **Photorealistic Rendering**: Advanced lighting, shadows, and materials
- **Real-time Performance**: High frame rates for interactive applications
- **Extensive Asset Library**: Models, environments, and effects
- **Scripting Capabilities**: C# scripting for custom behaviors
- **XR Support**: Virtual and augmented reality capabilities

While Gazebo excels at physics simulation, Unity excels at visual rendering, making the combination ideal for complete digital twin solutions.

## Unity Setup for Robotics Applications

### 1. Installing Unity Hub and Editor
- Download Unity Hub from https://unity.com/
- Install the latest LTS (Long Term Support) version
- Install additional modules for robotics:
  - Universal Render Pipeline (URP) or High Definition Render Pipeline (HDRP)
  - Physics libraries
  - Input System package

### 2. Essential Unity Packages for Robotics
Add these packages through the Package Manager:
- **Universal Render Pipeline (URP)**: For optimized rendering
- **Physics**: For collision detection and basic physics
- **Input System**: For handling user interactions
- **ProBuilder**: For quick environment prototyping
- **ProGrids**: For precise object placement

## Creating Realistic Rendering Scenarios

### 1. Setting Up a Robot Scene

#### Light Setup for Realistic Rendering
```csharp
// In a script, or via the Unity Editor
// Create a Directional Light to simulate sunlight
// Position it to match Gazebo's sun direction: -0.5, -0.1, -0.9
// Set Intensity to 1.0 and Color to a warm white
```

#### Material System for Robot Parts
Unity's material system allows for realistic robot rendering:

```csharp
// Example: Creating a metallic material for robot parts
Shader standardShader = Shader.Find("Universal Render Pipeline/Lit");
Material robotMetalMat = new Material(standardShader);
robotMetalMat.SetColor("_BaseColor", Color.grey);
robotMetalMat.SetFloat("_Metallic", 0.8f); // Highly metallic
robotMetalMat.SetFloat("_Smoothness", 0.6f); // Slightly smooth but not mirror-like
```

### 2. Environment Creation

Create realistic environments for your humanoid robot:

#### Using ProBuilder for Environment Creation
1. Create a new GameObject
2. Add ProBuilder Shape component (Cube for ground, walls, etc.)
3. Edit the shape using ProBuilder tools
4. Apply appropriate materials

#### Importing Real-World Environments
1. Use Google Earth data or satellite imagery
2. Import 3D models from SketchUp or other CAD software
3. Use Unity's Terrain tools for large outdoor areas
4. Apply realistic textures and lighting

### 3. Rendering Pipeline Considerations

For robotics applications, choose the appropriate rendering pipeline:

#### Universal Render Pipeline (URP) - Recommended for Robotics
- Better performance for real-time applications
- Suitable for rendering multiple robots simultaneously
- Good balance between quality and performance
- Lower system requirements

#### High Definition Render Pipeline (HDRP) - For Maximum Quality
- Photorealistic rendering quality
- Advanced lighting and shadow techniques
- Higher system requirements
- Better for marketing/final visualization

Example URP setup in C#:
```csharp
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Rendering.Universal;

public class RobotRendererSetup : MonoBehaviour
{
    void Start()
    {
        // Configure URP settings for robot rendering
        var universalRenderPipelineAsset = (UniversalRenderPipelineAsset)GraphicsSettings.renderPipelineAsset;
        
        if (universalRenderPipelineAsset != null)
        {
            // Set up rendering for robot models
            ConfigureForRobotRendering(universalRenderPipelineAsset);
        }
    }
    
    void ConfigureForRobotRendering(UniversalRenderPipelineAsset asset)
    {
        // Adjust rendering settings for optimal robot visualization
        // This is pseudocode; actual URP configuration requires different approach
        Debug.Log("URP configured for robot rendering");
    }
}
```

## Human-Robot Interaction Scenes

### 1. Interaction Design Principles

When designing human-robot interaction scenes in Unity, consider:

#### Safety Zones
- Visualize robot's workspace and safety boundaries
- Use transparent overlays to indicate danger zones
- Highlight robot's reach and operational area

#### Intuitive Controls
- Design interfaces that non-technical users can understand
- Provide clear feedback when interacting with the robot
- Use visual indicators for robot's current state

#### Natural Motion Representation
- Show robot's intention through subtle animations
- Visualize planned paths before execution
- Highlight robot's attention/focus points

### 2. Implementing Interaction Systems

#### Teleoperation Interface
Create a UI system for remote robot operation:

```csharp
using UnityEngine;
using UnityEngine.UI;

public class TeleoperationInterface : MonoBehaviour
{
    [Header("Robot Control")]
    public Transform robotBase;
    public Transform robotHead;
    
    [Header("UI Elements")]
    public Slider linearVelocitySlider;
    public Slider angularVelocitySlider;
    public Button startButton;
    public Button stopButton;
    
    [Header("Visualization")]
    public LineRenderer pathRenderer;
    
    // Update robot visualization based on inputs
    void Update()
    {
        if (robotBase != null)
        {
            // Update robot position/rotation visualization
            float linearVel = linearVelocitySlider.value;
            float angularVel = angularVelocitySlider.value;
            
            // Visualize robot's intent
            VisualizePath(linearVel, angularVel);
        }
    }
    
    void VisualizePath(float linearVel, float angularVel)
    {
        // Draw the planned path based on current velocity commands
        // This is where you'd implement path visualization
        if (pathRenderer != null)
        {
            // Calculate and display the robot's projected path
            Vector3[] pathPoints = CalculatePath(linearVel, angularVel);
            pathRenderer.positionCount = pathPoints.Length;
            pathRenderer.SetPositions(pathPoints);
        }
    }
    
    Vector3[] CalculatePath(float linearVel, float angularVel)
    {
        // Simplified path calculation
        Vector3[] points = new Vector3[10];
        Vector3 startPos = robotBase.position;
        Vector3 direction = robotBase.forward;
        
        for (int i = 0; i < points.Length; i++)
        {
            points[i] = startPos + direction * linearVel * i * 0.1f;
            direction = Quaternion.Euler(0, angularVel * i * 0.1f, 0) * robotBase.forward;
        }
        
        return points;
    }
    
    public void OnStartRobot()
    {
        // Send start command to robot
        Debug.Log("Start robot command sent");
    }
    
    public void OnStopRobot()
    {
        // Send stop command to robot
        Debug.Log("Stop robot command sent");
    }
}
```

#### Safety Visualization
Highlight important safety elements:

```csharp
using UnityEngine;

public class SafetyZoneVisualizer : MonoBehaviour
{
    [Header("Safety Parameters")]
    public float safeDistance = 2.0f; // Safe distance from robot
    public float dangerDistance = 0.5f; // Dangerous distance
    public LayerMask obstacleLayer;
    
    [Header("Visualization")]
    public GameObject safeZoneIndicator;
    public GameObject dangerZoneIndicator;
    
    void Update()
    {
        VisualizeSafetyZones();
    }
    
    void VisualizeSafetyZones()
    {
        // Draw spheres to represent safe and danger distances
        if (safeZoneIndicator != null)
        {
            safeZoneIndicator.transform.localScale = Vector3.one * safeDistance * 2;
            safeZoneIndicator.SetActive(true);
        }
        
        if (dangerZoneIndicator != null)
        {
            dangerZoneIndicator.transform.localScale = Vector3.one * dangerDistance * 2;
            dangerZoneIndicator.SetActive(true);
        }
    }
    
    // Check for obstacles in safety zones
    bool IsPathClear(Vector3 start, Vector3 end, float radius)
    {
        // Raycast or sphere cast to check for obstacles
        return !Physics.SphereCast(start, radius, (end - start).normalized, 
                                  Vector3.Distance(start, end), obstacleLayer);
    }
}
```

## Gazebo-Unity Bridging Strategies

A complete digital twin requires synchronization between Gazebo (physics simulation) and Unity (visual rendering). Here are the main approaches:

### 1. Direct Bridge Approach

Use the Unity Robotics Hub and Gazebo integration tools:

#### Unity Robotics Hub Setup
1. Install Unity Robotics Hub from Unity Asset Store
2. Add ROS-TCP-Connector package
3. Configure network settings

Example synchronization script:
```csharp
using UnityEngine;
using RosMessageTypes.Sensor;
using RosMessageTypes.Geometry;
using Unity.Robotics.ROSTCPConnector;

public class GazeboUnityBridge : MonoBehaviour
{
    [Header("ROS Connection")]
    public string rosIPAddress = "127.0.0.1";
    public int rosPort = 10000;
    
    [Header("Robot Components")]
    public Transform robotBase;
    public Transform[] robotJoints;
    public Transform[] robotLinks;
    
    private ROSConnection ros;
    private string jointTopic = "/joint_states";
    private string tfTopic = "/tf";
    
    void Start()
    {
        // Get reference to ROS connection
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<JointStateMsg>(jointTopic);
        
        // Subscribe to joint state updates
        ros.Subscribe<JointStateMsg>("joint_states", UpdateRobotFromGazebo);
    }
    
    void UpdateRobotFromGazebo(JointStateMsg jointMsg)
    {
        // Synchronize robot position and joint angles from Gazebo
        if (robotJoints.Length == jointMsg.position.Length)
        {
            for (int i = 0; i < robotJoints.Length; i++)
            {
                // Update joint rotation based on Gazebo simulation
                robotJoints[i].localRotation = Quaternion.Euler(0, 0, (float)(jointMsg.position[i] * Mathf.Rad2Deg));
            }
        }
    }
    
    void SendCommandsToGazebo()
    {
        // Send joint commands to Gazebo
        JointStateMsg cmd = new JointStateMsg();
        cmd.name = new string[] { "joint1", "joint2", "joint3" };
        cmd.position = new double[] { 0.0, 0.0, 0.0 }; // Example positions
        cmd.velocity = new double[] { 0.0, 0.0, 0.0 };
        cmd.effort = new double[] { 0.0, 0.0, 0.0 };
        
        ros.Publish(jointTopic, cmd);
    }
    
    void Update()
    {
        // Continuously send sensor data to Gazebo
        // Update visuals based on Gazebo state
    }
}
```

### 2. Custom Middleware Bridge

Develop a custom bridge using technologies like ZeroMQ:

```csharp
// Simplified example - would require ZeroMQ libraries and custom server
using UnityEngine;
using System.Collections;

public class CustomBridge : MonoBehaviour
{
    [Header("Bridge Settings")]
    public string bridgeAddress = "tcp://localhost:5555";
    
    // This is a conceptual example
    // Actual implementation would require specific bridge technology
    
    IEnumerator ConnectToBridge()
    {
        // Connect to the bridge service
        // This would handle communication with a custom bridge server
        yield return null;
    }
    
    void SynchronizeWithGazebo()
    {
        // Send/receive data to keep Unity and Gazebo in sync
        // This could be done via JSON, Protocol Buffers, or custom formats
    }
}
```

### 3. File-Based Synchronization

Use file exchanges for less real-time applications:

```csharp
using UnityEngine;
using System.IO;
using System.Collections.Generic;

public class FileBasedSync : MonoBehaviour
{
    [Header("File Sync Settings")]
    public string dataPath = "Assets/StreamingAssets/";
    public float syncInterval = 0.1f;
    
    private float lastSyncTime;
    
    void Update()
    {
        if (Time.time - lastSyncTime > syncInterval)
        {
            SyncWithGazebo();
            lastSyncTime = Time.time;
        }
    }
    
    void SyncWithGazebo()
    {
        // Read robot state from shared file
        string robotStateFile = Path.Combine(dataPath, "robot_state.json");
        if (File.Exists(robotStateFile))
        {
            string json = File.ReadAllText(robotStateFile);
            RobotState state = JsonUtility.FromJson<RobotState>(json);
            
            // Update Unity visualization based on robot state
            UpdateRobotVisualization(state);
        }
    }
    
    void UpdateRobotVisualization(RobotState state)
    {
        if (robotBase != null)
        {
            robotBase.position = new Vector3((float)state.position.x, (float)state.position.y, (float)state.position.z);
            robotBase.rotation = Quaternion.Euler((float)state.orientation.x, (float)state.orientation.y, (float)state.orientation.z);
        }
    }
}

[System.Serializable]
public class RobotState
{
    public Vector3D position;
    public Vector3D orientation;
    public double[] jointPositions;
    public double[] jointVelocities;
}

[System.Serializable]
public class Vector3D
{
    public double x, y, z;
}
```

## Creating Interactive Scenes

### 1. Scene Hierarchy and Organization

Create a well-organized scene structure:

```
RobotDigitalTwin
├── Environment
│   ├── Ground
│   ├── Walls
│   ├── Obstacles
│   └── Lighting
├── Robot
│   ├── RobotBase
│   │   ├── Head
│   │   ├── Torso
│   │   ├── LeftArm
│   │   ├── RightArm
│   │   ├── LeftLeg
│   │   └── RightLeg
│   └── Sensors
│       ├── IMU
│       ├── Cameras
│       └── LiDAR
└── UI
    ├── ControlPanel
    ├── SafetyZones
    └── PathVisualization
```

### 2. State Visualization

Show the robot's internal state and intentions:

```csharp
using UnityEngine;
using UnityEngine.UI;

public class RobotStateVisualizer : MonoBehaviour
{
    [Header("State Display")]
    public Text stateText;           // Current state (IDLE, NAVIGATING, etc.)
    public Image batteryFill;        // Battery level visualization
    public Image connectionIndicator; // Connection status
    public GameObject[] pathPoints;   // Planned path visualization
    
    [Header("Target Elements")]
    public GameObject targetIndicator; // Current goal location
    public Color idleColor = Color.gray;
    public Color activeColor = Color.green;
    public Color errorColor = Color.red;
    
    void Update()
    {
        UpdateStateDisplay();
        UpdateTargetVisualization();
    }
    
    void UpdateStateDisplay()
    {
        // Update based on robot state
        // This would typically come from ROS or simulation state
        string robotState = GetRobotState(); // Pseudo-implementation
        stateText.text = $"State: {robotState}";
        
        // Update visual indicators
        UpdateBatteryLevel();
        UpdateConnectionStatus();
    }
    
    string GetRobotState()
    {
        // In a real implementation, this would get the state from ROS topics
        // For simulation purposes
        return "NAVIGATING";
    }
    
    void UpdateBatteryLevel()
    {
        // Update battery visualization (0-100%)
        float batteryLevel = 85.0f; // Example value
        batteryFill.fillAmount = batteryLevel / 100.0f;
        
        // Change color based on battery level
        if (batteryLevel < 20)
            batteryFill.color = Color.red;
        else if (batteryLevel < 50)
            batteryFill.color = Color.yellow;
        else
            batteryFill.color = Color.green;
    }
    
    void UpdateConnectionStatus()
    {
        // Update connection indicator
        // This would check for ROS connection or simulation status
        bool isConnected = true; // Example value
        connectionIndicator.color = isConnected ? Color.green : Color.red;
    }
    
    void UpdateTargetVisualization()
    {
        // Show current target/destination
        if (targetIndicator != null)
        {
            // Position the target indicator at the robot's current goal
            // In a real implementation, this would come from navigation topics
            Vector3 targetPos = new Vector3(5, 0, 5); // Example position
            targetIndicator.transform.position = targetPos;
            targetIndicator.SetActive(true);
        }
    }
}
```

## Performance Optimization for Real-time Rendering

### 1. Level of Detail (LOD)
Implement LOD for robot models based on distance:

```csharp
using UnityEngine;

[RequireComponent(typeof(MeshRenderer))]
public class RobotLOD : MonoBehaviour
{
    [Header("LOD Settings")]
    [Range(0.0f, 1.0f)]
    public float lodDistance = 0.5f; // Fraction of screen space at which to switch LOD
    
    [Header("LOD Models")]
    public GameObject highDetailModel;
    public GameObject lowDetailModel;
    
    private Camera mainCamera;
    private float screenPercentage;
    
    void Start()
    {
        mainCamera = Camera.main;
        screenPercentage = lodDistance;
    }
    
    void Update()
    {
        if (mainCamera != null)
        {
            UpdateLOD();
        }
    }
    
    void UpdateLOD()
    {
        // Calculate the robot's size in screen space
        float distance = Vector3.Distance(transform.position, mainCamera.transform.position);
        float objectScreenSize = CalculateScreenSize(distance);
        
        if (objectScreenSize < screenPercentage)
        {
            // Use low-detail model
            if (highDetailModel != null) highDetailModel.SetActive(false);
            if (lowDetailModel != null) lowDetailModel.SetActive(true);
        }
        else
        {
            // Use high-detail model
            if (lowDetailModel != null) lowDetailModel.SetActive(false);
            if (highDetailModel != null) highDetailModel.SetActive(true);
        }
    }
    
    float CalculateScreenSize(float distance)
    {
        // Calculate the size of the object in screen space
        // This is a simplified calculation
        float objectHeight = 1.0f; // Estimated robot height
        float objectSize = 2.0f * Mathf.Atan(objectHeight / (2.0f * distance));
        return objectSize / (2.0f * Mathf.Tan(mainCamera.fieldOfView * Mathf.Deg2Rad / 2.0f));
    }
}
```

### 2. Occlusion Culling
Use Unity's occlusion culling for large environments:

1. Mark static objects as "Occluder Static" or "Occludee Static"
2. Build occlusion culling data via Window > Rendering > Occlusion Culling
3. Use this feature especially for indoor environments with walls/doors

## Hands-on Exercise 3.1: Create a Unity Human-Robot Interaction Scene

1. Create a new Unity scene with:
   - A simple humanoid robot model (or use the one from previous exercises)
   - An indoor environment with furniture and obstacles
   - Proper lighting setup

2. Implement the following interaction elements:
   - A UI panel with teleoperation controls (move forward, turn, stop)
   - Visual indicators for robot's current state
   - Path visualization showing where the robot plans to move
   - Safety zone indicators around the robot

3. Create a basic synchronization system with these elements:
   - Robot position updates (simulated, not connected to Gazebo yet)
   - Joint angle visualization
   - Basic sensor visualization (e.g., LiDAR points if creating them)

4. Test the scene for performance and usability

## Hands-on Exercise 3.2: Design a Gazebo-Unity Bridge Architecture

Create a document outlining your approach to bridging Gazebo and Unity:

1. Identify what data needs to flow between systems
2. Choose a bridge implementation approach (direct, custom middleware, file-based)
3. Design the data structures that will be synchronized
4. Consider real-time performance requirements for your application
5. Plan for handling delays and potential data desynchronization

## Validation Checklist
- [ ] I understand Unity's capabilities for high-fidelity robotics visualization
- [ ] I can create realistic rendering scenarios for humanoid robots
- [ ] I have implemented human-robot interaction scenes in Unity
- [ ] I understand different Gazebo-Unity bridging strategies
- [ ] I have created an interactive Unity scene with proper performance optimization
- [ ] I have designed a bridge architecture between Gazebo and Unity

## Summary

This chapter covered the advanced aspects of digital twin technology using Unity for high-fidelity visualization and human-robot interaction. We explored Unity setup for robotics, realistic rendering techniques, human-robot interaction design principles, and various approaches to bridging Gazebo and Unity for complete digital twin functionality.

The combination of Gazebo's physics simulation and Unity's high-fidelity rendering creates a powerful platform for digital twin applications in humanoid robotics. When properly implemented, these systems enable safe, efficient development and testing of complex robotic behaviors before deployment to physical robots.

With this understanding of digital twin technology, we now have the foundation for creating comprehensive simulation environments that can significantly reduce development time and risk for humanoid robotics projects.