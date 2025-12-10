# Data Model: Digital Twin Module (Gazebo & Unity)

## Key Entities

### Digital Twin
- **Description**: A digital replica of a physical robot or system that enables simulation, analysis, and monitoring of real-world behaviors
- **Attributes**:
  - twin_name: string (identifier for the digital twin)
  - physical_counterpart: string (description of the physical system being replicated)
  - synchronization_frequency: number (how often the twin updates from real world)
  - simulation_accuracy: number (confidence level in digital representation)
- **Relationships**: Maps to physical robot/system, connects Gazebo and Unity environments
- **Validation**: Must maintain consistent state between all representations

### Gazebo Simulation Environment
- **Description**: A physics-based simulation platform that provides realistic modeling of physical interactions, gravity, and collisions
- **Attributes**:
  - world_name: string (name of the simulation environment)
  - physics_engine: string (type of physics engine being used)
  - gravity: vector3 (gravity vector in m/s²)
  - objects: list (physical objects in the simulation)
  - sensors: list (sensors attached to robots or environment)
- **Relationships**: Contains robots, objects, and sensors
- **Validation**: Physics parameters must match real-world values

### Unity Visualization
- **Description**: A high-fidelity rendering environment for creating realistic visual representations of robots and environments
- **Attributes**:
  - scene_name: string (name of the Unity scene)
  - render_pipeline: string (URP/HDRP/Standard)
  - environment_assets: list (3D models, materials, lighting)
  - camera_configurations: list (different viewpoints)
  - interaction_elements: list (objects that can be manipulated)
- **Relationships**: Visualizes the Gazebo simulation environment
- **Validation**: Must accurately represent the physics simulation

### Physics Simulation
- **Description**: The computational modeling of physical properties such as gravity, collisions, and material properties
- **Attributes**:
  - gravity_constant: number (force of gravity in simulation)
  - collision_detection: string (method used for collision detection)
  - material_properties: list (friction, restitution, density)
  - update_rate: number (simulation steps per second)
- **Relationships**: Applied to objects and environments in Gazebo
- **Validation**: Must follow real-world physics laws

### Sensors (LiDAR, Depth, IMU)
- **Description**: Digital sensors that provide data in simulation similar to real-world sensors on physical robots
- **Attributes**:
  - sensor_type: enum ("LiDAR", "Depth", "IMU")
  - sensor_name: string (identifier for the sensor)
  - parent_robot: string (which robot the sensor is attached to)
  - position: vector3 (position relative to parent)
  - orientation: vector4 (orientation relative to parent)
  - data_output: any (the data produced by the sensor)
- **Relationships**: Attached to robots or environment objects
- **Validation**: Output must be in appropriate format for ROS 2

### Gazebo-Unity Bridge
- **Description**: The integration system that connects physics simulation in Gazebo with visualization in Unity
- **Attributes**:
  - synchronization_protocol: string (method for keeping environments in sync)
  - data_translation: object (how data types are converted between engines)
  - update_frequency: number (how often data is synchronized)
  - connection_status: string (current state of the connection)
- **Relationships**: Connects Gazebo simulation and Unity visualization environments
- **Validation**: Must maintain consistent state between both environments

## State Transitions

### For Gazebo Simulation Environment
- State: IDLE → LOADING → RUNNING → PAUSED → STOPPED
- Transitions triggered by simulation controls (start, pause, stop)

### For Sensors
- State: OFF → INITIALIZING → CALIBRATING → ACTIVE → ERROR
- Transitions based on sensor status and output quality

### For Digital Twin
- State: DESIGN → SYNCHRONIZED → MONITORING → ANALYZING → UPDATING
- Transitions based on real-world data input and twin purpose

## Relationships Summary
- Digital Twin encompasses both Gazebo (physics) and Unity (visualization) environments
- Gazebo Simulation Environment contains Robots with attached Sensors
- Unity Visualization renders the same environment and objects as Gazebo
- Gazebo-Unity Bridge maintains synchronization between both environments
- Physics Simulation is applied to objects in Gazebo and reflected in Unity