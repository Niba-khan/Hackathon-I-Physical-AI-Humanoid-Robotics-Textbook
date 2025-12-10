# Chapter 2: Gazebo Simulation Fundamentals - Physics Engine Basics, Environment Setup, Sensor Integration

## Objectives
- Understand Gazebo's physics engine and its capabilities
- Set up a basic simulation environment for humanoid robots
- Integrate various sensors (LiDAR, Depth, IMU) into the simulation
- Create and configure simulation worlds with realistic physics

## Introduction to Gazebo

Gazebo is a 3D simulation environment that enables the accurate and efficient testing of robotics algorithms, designs, and scenarios. For humanoid robotics, Gazebo provides:

- **Realistic Physics**: Accurate simulation of forces, collisions, and dynamics
- **Sensor Simulation**: Implementation of cameras, LiDAR, IMU, GPS, and other sensors
- **Visual Rendering**: High-quality 3D rendering for visualization
- **Plugin Architecture**: Extensible framework for custom sensors, actuators, and controllers

Gazebo integrates seamlessly with ROS/ROS 2, making it an ideal platform for testing humanoid robot algorithms before deployment on physical robots.

## Understanding Gazebo's Physics Engine

Gazebo uses the Open-Source Physics Engine (OSPE) as its foundation, with support for multiple physics engines:

### ODE (Open Dynamics Engine)
- Most commonly used physics engine in Gazebo
- Good balance between accuracy and performance
- Well-suited for humanoid robots with articulated joints
- Handles collisions, friction, and dynamics effectively

### Bullet Physics
- More advanced collision detection
- Better for complex contact scenarios
- More computationally intensive than ODE
- Useful for precise manipulation tasks

### Simbody
- Multibody dynamics engine
- Handles complex articulated systems well
- More accurate for systems with many constraints
- Less commonly used in robotics applications

## Setting Up a Basic Gazebo Environment

### 1. Gazebo Installation and Setup

First, ensure you have Gazebo installed along with the necessary ROS interfaces:

```bash
# For ROS 2 Humble with Gazebo Garden
sudo apt-get install ros-humble-gazebo-ros-pkgs
sudo apt-get install gazebo
```

### 2. Creating Your First World File

World files in Gazebo are written in SDF (Simulation Description Format) and define the environment, physics parameters, and initial robot placement. Create `my_humanoid_world.sdf`:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="humanoid_world">
    <!-- Physics parameters -->
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000.0</real_time_update_rate>
      <gravity>0 0 -9.8</gravity>
    </physics>

    <!-- Include common models -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Add a simple floor -->
    <model name="floor">
      <pose>0 0 0 0 0 0</pose>
      <link name="floor_link">
        <collision name="floor_collision">
          <geometry>
            <box>
              <size>10 10 0.1</size>
            </box>
          </geometry>
        </collision>
        <visual name="floor_visual">
          <geometry>
            <box>
              <size>10 10 0.1</size>
            </box>
          </geometry>
          <material>
            <ambient>0.7 0.7 0.7 1</ambient>
            <diffuse>0.7 0.7 0.7 1</diffuse>
            <specular>0.7 0.7 0.7 1</specular>
          </material>
        </visual>
        <pose>0 0 -0.05 0 0 0</pose>
      </link>
    </model>

    <!-- Add a simple obstacle -->
    <model name="obstacle">
      <pose>2 0 0 0 0 0</pose>
      <link name="obstacle_link">
        <collision name="obstacle_collision">
          <geometry>
            <box>
              <size>0.5 0.5 1.0</size>
            </box>
          </geometry>
        </collision>
        <visual name="obstacle_visual">
          <geometry>
            <box>
              <size>0.5 0.5 1.0</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.3 0.3 1</ambient>
            <diffuse>0.8 0.3 0.3 1</diffuse>
            <specular>0.8 0.3 0.3 1</specular>
          </material>
        </visual>
      </link>
    </model>

    <!-- Light source -->
    <light name="sun_light" type="directional">
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 -0.1 -0.9</direction>
    </light>
  </world>
</sdf>
```

### 3. Running Your World

To run your world file:

```bash
gz sim -r my_humanoid_world.sdf
```

## Physics Engine Configuration for Humanoid Robots

Humanoid robots have specific requirements for physics simulation:

### Gravity Settings
For Earth-based simulation:
```xml
<gravity>0 0 -9.8</gravity>
```

### Time Step Configuration
Humanoid robots require precise physics simulation:
```xml
<max_step_size>0.001</max_step_size> <!-- 1ms time steps for accuracy -->
<real_time_factor>1.0</real_time_factor> <!-- Run at real-time speed -->
<real_time_update_rate>1000.0</real_time_update_rate> <!-- 1000 Hz update rate -->
```

### Solver Parameters
For humanoid robots with multiple joints:
```xml
<ode>
  <solver>
    <type>quick</type>
    <iters>1000</iters> <!-- More iterations for stability -->
    <sor>1.3</sor>
  </solver>
  <constraints>
    <cfm>0.000001</cfm> <!-- Constraint Force Mixing -->
    <erp>0.2</erp> <!-- Error Reduction Parameter -->
    <contact_max_correcting_vel>100.0</contact_max_correcting_vel>
    <contact_surface_layer>0.001</contact_surface_layer>
  </constraints>
</ode>
```

## Sensor Integration in Gazebo

### 1. IMU (Inertial Measurement Unit)

Add an IMU sensor to a robot model by including it in the URDF or SDF:

```xml
<sensor name="imu_sensor" type="imu">
  <always_on>1</always_on>
  <update_rate>100</update_rate>
  <visualize>false</visualize>
  <topic>imu/data</topic>
  <imu>
    <angular_velocity>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>2e-4</stddev>
        </ne>
      </x>
      <y>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>2e-4</stddev>
        </noise>
      </y>
      <z>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>2e-4</stddev>
        </noise>
      </z>
    </angular_velocity>
    <linear_acceleration>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>1.7e-2</stddev>
        </noise>
      </x>
      <y>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>1.7e-2</stddev>
        </noise>
      </y>
      <z>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>1.7e-2</stddev>
        </noise>
      </z>
    </linear_acceleration>
  </imu>
</sensor>
```

### 2. Depth Camera

A depth camera is crucial for humanoid robots for navigation and obstacle detection:

```xml
<sensor name="depth_camera" type="depth">
  <always_on>true</always_on>
  <update_rate>30</update_rate>
  <camera name="head">
    <horizontal_fov>1.047</horizontal_fov>
    <image>
      <width>640</width>
      <height>480</height>
    </image>
    <clip>
      <near>0.1</near>
      <far>10.0</far>
    </clip>
  </camera>
  <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
    <frame_name>camera_frame</frame_name>
    <topic_name>depth_camera/image_raw</topic_name>
    <point_cloud_topic_name>depth_camera/points</point_cloud_topic_name>
  </plugin>
</sensor>
```

### 3. LiDAR Sensor

LiDAR is essential for mapping and navigation:

```xml
<sensor name="lidar" type="ray">
  <pose>0.1 0 0.1 0 0 0</pose>
  <ray>
    <scan>
      <horizontal>
        <samples>720</samples>
        <resolution>1</resolution>
        <min_angle>-3.14159</min_angle>
        <max_angle>3.14159</max_angle>
      </horizontal>
    </scan>
    <range>
      <min>0.1</min>
      <max>30.0</max>
      <resolution>0.01</resolution>
    </range>
  </ray>
  <plugin name="lidar_controller" filename="libgazebo_ros_laser.so">
    <topic_name>scan</topic_name>
    <frame_name>lidar_frame</frame_name>
  </plugin>
</sensor>
```

## Creating a Humanoid Robot Model with Sensors

Let's create a simple humanoid robot model with integrated sensors. Create `simple_humanoid_with_sensors.urdf`:

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">
  <!-- Base/Body of the robot -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
      <material name="light_grey">
        <color rgba="0.7 0.7 0.7 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10.0"/>
      <inertia ixx="1.0" ixy="0" ixz="0" iyy="1.0" iyz="0" izz="1.0"/>
    </inertial>
  </link>

  <!-- Head with sensors -->
  <joint name="neck_joint" type="revolute">
    <parent link="base_link"/>
    <child link="head_link"/>
    <origin xyz="0 0 0.35" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.785" upper="0.785" effort="10" velocity="1"/>
  </joint>

  <link name="head_link">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.04" ixy="0" ixz="0" iyy="0.04" iyz="0" izz="0.04"/>
    </inertial>
  </link>

  <!-- IMU in the head -->
  <gazebo reference="head_link">
    <sensor name="imu_sensor" type="imu">
      <always_on>true</always_on>
      <update_rate>100</update_rate>
      <visualize>false</visualize>
    </sensor>
  </gazebo>

  <!-- Depth Camera in the head -->
  <gazebo reference="head_link">
    <sensor name="depth_camera" type="depth">
      <always_on>true</always_on>
      <update_rate>30</update_rate>
      <camera name="head_camera">
        <horizontal_fov>1.047</horizontal_fov>
        <image>
          <width>640</width>
          <height>480</height>
        </image>
        <clip>
          <near>0.1</near>
          <far>10.0</far>
        </clip>
      </camera>
    </sensor>
  </gazebo>

  <!-- Left Leg -->
  <joint name="left_hip_joint" type="revolute">
    <parent link="base_link"/>
    <child link="left_upper_leg"/>
    <origin xyz="-0.1 0 -0.25" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="50" velocity="1"/>
  </joint>

  <link name="left_upper_leg">
    <visual>
      <geometry>
        <cylinder length="0.4" radius="0.06"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.4" radius="0.06"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="3.0"/>
      <inertia ixx="0.04" ixy="0" ixz="0" iyy="0.04" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <joint name="left_knee_joint" type="revolute">
    <parent link="left_upper_leg"/>
    <child link="left_lower_leg"/>
    <origin xyz="0 0 -0.4" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="0" upper="2.35" effort="50" velocity="1"/>
  </joint>

  <link name="left_lower_leg">
    <visual>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.5"/>
      <inertia ixx="0.03" ixy="0" ixz="0" iyy="0.03" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Right Leg -->
  <joint name="right_hip_joint" type="revolute">
    <parent link="base_link"/>
    <child link="right_upper_leg"/>
    <origin xyz="0.1 0 -0.25" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="50" velocity="1"/>
  </joint>

  <link name="right_upper_leg">
    <visual>
      <geometry>
        <cylinder length="0.4" radius="0.06"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.4" radius="0.06"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="3.0"/>
      <inertia ixx="0.04" ixy="0" ixz="0" iyy="0.04" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <joint name="right_knee_joint" type="revolute">
    <parent link="right_upper_leg"/>
    <child link="right_lower_leg"/>
    <origin xyz="0 0 -0.4" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="0" upper="2.35" effort="50" velocity="1"/>
  </joint>

  <link name="right_lower_leg">
    <visual>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.5"/>
      <inertia ixx="0.03" ixy="0" ixz="0" iyy="0.03" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <!-- LiDAR on top of the body -->
  <gazebo reference="base_link">
    <sensor name="lidar" type="ray">
      <pose>0.0 0.0 0.3 0 0 0</pose>
      <ray>
        <scan>
          <horizontal>
            <samples>720</samples>
            <resolution>1</resolution>
            <min_angle>-3.14159</min_angle>
            <max_angle>3.14159</max_angle>
          </horizontal>
        </scan>
        <range>
          <min>0.1</min>
          <max>30.0</max>
          <resolution>0.01</resolution>
        </range>
      </ray>
    </sensor>
  </gazebo>
</robot>
```

## Loading Your Robot in Gazebo

To load your robot model in Gazebo, you'll need to create a launch file or use ROS2 to spawn the robot. Here's a simple Python script that launches Gazebo and spawns your robot:

Create a package for your simulation:

```bash
cd ~/humanoid_ws/src
ros2 pkg create --build-type ament_python simulation_examples
```

Create a launch file `simulation_examples/launch/spawn_humanoid.launch.py`:

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Path to the Gazebo launch file
    gazebo_launch = PathJoinSubstitution([
        FindPackageShare('gazebo_ros'),
        'launch',
        'gazebo.launch.py'
    ])

    # Include Gazebo launch
    gazebo_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch),
        launch_arguments={'world': 'path_to_your_world.sdf'}.items()
    )

    # Robot state publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': open('path_to_your_robot.urdf', 'r').read()
        }]
    )

    # Spawn entity node
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'simple_humanoid'
        ],
        output='screen'
    )

    return LaunchDescription([
        gazebo_launch_cmd,
        robot_state_publisher,
        spawn_entity
    ])
```

## Physics Parameters Optimization for Humanoid Simulation

For humanoid robots, specific physics parameters need optimization:

### Stability Considerations
- **Time Step**: Use smaller time steps (1ms) for better accuracy with many joints
- **ERP and CFM**: Adjust constraint parameters to balance stability and accuracy
- **Constraint Iterations**: Increase iterations for models with many joints to improve stability

### Performance Considerations
- **Collision Detection**: Use simplified collision meshes where possible
- **Update Rates**: Balance sensor update rates with computational performance
- **Solver Types**: Choose appropriate solver based on simulation needs

## Hands-on Exercise 2.1: Create Your Own Simulation Environment

1. Create a new world file with a more complex environment containing:
   - Multiple obstacles with different shapes and sizes
   - Different surface materials (grass, concrete, etc.)
   - At least 3 different lighting conditions

2. Create a humanoid robot model with the following sensors:
   - IMU in the torso
   - Depth camera in the head
   - LiDAR on the torso
   - Force-torque sensors in the feet

3. Set up the physics parameters optimized for humanoid robot simulation

4. Launch your simulation and verify that all sensors are publishing data to ROS topics

## Validation Checklist
- [ ] I understand Gazebo's physics engine and its capabilities
- [ ] I can create and configure a basic simulation environment
- [ ] I have successfully integrated multiple sensor types (IMU, Depth, LiDAR) into my simulation
- [ ] I know how to optimize physics parameters for humanoid robot simulation
- [ ] I have created a complete humanoid robot model with integrated sensors
- [ ] I have tested that my simulation environment works properly with appropriate physics

## Summary

This chapter provided a comprehensive overview of Gazebo simulation fundamentals with a focus on humanoid robotics. We covered the physics engine capabilities, environment setup procedures, and how to integrate key sensors (LiDAR, Depth, IMU) into the simulation. We also created a complete humanoid robot model with sensors and discussed optimization techniques for physics simulation.

In the next chapter, we'll explore Unity for high-fidelity visualization and the integration between Gazebo and Unity for a complete digital twin solution.