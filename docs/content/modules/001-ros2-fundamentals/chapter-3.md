# Chapter 3: Python Agents → ROS 2 Control with rclpy + Intro to URDF

## Objectives
- Create a Python agent that interfaces with ROS 2 using rclpy
- Understand the structure and purpose of URDF in robot modeling
- Modify a simple humanoid URDF file
- Implement a complete Python agent that controls a simulated humanoid robot

## Introduction to Python Agents with ROS 2

A Python agent in the context of robotics is a program written in Python that can perceive its environment (through sensors), make decisions, and act upon the world (through actuators). With ROS 2's `rclpy` client library, Python becomes a powerful choice for creating various types of robotic agents, from high-level decision makers to direct controller interfaces.

Python agents are particularly useful for:
- High-level planning and decision making
- Prototyping and testing algorithms
- Human-robot interaction interfaces
- Data analysis and visualization

## Creating a Python Agent for Robot Control

Let's build a Python agent that acts as a high-level controller for our humanoid robot. This agent will receive sensor data, make decisions, and send commands to the robot's actuators.

First, create a new package for our agent:

```bash
cd ~/humanoid_ws/src
ros2 pkg create --build-type ament_python humanoid_agent
```

Now, let's create a Python agent that implements a simple state machine for walking control. Create `humanoid_agent/humanoid_agent/walking_agent.py`:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import numpy as np
import math

class WalkingAgent(Node):
    def __init__(self):
        super().__init__('walking_agent')
        
        # Subscribers for sensor data
        self.joint_state_sub = self.create_subscription(
            JointState, 
            '/joint_states', 
            self.joint_state_callback, 
            10
        )
        
        self.imu_sub = self.create_subscription(
            Imu, 
            '/imu/data', 
            self.imu_callback, 
            10
        )
        
        # Publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(
            Twist, 
            '/cmd_vel', 
            10
        )
        
        # Publisher for joint position commands
        self.joint_cmd_pub = self.create_publisher(
            Float64MultiArray, 
            '/joint_commands', 
            10
        )
        
        # Timer for control loop
        self.control_timer = self.create_timer(0.05, self.control_loop)  # 20 Hz
        
        # Internal state
        self.current_joint_states = None
        self.current_imu_data = None
        self.robot_state = "STANDING"  # STANDING, WALKING_FORWARD, WALKING_BACKWARD, TURNING
        self.step_phase = 0.0  # Phase of the walking cycle (0 to 2π)
        
    def joint_state_callback(self, msg):
        self.current_joint_states = msg
        
    def imu_callback(self, msg):
        self.current_imu_data = msg
    
    def control_loop(self):
        """Main control loop for the walking agent"""
        if self.current_joint_states is None or self.current_imu_data is None:
            return
            
        # Decision making based on sensor data and desired behavior
        self.make_decision()
        
        # Generate appropriate commands based on current state
        commands = self.generate_commands()
        
        # Publish commands
        self.cmd_vel_pub.publish(commands['twist'])
        self.joint_cmd_pub.publish(commands['joint_commands'])
    
    def make_decision(self):
        """Make high-level decisions based on sensor data"""
        # Example: If IMU indicates too much tilt, go to recovery state
        if self.current_imu_data is not None:
            orientation_z = self.current_imu_data.orientation.z
            if abs(orientation_z) > 0.3:  # If tilt is too high
                self.robot_state = "RECOVERY"
                return
        
        # For this example, we'll set a simple walking pattern
        # In a more complex agent, this would involve path planning, obstacle detection, etc.
        self.robot_state = "WALKING_FORWARD"
    
    def generate_commands(self):
        """Generate commands based on current state"""
        twist_cmd = Twist()
        joint_cmd = Float64MultiArray()
        
        if self.robot_state == "WALKING_FORWARD":
            # Set forward velocity
            twist_cmd.linear.x = 0.5
            twist_cmd.angular.z = 0.0  # No turning
            
            # Generate walking gait pattern for joints
            # This is a simplified example - real humanoid walking is much more complex
            self.step_phase += 0.1  # Advance the walking cycle
            if self.step_phase > 2 * math.pi:
                self.step_phase = 0.0
                
            # Generate joint positions for a simple walking pattern
            joint_positions = self.generate_walking_pattern()
            joint_cmd.data = joint_positions
            
        elif self.robot_state == "RECOVERY":
            # Stop movement and try to regain balance
            twist_cmd.linear.x = 0.0
            twist_cmd.angular.z = 0.0
            
            # Go to a safe standing position
            joint_cmd.data = [0.0] * 6  # 6 joints, all to neutral position
            
        else:  # STANDING
            twist_cmd.linear.x = 0.0
            twist_cmd.angular.z = 0.0
            joint_cmd.data = [0.0] * 6  # 6 joints, all to neutral position
            
        return {'twist': twist_cmd, 'joint_commands': joint_cmd}
    
    def generate_walking_pattern(self):
        """Generate a simple walking pattern for humanoid joints"""
        # This is a very simplified walking pattern
        # Real humanoid walking requires complex inverse kinematics and dynamics
        
        # Calculate joint positions based on step phase
        left_hip = 0.1 * math.sin(self.step_phase)
        left_knee = 0.05 * math.sin(self.step_phase * 2)
        left_ankle = -0.1 * math.sin(self.step_phase)
        
        right_hip = 0.1 * math.sin(self.step_phase + math.pi)  # Opposite phase
        right_knee = 0.05 * math.sin(self.step_phase * 2 + math.pi)
        right_ankle = -0.1 * math.sin(self.step_phase + math.pi)
        
        return [left_hip, left_knee, left_ankle, right_hip, right_knee, right_ankle]

def main(args=None):
    rclpy.init(args=args)
    agent = WalkingAgent()
    
    try:
        rclpy.spin(agent)
    except KeyboardInterrupt:
        pass
    finally:
        agent.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Understanding URDF (Unified Robot Description Format)

URDF (Unified Robot Description Format) is an XML-based format for representing a robot model. For humanoid robots, URDF defines:

- Physical structure (links)
- Joint connections and types
- Inertial properties
- Visual and collision properties
- Sensor mounting points

A basic URDF for a simplified humanoid leg would look like:

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid_leg">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.1" radius="0.05"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.1" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Hip joint -->
  <joint name="hip_joint" type="revolute">
    <parent link="base_link"/>
    <child link="thigh_link"/>
    <origin xyz="0 0 -0.1" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <!-- Thigh link -->
  <link name="thigh_link">
    <visual>
      <geometry>
        <cylinder length="0.4" radius="0.04"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.4" radius="0.04"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.05" ixy="0" ixz="0" iyy="0.05" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Knee joint -->
  <joint name="knee_joint" type="revolute">
    <parent link="thigh_link"/>
    <child link="shank_link"/>
    <origin xyz="0 0 -0.4" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="0" upper="2.35" effort="100" velocity="1"/>
  </joint>

  <!-- Shank (lower leg) link -->
  <link name="shank_link">
    <visual>
      <geometry>
        <cylinder length="0.4" radius="0.03"/>
      </geometry>
      <material name="green">
        <color rgba="0 1 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.4" radius="0.03"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.5"/>
      <inertia ixx="0.03" ixy="0" ixz="0" iyy="0.03" iyz="0" izz="0.01"/>
    </inertial>
  </link>
</robot>
```

## Modifying a Simple Humanoid URDF

Now, let's create and modify a simple humanoid URDF file. Create `humanoid_agent/urdf/simple_humanoid.urdf`:

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

  <!-- Head -->
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

  <!-- Left Arm -->
  <joint name="left_shoulder_joint" type="revolute">
    <parent link="base_link"/>
    <child link="left_upper_arm"/>
    <origin xyz="0.2 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  </joint>

  <link name="left_upper_arm">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.005"/>
    </inertial>
  </link>

  <joint name="left_elbow_joint" type="revolute">
    <parent link="left_upper_arm"/>
    <child link="left_lower_arm"/>
    <origin xyz="0 0 -0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  </joint>

  <link name="left_lower_arm">
    <visual>
      <geometry>
        <cylinder length="0.25" radius="0.04"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.25" radius="0.04"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.8"/>
      <inertia ixx="0.008" ixy="0" ixz="0" iyy="0.008" iyz="0" izz="0.004"/>
    </inertial>
  </link>

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

  <!-- Right Leg (similar to left, mirrored) -->
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
</robot>
```

## Modifying the URDF

Let's modify our URDF to add sensors, which is a common requirement for humanoid robots. We'll add an IMU to the head link and a camera to the head as well:

Update the `head_link` section in the URDF to include sensor mountings:

```xml
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
    
    <!-- IMU sensor -->
    <sensor name="imu_sensor">
      <origin xyz="0.05 0 0" rpy="0 0 0"/>
      <camera name="head_camera">
        <pose>0.1 0 0 0 0 0</pose>
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
  </link>
```

Actually, the above is not a valid URDF format. Let me provide the correct way to add sensors using Gazebo plugins in a separate file. Create `humanoid_agent/urdf/simple_humanoid.gazebo.xacro`:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <!-- IMU Sensor for Head -->
  <gazebo reference="head_link">
    <sensor name="imu_sensor" type="imu">
      <always_on>true</always_on>
      <update_rate>100</update_rate>
      <visualize>true</visualize>
      <imu>
        <angular_velocity>
          <x>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>2e-4</stddev>
            </noise>
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
  </gazebo>
  
  <!-- Camera Sensor for Head -->
  <gazebo reference="head_link">
    <sensor name="head_camera" type="camera">
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
      <always_on>true</always_on>
      <visualize>true</visualize>
    </sensor>
  </gazebo>
</robot>
```

## Hands-on Exercise 3.1: Complete Python Agent Implementation

1. Complete the `setup.py` file for the `humanoid_agent` package:

```python
from setuptools import setup

package_name = 'humanoid_agent'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Add URDF files
        ('share/' + package_name + '/urdf', [
            'urdf/simple_humanoid.urdf',
            'urdf/simple_humanoid.gazebo.xacro'
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Python agent for controlling humanoid robot',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'walking_agent = humanoid_agent.walking_agent:main',
        ],
    },
)
```

2. Build the package:
```bash
cd ~/humanoid_ws
colcon build --packages-select humanoid_agent
source install/setup.bash
```

3. Run the walking agent:
```bash
ros2 run humanoid_agent walking_agent
```

## Hands-on Exercise 3.2: URDF Modification Task

1. Add two more joints to the right arm of the humanoid (right shoulder and right elbow)
2. Create a second URDF file with a different walking stance (legs further apart)
3. Verify that your URDF file is valid by using the check_urdf command:
   ```bash
   check_urdf path/to/your/urdf/file.urdf
   ```

## Validation Checklist
- [ ] I have implemented a Python agent using rclpy that interfaces with ROS 2
- [ ] I understand the structure and purpose of URDF in robot modeling
- [ ] I have created and modified a simple humanoid URDF file
- [ ] I have added sensor information to the URDF
- [ ] I have tested that my Python agent can communicate with other ROS 2 nodes
- [ ] I have validated my URDF file for syntactic correctness

## Summary

This chapter introduced Python agents for humanoid robot control using the rclpy client library. We created a walking agent that demonstrates how to connect sensor inputs to decision-making and control outputs. We also explored URDF, the standard format for describing robot models in ROS, including how to define links, joints, and sensors.

The combination of Python agents and properly defined URDF files enables sophisticated control of humanoid robots, bridging the gap between high-level behavior and low-level hardware control. This foundation will be essential as we move into more advanced topics in the subsequent modules.

Now that you've completed Module 1, you have a solid foundation in ROS 2 fundamentals, including node communication, Python agent development, and robot modeling with URDF.