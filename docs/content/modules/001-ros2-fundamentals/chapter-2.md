# Chapter 2: Hands-on with Nodes, Publishers, Subscribers, Services, Message Flow

## Objectives
- Create and run ROS 2 nodes with publishers and subscribers
- Implement a service server and client
- Understand message flow in ROS 2
- Build a complete communication system with 2 nodes, 1 publisher, 1 subscriber, and 1 service

## Introduction

In this chapter, we'll move from theory to practice by implementing the core communication patterns in ROS 2. We'll create working examples that demonstrate how nodes communicate with each other through topics (publishers/subscribers) and services.

## Creating a Simple Publisher Node

Let's create a node that publishes joint state information (simulating data from a humanoid robot's sensors). We'll use Python with the `rclpy` client library.

First, create a package for our examples:

```bash
cd ~/humanoid_ws/src
ros2 pkg create --build-type ament_python joint_publisher
```

Now, let's implement the publisher node. Create a file `joint_publisher/joint_publisher/joint_publisher_node.py`:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

class JointPublisher(Node):
    def __init__(self):
        super().__init__('joint_publisher')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        
        # Define joint names for a simple humanoid (e.g., hip, knee, ankle)
        msg.name = ['left_hip_joint', 'left_knee_joint', 'left_ankle_joint', 
                   'right_hip_joint', 'right_knee_joint', 'right_ankle_joint']
        
        # Simulate changing joint positions (in radians)
        msg.position = [0.1 * self.i, 0.05 * self.i, 0.02 * self.i,
                       -0.1 * self.i, -0.05 * self.i, -0.02 * self.i]
        
        # Simulate velocities and efforts
        msg.velocity = [0.01] * 6  # 6 joints
        msg.effort = [0.5] * 6     # 6 joints
        
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing joint states: {msg.position}')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    joint_publisher = JointPublisher()
    rclpy.spin(joint_publisher)
    joint_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Creating a Simple Subscriber Node

Now let's create a subscriber node that receives the joint states and processes them. Create a file `joint_publisher/joint_publisher/joint_subscriber_node.py`:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointSubscriber(Node):
    def __init__(self):
        super().__init__('joint_subscriber')
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(
            f'Received joint states - Positions: {msg.position}, '
            f'Velocities: {msg.velocity}, Efforts: {msg.effort}'
        )

def main(args=None):
    rclpy.init(args=args)
    joint_subscriber = JointSubscriber()
    rclpy.spin(joint_subscriber)
    joint_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Creating a Service Server

Now let's implement a service that allows other nodes to request specific gait patterns for our humanoid robot. Create `joint_publisher/joint_publisher/gait_service.py`:

```python
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

class GaitService(Node):
    def __init__(self):
        super().__init__('gait_service')
        self.srv = self.create_service(Trigger, 'set_gait_pattern', self.set_gait_callback)

    def set_gait_callback(self, request, response):
        # In a real robot, this would set the gait pattern
        # For this example, we'll just return success
        self.get_logger().info('Setting gait pattern requested')
        response.success = True
        response.message = 'Gait pattern set successfully'
        return response

def main(args=None):
    rclpy.init(args=args)
    gait_service = GaitService()
    rclpy.spin(gait_service)
    gait_service.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Creating a Service Client

Now, let's create a client that can call our gait service. Create `joint_publisher/joint_publisher/gait_client.py`:

```python
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

class GaitClient(Node):
    def __init__(self):
        super().__init__('gait_client')
        self.cli = self.create_client(Trigger, 'set_gait_pattern')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = Trigger.Request()

    def send_request(self):
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    gait_client = GaitClient()
    response = gait_client.send_request()
    if response is not None:
        gait_client.get_logger().info(f'Result: {response.success}, {response.message}')
    else:
        gait_client.get_logger().info('Service call failed')

    gait_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Setting up the Package

To make our nodes executable, we need to update the `setup.py` file in the `joint_publisher` directory:

```python
from setuptools import setup

package_name = 'joint_publisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Simple joint publisher example for humanoid robot',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joint_publisher = joint_publisher.joint_publisher_node:main',
            'joint_subscriber = joint_publisher.joint_subscriber_node:main',
            'gait_service = joint_publisher.gait_service:main',
            'gait_client = joint_publisher.gait_client:main',
        ],
    },
)
```

## Testing the Complete System

To test our communication system, we'll run the nodes in separate terminals:

1. Build the package:
```bash
cd ~/humanoid_ws
colcon build --packages-select joint_publisher
source install/setup.bash
```

2. In Terminal 1, run the publisher:
```bash
ros2 run joint_publisher joint_publisher
```

3. In Terminal 2, run the subscriber:
```bash
ros2 run joint_publisher joint_subscriber
```

4. In Terminal 3, run the service:
```bash
ros2 run joint_publisher gait_service
```

5. In Terminal 4, run the client to test the service:
```bash
ros2 run joint_publisher gait_client
```

You should see the publisher sending messages, the subscriber receiving them, and the service responding to client requests.

## Understanding Message Flow

The message flow in our system works as follows:

1. The `joint_publisher` node publishes `JointState` messages to the `joint_states` topic every 0.1 seconds
2. The `joint_subscriber` node subscribes to the `joint_states` topic and processes incoming messages
3. The `gait_service` node listens for service requests on the `set_gait_pattern` service
4. The `gait_client` node sends requests to the `set_gait_pattern` service and receives responses

This demonstrates the fundamental communication patterns in ROS 2:
- **Topics** for asynchronous one-to-many communication (publisher-subscriber)
- **Services** for synchronous request-response communication

## Quality of Service (QoS) Considerations

For humanoid robots, the timing of messages is critical for safety and performance. ROS 2 provides Quality of Service (QoS) profiles to handle different communication requirements:

```python
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

# For critical control messages, use reliable delivery
qos_profile = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10
)

# For sensor data that can tolerate some loss, use best effort
sensor_qos = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=5
)
```

## Hands-on Exercise 2.1: Implement Your Own Communication System

1. Create a new package called `humanoid_controller`:
   ```bash
   cd ~/humanoid_ws/src
   ros2 pkg create --build-type ament_python humanoid_controller
   ```

2. Implement a node that publishes head camera images (use the `sensor_msgs/Image` message type)

3. Create a subscriber that processes these images and publishes detected objects

4. Implement an action server for head movement (use the `control_msgs/FollowJointTrajectory` action)

5. Test that all nodes can communicate properly

## Validation Checklist
- [ ] I have created a publisher node that sends joint state messages
- [ ] I have created a subscriber node that receives joint state messages
- [ ] I have implemented a service server and client
- [ ] I understand the difference between topics and services
- [ ] I have tested the complete communication system with 2 nodes (publisher + subscriber) and 1 service
- [ ] I understand Quality of Service concepts and their importance for humanoid robots

## Summary

This chapter provided hands-on experience with creating and running ROS 2 nodes with different communication patterns. We implemented a publisher-subscriber pair for continuous data flow and a service-server/client pair for request-response interactions. We also explored Quality of Service considerations that are critical for humanoid robot applications.

In the next chapter, we'll explore how to connect Python agents to ROS 2 using rclpy and introduce URDF for robot modeling.