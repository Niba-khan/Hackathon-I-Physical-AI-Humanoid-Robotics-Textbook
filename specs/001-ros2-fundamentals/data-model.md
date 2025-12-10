# Data Model: ROS 2 Fundamentals Module

## Key Entities

### ROS 2 Node
- **Description**: A process that performs computation and communicates with other nodes through topics and services
- **Attributes**: 
  - node_name: string (unique identifier)
  - node_namespace: string (optional grouping)
  - communication_interfaces: list (publishers, subscribers, services)
- **Relationships**: Communicates with other nodes via topics and services
- **Validation**: Node name must be unique within namespace

### Topic/Message
- **Description**: Communication mechanism where publishers send data to subscribers; unidirectional data flow
- **Attributes**:
  - topic_name: string (identifier for the communication channel)
  - message_type: string (defines structure of data)
  - publishers: list (nodes sending data)
  - subscribers: list (nodes receiving data)
- **Relationships**: Connects publishers to subscribers
- **Validation**: Topic must have matching message types between publisher and subscriber

### Service
- **Description**: Communication mechanism for request-response interactions between nodes; bidirectional synchronous communication
- **Attributes**:
  - service_name: string (identifier for the service)
  - request_type: string (structure of request data)
  - response_type: string (structure of response data)
  - server: node (the node providing the service)
  - clients: list (nodes using the service)
- **Relationships**: Connects service clients to service server
- **Validation**: Request and response types must match between client and server

### URDF (Unified Robot Description Format)
- **Description**: XML format for representing a robot model's structure, geometry, and properties
- **Attributes**:
  - robot_name: string (name of the robot)
  - links: list (rigid bodies of the robot)
  - joints: list (connections between links)
  - materials: list (visual properties)
- **Relationships**: Describes the structure of a robot
- **Validation**: Must conform to URDF XML schema

### rclpy
- **Description**: Python client library for ROS 2 that enables Python programs to interact with ROS 2 systems
- **Attributes**:
  - node_interface: interface (allows creating ROS 2 nodes)
  - publisher_interface: interface (allows publishing messages)
  - subscriber_interface: interface (allows subscribing to messages)
  - service_interface: interface (allows creating services)
- **Relationships**: Interfaces with ROS 2 middleware
- **Validation**: Must be compatible with ROS 2 Humble

## State Transitions

### For ROS 2 Node
- State: UNCONFIGURED → INACTIVE → ACTIVE → FINALIZED
- Transitions triggered by lifecycle management (if using lifecycle nodes)

### For Service Communication
- State: IDLE → REQUEST_RECEIVED → PROCESSING → RESPONSE_SENT
- Transitions occur during request-response cycle

## Relationships Summary
- ROS 2 Nodes communicate through Topics (publishers/subscribers) and Services
- URDF describes the physical structure of robots that ROS 2 nodes may control
- rclpy provides the Python interface to create and manage ROS 2 nodes