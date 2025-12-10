# Quickstart Guide: ROS 2 Fundamentals Module

## Overview
This guide provides a rapid introduction to the ROS 2 Fundamentals module for humanoid robotics. It covers the essential setup and first steps to get started with the educational content.

## Prerequisites
- ROS 2 Humble Hawksbill installed and configured
- Basic Python knowledge (Python 3.8+)
- Docusaurus development environment (for content contributors)
- Git for version control

## Setup for Learners

### 1. Environment Setup
```bash
# Install ROS 2 Humble (if not already installed)
# Follow the official installation guide: https://docs.ros.org/en/humble/Installation.html

# Verify installation
source /opt/ros/humble/setup.bash
ros2 --version
```

### 2. Access the Educational Content
The content for this module is available in MDX format and can be viewed through the Docusaurus-based textbook interface. For hands-on exercises, you'll need to:

1. Clone or download the textbook repository
2. Navigate to the ROS 2 fundamentals module
3. Follow the chapter-by-chapter instructions

### 3. Python Client Library (rclpy)
```bash
# Ensure you have rclpy available in your environment
python3 -c "import rclpy; print('rclpy is available')"
```

## Getting Started with Learning

### Chapter 1: Introduction to ROS 2
**Objective**: Understand what ROS 2 is and why middleware matters for humanoid robots

1. Read the chapter content focusing on:
   - The concept of middleware in robotics
   - How ROS 2 differs from ROS 1
   - The ROS 2 architecture (DDS, nodes, topics, services)

2. Complete the validation checklist at the end of the chapter

### Chapter 2: Hands-on with Nodes, Publishers/Subscriber, Services
**Objective**: Create and run basic ROS 2 nodes with publishers, subscribers, and services

1. Set up your development workspace:
```bash
mkdir -p ~/ros2_fundamentals_ws/src
cd ~/ros2_fundamentals_ws
colcon build
source install/setup.bash
```

2. Follow the step-by-step instructions to create:
   - 2 ROS 2 nodes
   - 1 publisher node
   - 1 subscriber node
   - 1 service server and client

3. Test the communication between nodes

### Chapter 3: Python Agent Integration with rclpy
**Objective**: Link a Python agent to ROS 2 using rclpy

1. Follow the tutorial to create a Python script using rclpy that:
   - Creates a ROS 2 node
   - Publishes messages to topics
   - Subscribes to topics and processes messages
   - Uses services for request-response communication

2. Explore the optional URDF section to:
   - Understand the structure of a simple humanoid URDF
   - Modify a URDF file according to the instructions

## For Content Contributors

### Setting up the Development Environment

1. Install Node.js (v16 or higher)
2. Install Docusaurus:
```bash
npm init docusaurus@latest textbook-app classic
```
3. Install Tailwind CSS and shadcn/ui components as specified in the project constitution

### Content Creation Workflow

1. Create new MDX files following the modular structure
2. Use the custom components for diagrams, callouts, and code blocks
3. Validate against official ROS 2 documentation
4. Ensure all technical claims are accurate

### Testing Content

1. Verify MDX compiles without Docusaurus errors
2. Test all ROS 2 code samples in a Humble environment
3. Validate rclpy agent script connects correctly
4. Check RAG integration for accuracy

## Troubleshooting

### Common Issues
- **ROS 2 not found**: Make sure you've sourced the ROS 2 environment: `source /opt/ros/humble/setup.bash`
- **rclpy unavailable**: Ensure ROS 2 is properly installed and environment is sourced
- **Code examples not working**: Check that you're using ROS 2 Humble and that your Python environment can access rclpy

### Getting Help
- Refer to the official ROS 2 Humble documentation
- Check the validation checklists in each chapter
- Use the RAG system to query the textbook content for clarification