# Quickstart Guide: Digital Twin Module (Gazebo & Unity)

## Overview
This guide provides a rapid introduction to the Digital Twin module for humanoid robotics using Gazebo and Unity. It covers the essential setup and first steps to get started with the educational content.

## Prerequisites
- Gazebo (Humanoid-supported version - Garden or Citadel)
- Unity LTS
- ROS 2 Humble
- Docusaurus development environment (for content contributors)
- Git for version control

## Setup for Learners

### 1. Environment Setup
```bash
# Install Gazebo Garden (or appropriate version)
# Follow the official installation guide: https://gazebosim.org/docs/garden/install

# Install Unity LTS
# Download from https://unity.com/releases/editor/lts-release and install

# Install ROS 2 Humble
# Follow the official installation guide: https://docs.ros.org/en/humble/Installation.html

# Verify installation
source /opt/ros/humble/setup.bash
gz --version  # For Gazebo Garden
```

### 2. Access the Educational Content
The content for this module is available in MDX format and can be viewed through the Docusaurus-based textbook interface. For hands-on exercises, you'll need to:

1. Clone or download the textbook repository
2. Navigate to the Digital Twin module
3. Follow the chapter-by-chapter instructions

### 3. Gazebo and Unity Integration Setup
```bash
# Create a workspace directory
mkdir -p ~/digital_twin_ws
cd ~/digital_twin_ws

# Build any necessary ROS 2 packages for the tutorials
colcon build
source install/setup.bash
```

## Getting Started with Learning

### Chapter 1: Digital Twin Concepts
**Objective**: Understand what digital twins are and why they matter for humanoid robotics

1. Read the chapter content focusing on:
   - The concept of a digital twin
   - The value proposition in robotics
   - How digital twins apply to humanoid robots

2. Complete the validation checklist at the end of the chapter

### Chapter 2: Gazebo Simulation Fundamentals
**Objective**: Create a Gazebo simulation with physics and at least one sensor

1. Set up your Gazebo environment:
```bash
# Launch an empty world to verify Gazebo is working
gz sim -v 4 empty.sdf
```

2. Follow the step-by-step instructions to create:
   - A Gazebo world with physics simulation
   - At least one sensor (LiDAR, Depth, or IMU)
   - A humanoid robot model in the simulation

3. Verify that the simulation runs with realistic physics and sensor data

### Chapter 3: Unity High-Fidelity Rendering & Interaction
**Objective**: Create a Unity scene showing humanoid interaction and understand Gazebo-Unity bridging

1. Launch Unity and create a new 3D project

2. Import the necessary assets:
   - Humanoid robot model
   - Environment assets
   - Any required packages for robotics simulation

3. Follow the tutorial to:
   - Create a realistic Unity scene
   - Implement humanoid interaction elements
   - Understand the concept of Gazebo-Unity bridging (though full implementation may require advanced tools)

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
2. Use the custom components for diagrams, callouts, simulation concepts, and code blocks
3. Validate against official Gazebo and Unity documentation
4. Ensure all technical claims are accurate

### Testing Content

1. Verify MDX compiles without Docusaurus errors
2. Test all Gazebo simulation examples in the appropriate environment
3. Verify Unity scene instructions are reproducible
4. Check RAG integration for accuracy

## Troubleshooting

### Common Issues
- **Gazebo not launching**: Make sure you've sourced the ROS 2 environment: `source /opt/ros/humble/setup.bash`
- **Unity project errors**: Check that you're using Unity LTS version
- **Simulation examples not working**: Verify that your Gazebo version matches the requirements

### Getting Help
- Refer to the official Gazebo and Unity documentation
- Check the validation checklists in each chapter
- Use the RAG system to query the textbook content for clarification