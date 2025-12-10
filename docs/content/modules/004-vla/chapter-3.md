# Chapter 3: Cognitive Planning & Capstone - NL → ROS 2 Action Mapping, Navigation, Obstacle Avoidance, Object ID & Manipulation

## Objectives
- Implement cognitive planning for natural language to action mapping
- Map natural language commands to ROS 2 action sequences
- Integrate navigation, obstacle avoidance, and object manipulation
- Execute the full capstone pipeline: voice → plan → navigate → identify → manipulate
- Validate the complete VLA system performance
- Understand the integration challenges in the full pipeline

## Cognitive Planning for Natural Language to Robot Action

### Understanding Cognitive Planning

Cognitive planning in the context of Vision-Language-Action (VLA) systems involves translating high-level natural language commands into executable robotic actions. This is a multi-stage process that bridges the gap between human intentions expressed in language and the physical capabilities of a humanoid robot.

Key aspects of cognitive planning:
- **Hierarchical Task Decomposition**: Breaking complex commands into simpler, executable subtasks
- **Semantic Grounding**: Connecting abstract language concepts to concrete visual and spatial entities
- **Action Selection**: Choosing appropriate robot behaviors for task execution
- **Constraint Satisfaction**: Ensuring planned actions respect physical, safety, and operational constraints

### Planning Architecture

The cognitive planning system follows this architecture:

```
Natural Language Command
        ↓
[Semantic Parser] → [Task Decomposer]
        ↓                 ↓
[Context Resolver] → [Action Selector]  
        ↓                 ↓
[Visual Grounding] → [Motion Planners]
        ↓                 ↓
[Constraint Checker] → [Execution Scheduler]
        ↓                 ↓
[Validation Layer] → [Action Sequence]
        ↓
[ROS 2 Action Publishers]
```

### Implementation Framework

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose, PoseStamped
from action_msgs.msg import GoalStatus

class CognitivePlanningNode(Node):
    def __init__(self):
        super().__init__('cognitive_planning_node')
        
        # Subscriptions
        self.intent_sub = self.create_subscription(
            String,
            '/validated_intent',
            self.intent_callback,
            10
        )
        
        self.scene_desc_sub = self.create_subscription(
            String,  # In practice, this would be a more structured message
            '/scene_description',
            self.scene_desc_callback,
            10
        )
        
        # Publishers
        self.action_sequence_pub = self.create_publisher(
            String,  # Structured action message in practice
            '/action_sequence',
            10
        )
        
        self.nav_goal_pub = self.create_publisher(
            PoseStamped,
            '/goal_pose',
            10
        )
        
        self.manipulation_cmd_pub = self.create_publisher(
            String,  # Manipulation command message
            '/manipulation_command',
            10
        )
        
        # Internal state
        self.current_scene = {}
        self.robot_capabilities = self.define_robot_capabilities()
        self.planning_context = {}
        
        self.get_logger().info('Cognitive Planning Node initialized')
    
    def define_robot_capabilities(self):
        """Define the robot's available actions and constraints"""
        return {
            'navigation': {
                'max_speed': 0.5,  # m/s
                'min_turn_radius': 0.3,  # m
                'max_slope': 15,  # degrees
            },
            'manipulation': {
                'reach_range': {'min': 0.1, 'max': 0.8},  # m
                'weight_limit': 2.0,  # kg
                'grip_types': ['pinch', 'power', 'hook'],
                'precision_modes': ['coarse', 'fine', 'very_fine']
            },
            'locomotion': {
                'step_size': {'max_forward': 0.3, 'max_sideways': 0.15, 'max_backward': 0.1},  # m
                'turn_rate': 0.5,  # rad/s
                'balance_constraints': {
                    'max_tilt': 0.2,  # rad
                    'zmp_limits': {'x': 0.1, 'y': 0.05}  # m
                }
            },
            'perception': {
                'fov_horizontal': 90,  # degrees
                'fov_vertical': 60,
                'min_detection_range': 0.1,  # m
                'max_detection_range': 3.0,  # m
                'recognition_accuracy': 0.85  # threshold
            }
        }
    
    def intent_callback(self, msg):
        """Process validated intent and generate action plan"""
        try:
            intent_data = json.loads(msg.data)
            
            # Create planning context from intent and available scene info
            self.planning_context = {
                'intent': intent_data,
                'scene': self.current_scene,
                'capabilities': self.robot_capabilities
            }
            
            # Generate action plan
            action_plan = self.generate_action_plan(intent_data)
            
            if action_plan:
                # Validate the action plan
                validated_plan = self.validate_action_plan(action_plan)
                
                if validated_plan:
                    # Execute the action plan
                    self.execute_action_plan(validated_plan)
                    
                    # Publish action sequence for downstream components
                    plan_msg = String()
                    plan_msg.data = json.dumps(validated_plan)
                    self.action_sequence_pub.publish(plan_msg)
                    
                    self.get_logger().info(f'Action plan executed: {len(validated_plan)} steps')
                else:
                    self.get_logger().warn('Action plan validation failed')
            else:
                self.get_logger().warn('Could not generate action plan')
                
        except Exception as e:
            self.get_logger().error(f'Cognitive planning error: {e}')
    
    def generate_action_plan(self, intent_data):
        """Generate action plan from intent and context"""
        command_type = intent_data.get('command_type')
        
        if command_type == 'navigation':
            return self.plan_navigation(intent_data)
        elif command_type == 'fetch':
            return self.plan_fetch_object(intent_data)
        elif command_type == 'manipulation':
            return self.plan_manipulation(intent_data)
        elif command_type == 'interaction':
            return self.plan_interaction(intent_data)
        else:
            return self.plan_generic(intent_data)
    
    def plan_navigation(self, intent_data):
        """Plan navigation action sequence"""
        entities = intent_data.get('entities', {})
        target_location = entities.get('location') or entities.get('destination')
        
        if not target_location:
            self.get_logger().error('No navigation target specified')
            return None
        
        # Find target pose in scene description
        target_pose = self.find_target_pose(target_location)
        
        if not target_pose:
            self.get_logger().warn(f'Target location "{target_location}" not found in scene')
            return None
        
        # Create navigation plan
        plan = []
        
        # 1. Localize robot (if needed)
        plan.append({
            'action': 'localize',
            'description': 'Ensure accurate robot localization'
        })
        
        # 2. Plan path to target
        plan.append({
            'action': 'path_planning',
            'target': target_pose,
            'description': f'Plan path to {target_location}'
        })
        
        # 3. Execute navigation
        plan.append({
            'action': 'navigation_execution',
            'target_pose': target_pose,
            'parameters': {
                'speed': 'normal',
                'obstacle_avoidance': 'active'
            },
            'description': f'Navigate to {target_location}'
        })
        
        # 4. Confirm arrival
        plan.append({
            'action': 'arrival_confirmation',
            'description': 'Confirm arrival at destination'
        })
        
        return plan
    
    def plan_fetch_object(self, intent_data):
        """Plan object fetching action sequence"""
        entities = intent_data.get('entities', {})
        target_object = entities.get('object') or entities.get('target')
        
        if not target_object:
            self.get_logger().error('No object specified for fetching')
            return None
        
        # Find object in scene description
        object_info = self.find_object_in_scene(target_object)
        
        if not object_info:
            self.get_logger().warn(f'Target object "{target_object}" not found in scene')
            return None
        
        # Verify object is graspable
        if not self.is_object_graspable(object_info):
            self.get_logger().warn(f'Target object "{target_object}" is not graspable')
            return None
        
        plan = []
        
        # 1. Navigate to object location
        approach_pose = self.calculate_approach_pose(object_info['pose'])
        plan.append({
            'action': 'navigation_execution',
            'target_pose': approach_pose,
            'parameters': {
                'speed': 'careful',
                'obstacle_avoidance': 'active'
            },
            'description': f'Approach {target_object}'
        })
        
        # 2. Verify object position
        plan.append({
            'action': 'object_verification',
            'target_object': target_object,
            'description': f'Confirm {target_object} location and state'
        })
        
        # 3. Grasp planning
        grasp_pose = self.calculate_grasp_pose(object_info)
        grasp_type = self.select_grasp_type(object_info)
        
        plan.append({
            'action': 'grasp_planning',
            'object_info': object_info,
            'grasp_pose': grasp_pose,
            'grasp_type': grasp_type,
            'description': f'Plan grasp of {target_object}'
        })
        
        # 4. Execute grasp
        plan.append({
            'action': 'execute_grasp',
            'grasp_pose': grasp_pose,
            'grasp_type': grasp_type,
            'description': f'Grasp {target_object}'
        })
        
        # 5. Transport to destination (if specified)
        if 'destination' in entities:
            destination = entities['destination']
            dest_pose = self.find_target_pose(destination)
            
            if dest_pose:
                plan.append({
                    'action': 'navigation_execution',
                    'target_pose': dest_pose,
                    'parameters': {
                        'speed': 'careful',  # Slower with object
                        'obstacle_avoidance': 'active'
                    },
                    'description': f'Transport {target_object} to {destination}'
                })
        
        return plan
    
    def plan_manipulation(self, intent_data):
        """Plan manipulation action sequence"""
        entities = intent_data.get('entities', {})
        target_object = entities.get('object')
        target_location = entities.get('destination') or entities.get('target')
        
        if not target_object or not target_location:
            self.get_logger().error('Both object and location needed for manipulation')
            return None
        
        # Find object and target location in scene
        object_info = self.find_object_in_scene(target_object)
        if not object_info:
            self.get_logger().warn(f'Object "{target_object}" not found in scene')
            return None
        
        target_pose = self.find_target_pose(target_location)
        if not target_pose:
            self.get_logger().warn(f'Target location "{target_location}" not found in scene')
            return None
        
        plan = []
        
        # 1. Navigate to object
        approach_pose = self.calculate_approach_pose(object_info['pose'])
        plan.append({
            'action': 'navigation_execution',
            'target_pose': approach_pose,
            'parameters': {
                'speed': 'normal',
                'obstacle_avoidance': 'active'
            },
            'description': f'Approach {target_object}'
        })
        
        # 2. Grasp object
        grasp_pose = self.calculate_grasp_pose(object_info)
        grasp_type = self.select_grasp_type(object_info)
        
        plan.append({
            'action': 'execute_grasp',
            'grasp_pose': grasp_pose,
            'grasp_type': grasp_type,
            'description': f'Grasp {target_object}'
        })
        
        # 3. Navigate to destination
        plan.append({
            'action': 'navigation_execution',
            'target_pose': target_pose,
            'parameters': {
                'speed': 'careful',
                'obstacle_avoidance': 'active'
            },
            'description': f'Move {target_object} to {target_location}'
        })
        
        # 4. Place object
        placement_pose = self.calculate_placement_pose(target_pose)
        plan.append({
            'action': 'execute_place',
            'placement_pose': placement_pose,
            'description': f'Place {target_object} at {target_location}'
        })
        
        return plan
    
    def plan_interaction(self, intent_data):
        """Plan human interaction action sequence"""
        entities = intent_data.get('entities', {})
        target_person = entities.get('target')
        
        plan = []
        
        # 1. Locate target person
        person_pose = self.find_target_pose(target_person or 'closest_person')
        if person_pose:
            # 2. Navigate toward person (but maintain respectful distance)
            approach_pose = self.calculate_interaction_pose(person_pose)
            plan.append({
                'action': 'navigation_execution',
                'target_pose': approach_pose,
                'parameters': {
                    'speed': 'normal',
                    'obstacle_avoidance': 'active'
                },
                'description': f'Approach {target_person or "person"}'
            })
        
        # 3. Execute interaction
        command_type = intent_data.get('command_type', 'acknowledge')
        if command_type == 'greet' or 'wave' in intent_data.get('raw_text', '').lower():
            plan.append({
                'action': 'perform_gesture',
                'gesture_type': 'wave',
                'description': f'Wave to {target_person or "person"}'
            })
        else:
            plan.append({
                'action': 'perform_acknowledgement',
                'description': f'Acknowledge interaction request'
            })
        
        return plan
    
    def plan_generic(self, intent_data):
        """Plan for unrecognized command type"""
        self.get_logger().warn(f'Unknown command type: {intent_data.get("command_type")}')
        return None
    
    def find_target_pose(self, location_name):
        """Find location pose in scene database"""
        # This would query a scene database/map in practice
        # For example purposes, we'll use hardcoded locations
        locations = {
            'kitchen': Pose(x=3.0, y=1.0, z=0.0),
            'living room': Pose(x=0.0, y=0.0, z=0.0),
            'bedroom': Pose(x=-2.0, y=1.5, z=0.0),
            'office': Pose(x=1.0, y=-2.0, z=0.0),
            'dining': Pose(x=2.5, y=-1.0, z=0.0)
        }
        
        if location_name in locations:
            pose = locations[location_name]
            pose_stamped = PoseStamped()
            pose_stamped.pose = pose
            pose_stamped.header.frame_id = "map"
            return pose_stamped
        else:
            # Try fuzzy matching or closest approximation
            for loc, pose in locations.items():
                if location_name.lower() in loc.lower() or \
                   loc.lower() in location_name.lower():
                    pose_stamped = PoseStamped()
                    pose_stamped.pose = pose
                    pose_stamped.header.frame_id = "map"
                    return pose_stamped
        
        return None
    
    def find_object_in_scene(self, object_name):
        """Find object information in current scene"""
        # In a real system, this would interface with the perception system
        # For this example, we'll simulate object detection
        if not self.current_scene:
            # Default scene with objects
            self.current_scene = {
                'objects': [
                    {'name': 'coffee_mug', 'pose': Pose(x=1.0, y=1.0, z=0.8), 'type': 'container', 'graspable': True},
                    {'name': 'book', 'pose': Pose(x=0.8, y=1.0, z=0.8), 'type': 'flat', 'graspable': True},
                    {'name': 'ball', 'pose': Pose(x=1.2, y=0.8, z=0.8), 'type': 'spherical', 'graspable': True},
                    {'name': 'table', 'pose': Pose(x=1.0, y=1.0, z=0.5), 'type': 'support', 'graspable': False}
                ]
            }
        
        for obj in self.current_scene.get('objects', []):
            if object_name.lower() in obj['name'].lower():
                return obj
        
        return None
    
    def is_object_graspable(self, object_info):
        """Check if object can be grasped by the robot"""
        if not object_info.get('graspable', True):
            return False
        
        # Additional checks could include:
        # - Weight limits
        # - Size constraints
        # - Shape compatibility
        # - Safety considerations
        
        return True
    
    def calculate_approach_pose(self, object_pose):
        """Calculate approach pose for object interaction"""
        # Calculate position 30cm in front of the object
        approach_pose = PoseStamped()
        approach_pose.header.frame_id = "map"
        
        # For simplicity, approach from the front (positive x direction)
        approach_pose.pose = object_pose
        approach_pose.pose.position.x -= 0.3  # 30cm away
        
        return approach_pose
    
    def calculate_grasp_pose(self, object_info):
        """Calculate optimal grasp pose for object"""
        # This would use inverse kinematics in practice
        grasp_pose = PoseStamped()
        grasp_pose.pose = object_info['pose']
        grasp_pose.header.frame_id = "map"
        
        # Add approach offsets based on object type
        obj_type = object_info.get('type', 'generic')
        if obj_type == 'container':
            # Approach from above or side depending on handle position
            grasp_pose.pose.position.z += 0.05  # Just above the object
        elif obj_type == 'spherical':
            # Approach from side for spherical objects
            grasp_pose.pose.position.y += 0.05
        elif obj_type == 'flat':
            # Approach from above for flat objects
            grasp_pose.pose.position.z += 0.05
        
        return grasp_pose
    
    def select_grasp_type(self, object_info):
        """Select appropriate grasp type based on object properties"""
        obj_type = object_info.get('type', 'generic')
        
        if obj_type == 'container':
            return 'precision_pinch'  # Grasp handle
        elif obj_type == 'spherical':
            return 'power_sphere'  # Wrap fingers around sphere
        elif obj_type == 'flat':
            return 'large_diameter'  # Grasp from underneath
        else:
            return 'cylindrical'  # Default grasp
    
    def calculate_placement_pose(self, target_pose):
        """Calculate safe placement pose at target location"""
        placement_pose = PoseStamped()
        placement_pose.pose = target_pose.pose
        placement_pose.header.frame_id = target_pose.header.frame_id
        
        # Add slight height offset to place on top of surface
        placement_pose.pose.position.z += 0.1  # 10cm above surface
        
        return placement_pose
    
    def calculate_interaction_pose(self, person_pose):
        """Calculate respectful interaction distance from person"""
        interaction_pose = PoseStamped()
        interaction_pose.header.frame_id = "map"
        
        # Maintain respectful distance (1.2m) from person
        # For simplicity, this just moves slightly closer to the person
        interaction_pose.pose.position.x = person_pose.position.x - 1.2
        interaction_pose.pose.position.y = person_pose.position.y
        interaction_pose.pose.position.z = person_pose.position.z
        
        return interaction_pose
    
    def validate_action_plan(self, plan):
        """Validate action plan against constraints"""
        if not plan:
            return None
        
        # Check each action in the plan
        for action in plan:
            if not self.validate_action(action):
                self.get_logger().warn(f'Action validation failed: {action.get("action")}')
                return None
        
        # Check overall plan feasibility
        if not self.validate_plan_feasibility(plan):
            self.get_logger().warn('Plan feasibility validation failed')
            return None
        
        return plan
    
    def validate_action(self, action):
        """Validate single action"""
        action_type = action.get('action')
        
        if action_type == 'navigation_execution':
            target_pose = action.get('target_pose')
            if not target_pose:
                return False
            
            # Check if destination is reachable
            if not self.is_reachable(target_pose):
                return False
        
        elif action_type == 'execute_grasp':
            grasp_pose = action.get('grasp_pose')
            if not grasp_pose:
                return False
            
            # Check if grasp is physically possible
            if not self.is_grasp_possible(grasp_pose):
                return False
        
        return True
    
    def is_reachable(self, pose):
        """Check if robot can reach the specified pose"""
        # In a real implementation, this would check:
        # - Path existence
        # - Obstacle-free path
        # - Navigation constraints
        
        # For this example, assume all positions within reasonable bounds are reachable
        if abs(pose.pose.position.x) > 10.0 or abs(pose.pose.position.y) > 10.0:
            return False  # Too far away
        
        return True
    
    def is_grasp_possible(self, grasp_pose):
        """Check if grasp pose is physically possible"""
        # In a real implementation, this would check:
        # - Arm reachability
        # - Collision constraints
        # - Grasp stability
        
        # For this example, assume grasp is possible
        return True
    
    def validate_plan_feasibility(self, plan):
        """Validate overall plan feasibility"""
        # Check that plan can be executed given robot's limitations
        # and environmental constraints
        
        # For this example, just ensure plan is not empty and not too long
        if not plan or len(plan) > 50:  # Arbitrary limit
            return False
        
        return True
    
    def execute_action_plan(self, plan):
        """Execute the validated action plan"""
        for action in plan:
            success = self.execute_single_action(action)
            if not success:
                self.get_logger().warn(f'Action failed: {action.get("action")}')
                # Implement recovery behavior here
                break
    
    def execute_single_action(self, action):
        """Execute a single action in the plan"""
        action_type = action.get('action')
        
        if action_type == 'navigation_execution':
            return self.execute_navigation(action)
        elif action_type == 'execute_grasp':
            return self.execute_grasp(action)
        elif action_type == 'execute_place':
            return self.execute_place(action)
        elif action_type == 'perform_gesture':
            return self.execute_gesture(action)
        elif action_type == 'localize':
            return self.execute_localize(action)
        else:
            self.get_logger().warn(f'Unknown action type: {action_type}')
            return False
    
    def execute_navigation(self, action):
        """Execute navigation action"""
        target_pose = action.get('target_pose')
        params = action.get('parameters', {})
        
        if not target_pose:
            return False
        
        # Publish navigation goal
        self.nav_goal_pub.publish(target_pose)
        self.get_logger().info(f'Navigation goal published: {target_pose}')
        
        # In a real implementation, this would wait for execution completion
        # and handle recovery from failures
        
        return True
    
    def execute_grasp(self, action):
        """Execute grasp action"""
        grasp_pose = action.get('grasp_pose')
        grasp_type = action.get('grasp_type')
        
        if not grasp_pose or not grasp_type:
            return False
        
        # Create manipulation command
        cmd_msg = String()
        cmd_dict = {
            'command': 'grasp',
            'pose': grasp_pose,
            'type': grasp_type
        }
        cmd_msg.data = json.dumps(cmd_dict)
        
        self.manipulation_cmd_pub.publish(cmd_msg)
        self.get_logger().info(f'Grasp command published: {grasp_type} grasp at {grasp_pose}')
        
        # In a real implementation, this would wait for completion
        return True
    
    def execute_place(self, action):
        """Execute place action"""
        placement_pose = action.get('placement_pose')
        
        if not placement_pose:
            return False
        
        # Create manipulation command for placing
        cmd_msg = String()
        cmd_dict = {
            'command': 'place',
            'pose': placement_pose
        }
        cmd_msg.data = json.dumps(cmd_dict)
        
        self.manipulation_cmd_pub.publish(cmd_msg)
        self.get_logger().info(f'Place command published: {placement_pose}')
        
        # In a real implementation, this would wait for completion
        return True
    
    def execute_gesture(self, action):
        """Execute gesture action"""
        gesture_type = action.get('gesture_type')
        
        if not gesture_type:
            return False
        
        cmd_msg = String()
        cmd_dict = {
            'command': 'gesture',
            'type': gesture_type
        }
        cmd_msg.data = json.dumps(cmd_dict)
        
        self.manipulation_cmd_pub.publish(cmd_msg)
        self.get_logger().info(f'Gesture command published: {gesture_type}')
        
        return True
    
    def execute_localize(self, action):
        """Execute localization action"""
        # This would trigger localization procedures
        # For this example, assume successful localization
        self.get_logger().info('Localization completed')
        return True
    
    def scene_desc_callback(self, msg):
        """Update internal scene representation"""
        try:
            scene_data = json.loads(msg.data)
            self.current_scene = scene_data
            self.get_logger().info(f'Scene updated with {len(scene_data.get("objects", []))} objects')
        except Exception as e:
            self.get_logger().error(f'Scene description parsing error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = CognitivePlanningNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Mapping Natural Language to ROS 2 Actions

### ROS 2 Action Architecture

The VLA system maps natural language commands to ROS 2 action executions through a hierarchical system:

- **High-level Actions**: Abstract commands like "fetch object" or "navigate to location"
- **Mid-level Actions**: Specific robot behaviors like "move_to_pose" or "grasp_object"
- **Low-level Actions**: Joint-level movements and specific commands

### Action Mapping Implementation

```python
from geometry_msgs.msg import Pose, PoseStamped
from action_msgs.msg import GoalStatus
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker
import time

class ROSActionMappingNode(Node):
    def __init__(self):
        super().__init__('ros_action_mapping_node')
        
        # Action clients for various robot capabilities
        from rclpy.action import ActionClient
        from geometry_msgs.action import NavigateToPose
        from control_msgs.action import FollowJointTrajectory
        from manipulation_msgs.action import Grasp
        
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.traj_client = ActionClient(self, FollowJointTrajectory, '/joint_trajectory_controller/follow_joint_trajectory') 
        self.grasp_client = ActionClient(self, Grasp, '/grasp_controller/grasp')
        
        # Subscriptions
        self.action_seq_sub = self.create_subscription(
            String,
            '/action_sequence',
            self.action_sequence_callback,
            10
        )
        
        # Publishers
        self.marker_pub = self.create_publisher(Marker, '/action_markers', 10)
        
        # TF for coordinate transformations
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.get_logger().info('ROS Action Mapping Node initialized')
    
    def action_sequence_callback(self, msg):
        """Process action sequence and execute ROS 2 actions"""
        try:
            action_sequence = json.loads(msg.data)
            self.get_logger().info(f'Executing action sequence with {len(action_sequence)} steps')
            
            for i, action in enumerate(action_sequence):
                success = self.execute_action(action)
                if not success:
                    self.get_logger().error(f'Action {i+1} failed: {action.get("action")}')
                    # Implement recovery strategy here
                    break
                else:
                    self.get_logger().info(f'Action {i+1} completed: {action.get("action")}')
        
        except Exception as e:
            self.get_logger().error(f'Action sequence execution error: {e}')
    
    def execute_action(self, action):
        """Execute single action as ROS 2 action or service call"""
        action_type = action.get('action', '')
        
        if action_type == 'navigation_execution':
            return self.execute_navigation_action(action)
        elif action_type == 'execute_grasp':
            return self.execute_grasp_action(action)
        elif action_type == 'execute_place':
            return self.execute_place_action(action)
        elif action_type == 'perform_gesture':
            return self.execute_gesture_action(action)
        elif action_type == 'localize':
            return self.execute_localize_action(action)
        else:
            self.get_logger().warn(f'Unknown action type: {action_type}')
            return False
    
    async def execute_navigation_action(self, action):
        """Execute navigation using the Nav2 action server"""
        target_pose = action.get('target_pose')
        params = action.get('parameters', {})
        
        if not target_pose:
            self.get_logger().error('Navigation action missing target pose')
            return False
        
        # Wait for navigation server
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Navigation server not available')
            return False
        
        # Create navigation goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = target_pose
        
        # Set navigation parameters
        if 'speed' in params:
            goal_msg.speed = params['speed']
        
        # Send navigation goal
        goal_handle = await self.nav_client.send_goal_async(goal_msg)
        
        if not goal_handle.accepted:
            self.get_logger().error('Navigation goal rejected')
            return False
        
        self.get_logger().info(f'Navigation goal sent to: {target_pose.pose.position}')
        
        # Wait for result
        result = await goal_handle.get_result_async()
        status = result.status
        
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Navigation succeeded')
            return True
        else:
            self.get_logger().warn(f'Navigation failed with status: {status}')
            return False
    
    async def execute_grasp_action(self, action):
        """Execute grasp using manipulation action server"""
        grasp_pose = action.get('grasp_pose')
        grasp_type = action.get('grasp_type')
        
        if not grasp_pose or not grasp_type:
            self.get_logger().error('Grasp action missing required parameters')
            return False
        
        # Wait for grasp server
        if not self.grasp_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Grasp server not available')
            return False
        
        # Create grasp goal
        goal_msg = Grasp.Goal()
        goal_msg.target_pose = grasp_pose
        goal_msg.grasp_type = grasp_type
        goal_msg.pre_grasp_approach.desired_distance = 0.1
        goal_msg.post_grasp_retreat.min_distance = 0.1
        
        # Send grasp goal
        goal_handle = await self.grasp_client.send_goal_async(goal_msg)
        
        if not goal_handle.accepted:
            self.get_logger().error('Grasp goal rejected')
            return False
        
        self.get_logger().info(f'Grasp goal sent for {grasp_type} grasp')
        
        # Wait for result
        result = await goal_handle.get_result_async()
        status = result.status
        
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Grasp succeeded')
            return True
        else:
            self.get_logger().warn(f'Grasp failed with status: {status}')
            return False
    
    def execute_place_action(self, action):
        """Execute place action (simplified implementation)"""
        placement_pose = action.get('placement_pose')
        
        if not placement_pose:
            self.get_logger().error('Place action missing placement pose')
            return False
        
        # This would typically involve opening grippers and moving to placement pose
        self.get_logger().info(f'Place action executed at: {placement_pose.pose.position}')
        
        # In a real implementation, this would send specific ROS 2 commands
        return True
    
    def execute_gesture_action(self, action):
        """Execute gesture using joint trajectory control"""
        gesture_type = action.get('gesture_type')
        
        if not gesture_type:
            self.get_logger().error('Gesture action missing gesture type')
            return False
        
        # Define gesture trajectories based on type
        trajectories = self.define_gesture_trajectories(gesture_type)
        
        if not trajectories:
            self.get_logger().error(f'Unknown gesture: {gesture_type}')
            return False
        
        # Execute gesture trajectory
        success = self.execute_trajectory(trajectories)
        
        if success:
            self.get_logger().info(f'Gesture {gesture_type} completed')
        else:
            self.get_logger().warn(f'Gesture {gesture_type} failed')
        
        return success
    
    def define_gesture_trajectories(self, gesture_type):
        """Define joint trajectories for different gestures"""
        if gesture_type == 'wave':
            # Simplified wave gesture (in practice, this would be more complex)
            return [
                {'joint_names': ['right_shoulder', 'right_elbow'], 
                 'positions': [0.5, 0.0], 'time_from_start': 1.0},
                {'joint_names': ['right_shoulder', 'right_elbow'], 
                 'positions': [0.0, 0.5], 'time_from_start': 2.0},
                {'joint_names': ['right_shoulder', 'right_elbow'], 
                 'positions': [0.5, 0.0], 'time_from_start': 3.0}
            ]
        elif gesture_type == 'point':
            # Point gesture
            return [
                {'joint_names': ['right_shoulder', 'right_elbow', 'right_wrist'], 
                 'positions': [0.8, 0.5, 0.2], 'time_from_start': 1.0}
            ]
        else:
            return []
    
    def execute_trajectory(self, trajectory):
        """Execute joint trajectory using trajectory controller"""
        # Wait for trajectory server
        if not self.traj_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Trajectory server not available')
            return False
        
        # Create trajectory goal
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = self.create_trajectory_msg(trajectory)
        
        # Send trajectory goal
        future = self.traj_client.send_goal_async(goal_msg)
        
        # In practice, we'd wait for completion here
        self.get_logger().info('Trajectory goal sent')
        
        return True
    
    def create_trajectory_msg(self, waypoints):
        """Convert waypoints to ROS 2 trajectory message"""
        # This would create a proper trajectory message in practice
        # For now, this is a simplified representation
        return waypoints  # Placeholder

def main(args=None):
    rclpy.init(args=args)
    node = ROSActionMappingNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Full Capstone Implementation: Voice → Plan → Navigate → Identify → Manipulate

### The Complete VLA Pipeline

For the capstone project, we'll integrate all components into a complete voice-controlled humanoid robot system:

```python
from rclpy.qos import QoSProfile
from std_msgs.msg import String
from geometry_msgs.msg import Pose
import asyncio

class VLAGlobalPipelineNode(Node):
    def __init__(self):
        super().__init__('vla_global_pipeline_node')
        
        # Initialize all pipeline components
        self.voice_capture = VoiceCaptureNode()
        self.whisper_transcription = WhisperTranscriptionNode()
        self.intent_extraction = IntentExtractionNode()
        self.cognitive_planning = CognitivePlanningNode()
        self.ros_action_mapping = ROSActionMappingNode()
        
        # Create a timer to update the pipeline state
        self.pipeline_timer = self.create_timer(0.1, self.pipeline_status_callback)
        
        self.pipeline_state = {
            'voice_active': False,
            'transcribing': False,
            'intent_extracted': False,
            'planning': False,
            'executing': False,
            'last_command': '',
            'execution_status': 'idle'
        }
        
        self.get_logger().info('Complete VLA Pipeline initialized')
    
    def pipeline_status_callback(self):
        """Update pipeline status"""
        # In a real implementation, this would monitor the status
        # of all pipeline components
        pass
    
    def start_pipeline(self):
        """Start the complete VLA pipeline"""
        self.get_logger().info('Starting complete VLA pipeline...')
        
        # All components are already initialized by this point
        # The system will process commands through the pipeline automatically
        
    def stop_pipeline(self):
        """Stop the VLA pipeline"""
        self.get_logger().info('Stopping VLA pipeline...')
        # Implementation for stopping all components would go here

def main(args=None):
    rclpy.init(args=args)
    node = VLAGlobalPipelineNode()
    
    try:
        # Start the complete pipeline
        node.start_pipeline()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop the pipeline on shutdown
        node.stop_pipeline()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## System Validation and Testing

### Comprehensive Validation Framework

```python
class VLAValidationNode(Node):
    def __init__(self):
        super().__init__('vla_validation_node')
        
        # Subscriptions for monitoring the entire pipeline
        self.voice_monitor_sub = self.create_subscription(
            String, '/voice_feedback', self.voice_monitor_callback, 10
        )
        
        self.intent_monitor_sub = self.create_subscription(
            String, '/extracted_intent', self.intent_monitor_callback, 10
        )
        
        self.planning_monitor_sub = self.create_subscription(
            String, '/action_sequence', self.planning_monitor_callback, 10
        )
        
        self.execution_monitor_sub = self.create_subscription(
            String, '/execution_status', self.execution_monitor_callback, 10
        )
        
        # Publishers for validation reports
        self.validation_report_pub = self.create_publisher(
            String, '/validation_report', 10
        )
        
        # Timer for periodic validation checks
        self.validation_timer = self.create_timer(1.0, self.run_validation_cycle)
        
        # Validation metrics
        self.metrics = {
            'commands_processed': 0,
            'transcription_success_rate': 0.0,
            'intent_extraction_accuracy': 0.0,
            'planning_success_rate': 0.0,
            'execution_success_rate': 0.0,
            'avg_response_time': 0.0
        }
        
    def voice_monitor_callback(self, msg):
        """Monitor voice processing performance"""
        # Update voice processing metrics
        pass
    
    def intent_monitor_callback(self, msg):
        """Monitor intent extraction accuracy"""
        # Update intent extraction metrics
        pass
    
    def planning_monitor_callback(self, msg):
        """Monitor planning success rate"""
        # Update planning metrics
        pass
    
    def execution_monitor_callback(self, msg):
        """Monitor execution performance"""
        # Update execution metrics
        pass
    
    def run_validation_cycle(self):
        """Run comprehensive validation of the VLA system"""
        # Perform various validation checks
        
        # 1. Check pipeline integrity
        pipeline_integrity_score = self.check_pipeline_integrity()
        
        # 2. Check response times
        response_times_score = self.check_response_times()
        
        # 3. Check accuracy metrics
        accuracy_score = self.check_accuracy_metrics()
        
        # 4. Check safety constraints
        safety_score = self.check_safety_constraints()
        
        # Generate comprehensive validation report
        validation_report = {
            'timestamp': time.time(),
            'pipeline_integrity_score': pipeline_integrity_score,
            'response_times_score': response_times_score,
            'accuracy_score': accuracy_score,
            'safety_score': safety_score,
            'overall_system_score': (pipeline_integrity_score + response_times_score + 
                                   accuracy_score + safety_score) / 4,
            'recommendations': self.generate_recommendations()
        }
        
        # Publish validation report
        report_msg = String()
        report_msg.data = json.dumps(validation_report, indent=2)
        self.validation_report_pub.publish(report_msg)
        
        self.get_logger().info(f'Validation report generated: Overall score = {validation_report["overall_system_score"]:.2f}')
    
    def check_pipeline_integrity(self):
        """Check that all pipeline components are functioning"""
        # This would check if all nodes are active and communicating
        # For this example, return a placeholder score
        return 0.95  # 95% integrity
    
    def check_response_times(self):
        """Check that system responses are within acceptable time limits"""
        # This would measure actual response times
        return 0.85  # 85% of responses within limits
    
    def check_accuracy_metrics(self):
        """Check accuracy of transcription, intent extraction, and planning"""
        # This would use actual metrics from the system
        return 0.90  # 90% accuracy average
    
    def check_safety_constraints(self):
        """Check that all safety constraints are being enforced"""
        # This would verify safety systems are active
        return 1.0  # 100% safety compliance
    
    def generate_recommendations(self):
        """Generate recommendations for system improvement"""
        recommendations = []
        
        if self.metrics['transcription_success_rate'] < 0.9:
            recommendations.append('Improve acoustic environment or microphone array')
        
        if self.metrics['intent_extraction_accuracy'] < 0.85:
            recommendations.append('Retrain NLP model with more domain-specific data')
        
        if self.metrics['execution_success_rate'] < 0.9:
            recommendations.append('Implement better error recovery mechanisms')
        
        if not recommendations:
            recommendations.append('System is performing well overall')
        
        return recommendations

def main(args=None):
    rclpy.init(args=args)
    node = VLAValidationNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Hands-on Exercise 3.1: Complete VLA Pipeline Integration

1. Integrate all the components you've created:
   - Voice capture and transcription
   - Intent extraction
   - Cognitive planning
   - ROS 2 action mapping
   
2. Implement the complete capstone pipeline that connects voice input to physical robot actions

3. Test with various commands covering different robot capabilities:
   - Navigation commands
   - Object fetching
   - Object manipulation
   - Human interaction

4. Verify that the full pipeline works correctly: voice → plan → navigate → identify → manipulate

## Hands-on Exercise 3.2: Performance Optimization and Validation

1. Implement the validation framework to monitor system performance
2. Measure transcription accuracy, intent extraction accuracy, and execution success
3. Identify bottlenecks in the system and optimize performance
4. Test the system with natural language variations and ambiguous commands
5. Validate that all safety mechanisms are functioning correctly

## Validation Checklist
- [ ] I can implement cognitive planning for natural language to action mapping
- [ ] I have successfully mapped natural language commands to ROS 2 action sequences
- [ ] I have integrated navigation, obstacle avoidance, and object manipulation
- [ ] I have executed the full capstone pipeline: voice → plan → navigate → identify → manipulate
- [ ] I have validated the complete VLA system performance
- [ ] I understand the integration challenges in the full VLA pipeline
- [ ] I have tested the system with various types of commands
- [ ] I have implemented proper validation and safety checks

## Summary

This chapter covered the complete cognitive planning system for the Vision-Language-Action pipeline, including the detailed mapping of natural language commands to ROS 2 actions, and the integration of all components in a capstone implementation. We explored how to implement a comprehensive system that handles the complete pipeline from voice command to robot execution.

The VLA system represents a sophisticated integration of multiple AI and robotics technologies, enabling humanoid robots to understand and execute complex, natural language commands. When properly implemented, these systems can dramatically increase the accessibility and utility of humanoid robots, making them more intuitive for everyday users to interact with.

The validation and testing components ensure that the system operates safely and reliably, which is crucial for real-world deployment of humanoid robots. The modular architecture allows for continued improvement and adaptation to new tasks and environments.

This completes the Vision-Language-Action module, providing you with a comprehensive understanding of how to implement an end-to-end system for natural human-robot interaction using VLA principles.