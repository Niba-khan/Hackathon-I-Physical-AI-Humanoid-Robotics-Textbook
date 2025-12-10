# Chapter 2: Perception & Synthetic Data - Photorealistic Scenes, Sensor Simulation, Dataset Generation

## Objectives
- Understand perception systems in humanoid robotics using Isaac Sim
- Create photorealistic scenes with accurate physics in Isaac Sim
- Implement sensor simulation for realistic data generation
- Generate synthetic datasets for training perception models
- Validate perception outputs against ground truth

## Introduction to Perception in Humanoid Robotics

Perception is fundamental to humanoid robotics - it enables robots to understand and interact with their environment. For humanoid robots, perception systems must process complex visual and sensory information in real-time to support navigation, manipulation, object recognition, and human interaction.

Isaac Sim provides a comprehensive platform for developing and testing perception systems in photorealistic environments with accurate physics simulation. This enables the generation of high-quality synthetic data that can be used to train perception models before deployment on physical robots.

## Creating Photorealistic Scenes in Isaac Sim

### Scene Composition with USD

Isaac Sim uses Universal Scene Description (USD) as its core scene representation format. USD enables:

- **Scalable scene composition**: Build complex scenes from modular components
- **Collaborative workflows**: Multiple developers can work on different parts of a scene
- **Version control**: Track changes to scenes over time
- **Interoperability**: Import/export scenes to/from other tools

### Setting up a Photorealistic Environment

Let's create a photorealistic indoor environment for humanoid robot perception training:

```python
# Example: Creating a photorealistic room in Isaac Sim using Python API
import omni
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.prims import create_primitive
from omni.isaac.sensor import Camera
from pxr import Gf
import numpy as np

class PhotorealisticEnvironment:
    def __init__(self):
        self.world = World(stage_units_in_meters=1.0)
        self.setup_scene()
    
    def setup_scene(self):
        # Add a ground plane
        create_primitive(
            prim_path="/World/GroundPlane",
            primitive_type="Plane",
            position=np.array([0, 0, 0]),
            orientation=np.array([0, 0, 0, 1]),
            scale=np.array([10, 10, 1])
        )
        
        # Add walls
        create_primitive(
            prim_path="/World/Wall1",
            primitive_type="Cube",
            position=np.array([0, -5, 1.5]),
            orientation=np.array([0, 0, 0, 1]),
            scale=np.array([10, 0.1, 3])
        )
        
        # Add furniture
        create_primitive(
            prim_path="/World/Table",
            primitive_type="Cube",
            position=np.array([2, 0, 0.4]),
            orientation=np.array([0, 0, 0, 1]),
            scale=np.array([1.5, 0.8, 0.8])
        )
        
        # Add objects for perception training
        self.add_objects_for_perception()
        
        # Setup lighting
        self.setup_lighting()
    
    def add_objects_for_perception(self):
        """Add objects that will be used for perception training"""
        # Add various objects at different positions
        objects = [
            {"name": "cube", "type": "Cube", "pos": [1, 1, 0.2], "size": [0.2, 0.2, 0.2]},
            {"name": "sphere", "type": "Sphere", "pos": [-1, 0.5, 0.2], "size": [0.2, 0.2, 0.2]},
            {"name": "cylinder", "type": "Cylinder", "pos": [0, -1, 0.2], "size": [0.2, 0.2, 0.4]},
        ]
        
        for i, obj_data in enumerate(objects):
            create_primitive(
                prim_path=f"/World/Object{i}",
                primitive_type=obj_data["type"],
                position=np.array(obj_data["pos"]),
                orientation=np.array([0, 0, 0, 1]),
                scale=np.array(obj_data["size"])
            )
    
    def setup_lighting(self):
        """Setup realistic lighting for the scene"""
        # Add a dome light to simulate ambient lighting
        dome_light = create_primitive(
            prim_path="/World/DomeLight",
            primitive_type="DomeLight",
            position=np.array([0, 0, 0]),
            scale=np.array([1, 1, 1])
        )
        
        dome_light.GetAttribute("inputs:color").Set(Gf.Vec3f(0.8, 0.8, 0.8))
        dome_light.GetAttribute("inputs:intensity").Set(3000)
        
        # Add a key light (main light source)
        key_light = create_primitive(
            prim_path="/World/KeyLight",
            primitive_type="DistantLight",
            position=np.array([3, -3, 5]),
            orientation=np.array([0.3, 0.1, 0, 1])
        )
        
        key_light.GetAttribute("inputs:color").Set(Gf.Vec3f(1.0, 0.95, 0.8))
        key_light.GetAttribute("inputs:intensity").Set(4000)

# To run this in Isaac Sim:
# env = PhotorealisticEnvironment()
# env.world.reset()
```

### Material and Texturing for Realism

Creating photorealistic scenes requires attention to materials and textures:

1. **Material Definition**: Use Physically-Based Rendering (PBR) materials that accurately reflect light
2. **Texture Mapping**: Apply high-resolution textures to surfaces
3. **Lighting Setup**: Configure realistic lighting with proper intensities and colors

In Isaac Sim, materials are defined using the MaterialX standard, which ensures physically accurate rendering. For our humanoid robot's environment, we might create materials for:
- Metal surfaces (robot parts)
- Fabric materials (furniture)
- Wood textures (tables, floors)
- Plastic objects (household items)

## Sensor Simulation in Isaac Sim

### Camera Simulation

Cameras are critical for visual perception in humanoid robots. Isaac Sim provides realistic camera simulation:

```python
from omni.isaac.sensor import Camera
import numpy as np

def setup_camera(robot_prim_path, name="rgb_camera", position=[0.1, 0, 0.1], orientation=[0, 0, 0, 1]):
    """
    Setup RGB camera on the robot
    """
    camera = Camera(
        prim_path=f"{robot_prim_path}/{name}",
        name=name,
        position=position,
        frequency=30,  # Hz
        resolution=(640, 480)
    )
    
    # Configure camera properties
    camera.set_focal_length(24.0)
    camera.set_horizontal_aperture(20.955)
    camera.set_vertical_aperture(15.29)
    
    # Enable noise modeling
    camera.add_noise_model("RgbNoiseModel", intensity=0.1)
    
    return camera
```

### LiDAR Simulation

LiDAR sensors provide 3D spatial information crucial for navigation and mapping:

```python
def setup_lidar(robot_prim_path, name="lidar", position=[0.2, 0, 0.5]):
    """
    Setup LiDAR sensor on the robot
    """
    from omni.isaac.range_sensor import LidarRtx
    
    lidar = LidarRtx(
        prim_path=f"{robot_prim_path}/{name}",
        name=name,
        translation=position,
        orientation=[0, 0, 0, 1],
        config="Example_Rotary_M16",  # 16 beam LiDAR
        visible=True
    )
    
    # Configure LiDAR properties
    lidar.set_max_range(25.0)  # meters
    lidar.set_lowest_range(0.1)  # meters
    
    return lidar
```

### IMU Simulation

Inertial Measurement Units (IMUs) provide orientation and acceleration data:

```python
def setup_imu(robot_prim_path, name="imu", position=[0, 0, 0.5]):
    """
    Setup IMU sensor on the robot
    """
    from omni.isaac.core.sensors import ImuSensor
    
    imu = ImuSensor(
        prim_path=f"{robot_prim_path}/{name}",
        name=name,
        position=position
    )
    
    # Configure IMU properties
    imu._sensor.add_noise_imu()  # Add realistic noise
    
    return imu
```

## Synthetic Data Generation Workflow

### Domain Randomization

Domain randomization is a technique to make perception models more robust by varying environmental conditions:

```python
import random
from pxr import Gf

def apply_domain_randomization(world_stage):
    """Apply domain randomization to the scene"""
    
    # Randomize lighting
    dome_light = world_stage.GetPrimAtPath("/World/DomeLight")
    if dome_light.IsValid():
        # Random color temperature (between 4000K and 8000K)
        color_range = random.uniform(0.9, 1.1)
        dome_light.GetAttribute("inputs:color").Set(
            Gf.Vec3f(color_range, color_range * random.uniform(0.95, 1.05), random.uniform(0.9, 1.1))
        )
        
        # Random intensity
        intensity = random.uniform(2000, 5000)
        dome_light.GetAttribute("inputs:intensity").Set(intensity)
    
    # Randomize object positions
    for i in range(5):  # Randomize 5 objects
        obj_prim = world_stage.GetPrimAtPath(f"/World/Object{i}")
        if obj_prim.IsValid():
            # Add random position offset
            current_pos = obj_prim.GetAttribute("xformOp:translate").Get()
            new_pos = [
                current_pos[0] + random.uniform(-0.2, 0.2),
                current_pos[1] + random.uniform(-0.2, 0.2), 
                current_pos[2] + random.uniform(-0.1, 0.1)
            ]
            obj_prim.GetAttribute("xformOp:translate").Set(new_pos)
    
    # Randomize material properties
    # This would involve modifying material properties of objects
    # to vary appearance while maintaining physical properties
```

### Data Collection Pipeline

Creating a complete data collection pipeline:

```python
import cv2
import numpy as np
import omni
from omni.isaac.core import World
from omni.vision.annotation import AnnotationManager
import json
from datetime import datetime

class SyntheticDataCollector:
    def __init__(self, world, output_dir="synthetic_data"):
        self.world = world
        self.output_dir = output_dir
        self.annotation_manager = AnnotationManager()
        self.frame_count = 0
        
    def collect_frame_data(self, cameras, lidars, annotations=True):
        """Collect synchronized data from all sensors"""
        
        # Step the physics engine to get new sensor data
        self.world.step(render=True)
        
        frame_data = {
            "frame_id": self.frame_count,
            "timestamp": datetime.now().isoformat(),
            "sensors": {}
        }
        
        # Collect RGB camera data
        for cam_name, camera in cameras.items():
            # Get RGB image
            rgb_data = camera.get_rgb()
            if rgb_data is not None:
                # Save RGB image
                img_path = f"{self.output_dir}/{cam_name}_rgb_{self.frame_count:06d}.png"
                cv2.imwrite(img_path, cv2.cvtColor(rgb_data, cv2.COLOR_RGB2BGR))
                frame_data["sensors"][cam_name] = {"rgb_path": img_path}
        
        # Collect LiDAR data
        for lidar_name, lidar in lidars.items():
            # Get LiDAR point cloud
            point_cloud = lidar.get_linear_depth_data()
            if point_cloud is not None:
                # Save point cloud data
                pc_path = f"{self.output_dir}/{lidar_name}_points_{self.frame_count:06d}.npy"
                np.save(pc_path, point_cloud)
                frame_data["sensors"][f"{lidar_name}_lidar"] = {"pointcloud_path": pc_path}
        
        # Collect annotations if requested
        if annotations:
            # Get segmentation masks, bounding boxes, etc.
            annotations_data = self.collect_annotations()
            frame_data["annotations"] = annotations_data
        
        # Save frame metadata
        metadata_path = f"{self.output_dir}/frame_{self.frame_count:06d}_metadata.json"
        with open(metadata_path, 'w') as f:
            json.dump(frame_data, f, indent=2)
        
        self.frame_count += 1
        
        return frame_data
    
    def collect_annotations(self):
        """Collect ground truth annotations"""
        annotations = {}
        
        # Example: 2D bounding boxes for objects
        # This would use Isaac's annotation tools to get ground truth
        bbox_annotations = self.annotation_manager.get_2d_bounding_box_annotations()
        annotations["bounding_boxes"] = bbox_annotations
        
        # Example: Semantic segmentation
        seg_annotations = self.annotation_manager.get_semantic_segmentation_annotations()
        annotations["semantic_segmentation"] = seg_annotations
        
        return annotations
```

## Generating Training Datasets

### Classification Dataset

For object classification, generate a dataset with labeled images:

```python
import os
import shutil
from sklearn.model_selection import train_test_split

def generate_classification_dataset(synthetic_data_path, output_path, class_names):
    """
    Organize synthetic data into classification dataset format
    """
    # Create train/validation/test splits
    all_images = []
    all_labels = []
    
    # Collect all synthetic images and their labels
    for class_name in class_names:
        class_images = [f for f in os.listdir(synthetic_data_path) 
                       if class_name in f and f.endswith('.png')]
        
        for img in class_images:
            all_images.append(img)
            all_labels.append(class_name)
    
    # Split the data
    train_imgs, temp_imgs, train_labels, temp_labels = train_test_split(
        all_images, all_labels, test_size=0.3, random_state=42, stratify=all_labels
    )
    
    val_imgs, test_imgs, val_labels, test_labels = train_test_split(
        temp_imgs, temp_labels, test_size=0.5, random_state=42, stratify=temp_labels
    )
    
    # Create directory structure
    for split_name, split_data in [("train", (train_imgs, train_labels)), 
                                   ("val", (val_imgs, val_labels)), 
                                   ("test", (test_imgs, test_labels))]:
        split_path = os.path.join(output_path, split_name)
        
        for class_name in class_names:
            class_path = os.path.join(split_path, class_name)
            os.makedirs(class_path, exist_ok=True)
        
        # Move images to appropriate directories
        images, labels = split_data
        for img, label in zip(images, labels):
            src_path = os.path.join(synthetic_data_path, img)
            dst_path = os.path.join(split_path, label, img)
            shutil.copy2(src_path, dst_path)
```

### Detection Dataset

For object detection, format data in standard formats like COCO:

```python
def generate_detection_dataset(synthetic_data_path, output_path):
    """
    Generate detection dataset in COCO format
    """
    import json
    
    coco_format = {
        "info": {
            "year": 2024,
            "version": "1.0",
            "description": "Synthetic Humanoid Robot Perception Dataset",
            "contributor": "Physical AI & Humanoid Robotics Project",
            "url": "",
            "date_created": datetime.now().isoformat()
        },
        "licenses": [],
        "images": [],
        "annotations": [],
        "categories": []
    }
    
    # Add categories (object classes)
    class_names = ["cube", "sphere", "cylinder", "humanoid_robot", "table"]
    for i, name in enumerate(class_names):
        coco_format["categories"].append({
            "id": i,
            "name": name,
            "supercategory": "object"
        })
    
    # Process each synthetic image and its annotations
    metadata_files = [f for f in os.listdir(synthetic_data_path) 
                     if f.endswith('_metadata.json')]
    
    image_id = 0
    annotation_id = 0
    
    for meta_file in metadata_files:
        meta_path = os.path.join(synthetic_data_path, meta_file)
        with open(meta_path, 'r') as f:
            frame_data = json.load(f)
        
        # Add image entry
        img_path = next(iter(frame_data["sensors"].values()))["rgb_path"]
        img_filename = os.path.basename(img_path)
        
        coco_format["images"].append({
            "id": image_id,
            "license": 0,
            "file_name": img_filename,
            "height": 480,
            "width": 640,
            "date_captured": frame_data["timestamp"]
        })
        
        # Add annotations (bounding boxes)
        if "annotations" in frame_data and "bounding_boxes" in frame_data["annotations"]:
            for bbox in frame_data["annotations"]["bounding_boxes"]:
                # Assuming bbox format is [x, y, width, height]
                coco_format["annotations"].append({
                    "id": annotation_id,
                    "image_id": image_id,
                    "category_id": bbox["category_id"],
                    "bbox": bbox["bbox"],  # [x, y, width, height]
                    "area": bbox["bbox"][2] * bbox["bbox"][3],
                    "iscrowd": 0
                })
                annotation_id += 1
        
        image_id += 1
    
    # Save the COCO format dataset
    with open(os.path.join(output_path, "annotations.json"), 'w') as f:
        json.dump(coco_format, f, indent=2)
```

## Validation of Synthetic Data Quality

### Comparing with Real Data

To validate synthetic data quality, compare it with real data:

1. **Statistical Similarity**: Compare distributions of image features
2. **Model Performance**: Train models on synthetic and real data, compare performance
3. **Domain Gap Metrics**: Use metrics like Fréchet Inception Distance (FID)

### Quality Assessment Techniques

```python
def assess_synthetic_data_quality(synthetic_images, real_images):
    """
    Assess the quality of synthetic data vs real data
    """
    import torch
    import torchvision.transforms as transforms
    from torch_fidelity import calculate_metrics
    
    # Calculate FID (Fréchet Inception Distance)
    # Lower FID indicates better similarity between datasets
    metrics = calculate_metrics(
        synthetic_images, 
        real_images,
        cuda=True,
        isc=True,  # Inception Score
        fid=True,  # Fréchet Inception Distance
        verbose=False
    )
    
    print(f"Synthetic Data Quality Metrics:")
    print(f"  FID Score: {metrics['frechet_inception_distance']:.2f}")
    print(f"  Inception Score: {metrics['inception_score_mean']:.2f}")
    
    return metrics
```

## Hands-on Exercise 2.1: Create a Perception Scene

1. Create a new Isaac Sim scene with:
   - A humanoid robot model
   - Various household objects for perception
   - Proper lighting setup
   - At least one RGB camera and one LiDAR sensor

2. Implement domain randomization by varying:
   - Object positions and orientations
   - Lighting conditions
   - Background textures

3. Set up a basic data collection pipeline that captures:
   - RGB images from the camera
   - Point cloud data from the LiDAR
   - Basic annotations (object poses)

4. Generate at least 50 synthetic frames with annotations

## Hands-on Exercise 2.2: Generate a Perception Dataset

1. Organize your collected synthetic data into a proper dataset format
2. Create a classification dataset with at least 3 object classes
3. Generate COCO-style annotations for object detection
4. Validate the dataset format using a basic dataset loader
5. Create a visualization script to verify annotations are correct

## Validation Checklist
- [ ] I understand the importance of perception in humanoid robotics
- [ ] I can create photorealistic scenes in Isaac Sim with USD
- [ ] I know how to set up and configure different sensor types (camera, LiDAR, IMU)
- [ ] I can implement domain randomization techniques for synthetic data
- [ ] I have created a complete data collection pipeline
- [ ] I can format synthetic data into standard dataset formats (classification, detection)
- [ ] I have validated the quality of my synthetic data
- [ ] I have generated a usable perception dataset for training models

## Summary

This chapter covered the creation of perception systems for humanoid robots using Isaac Sim's photorealistic simulation capabilities. We explored scene composition with USD, sensor simulation, and synthetic data generation workflows. We also covered domain randomization techniques and dataset formatting for machine learning applications.

The combination of Isaac Sim's realistic rendering and physically-accurate simulation makes it ideal for generating training data that can bridge the sim-to-real gap for humanoid robot perception systems. The synthetic data generation pipeline we've explored enables rapid development and testing of perception models before deployment on physical robots.

In the next chapter, we'll explore navigation and path planning for humanoid robots using Isaac ROS and Nav2.