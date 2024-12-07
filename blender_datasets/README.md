# SLAM Dataset Generation Using Blender

This guide provides detailed instructions for generating synthetic datasets for SLAM evaluation using Blender. The pipeline includes scene creation, camera setup, animation configuration, and conversion to ROS bag format. Refer to the `rosbags` library for more information on ROS bag file format, and how to customize the data generation pipeline [here](https://pypi.org/project/rosbags/).

## Prerequisites

- **Blender**: Version 2.8 or higher
- **Python 3.8+**
- **Required Python packages**:
  ```bash
  pip install numpy rosbags matplotlib opencv-python transforms3d tqdm
  ```

## Overview

The dataset generation pipeline consists of the following steps:
1. Scene preparation in Blender
2. Camera and sensor configuration
3. Animation setup and recording
4. Data extraction and processing
5. ROS bag conversion

## Step-by-Step Guide

### 1. Scene Preparation

1. **Create or Import Scene**:
   - Launch Blender and create a new scene
   - For photogrammetry data: `File > Import > [Choose Format]`
   - Scale imported models to match real-world dimensions
   - Ensure proper lighting setup using Blender's Cycles renderer

2. **Scene Optimization**:
   ```python
   # Recommended settings in Blender Python console
   bpy.context.scene.render.engine = 'CYCLES'
   bpy.context.scene.cycles.device = 'GPU'
   bpy.context.scene.render.resolution_x = 1280
   bpy.context.scene.render.resolution_y = 720
   ```

### 2. Camera Configuration

1. **Add Camera**:
   - Add new camera: `Add > Camera`
   - Position at desired starting location
   - Set camera parameters:
     ```python
     camera = bpy.data.objects['Camera']
     camera.data.lens = 35  # Focal length in mm
     camera.data.sensor_width = 32  # Sensor width in mm
     ```

2. **Configure Camera Properties**:
   ```python
   # Set camera intrinsics
   camera.data.shift_x = 0  # Principal point X
   camera.data.shift_y = 0  # Principal point Y
   camera.data.clip_start = 0.1  # Near clip
   camera.data.clip_end = 100    # Far clip
   # these are blender metres
   ```

### 3. Animation Setup

1. **Create Camera Path**:
   - Switch to Curve Edit mode
   - Create path using Bezier curves
   - Attach camera to path:
     ```python
     # Follow path constraint
     constraint = camera.constraints.new('FOLLOW_PATH')
     constraint.target = bpy.data.objects['CameraPath']
     ```

2. **Configure Animation Settings**:
   ```python
   # Set animation length and frame rate
   bpy.context.scene.frame_start = 0
   bpy.context.scene.frame_end = 250
   bpy.context.scene.render.fps = 30
   ```

### 4. Data Extraction

1. **Extract Camera Parameters**:
   ```python
   def get_camera_parameters():
       camera = bpy.data.objects['Camera']
       K = get_calibration_matrix()
       RT = camera.matrix_world
       return K, RT
   ```

2. **Generate Depth Maps**:
    You can also use `mist` pass to get depth maps. But these two methods give direct depth maps and not depth maps that real world cameras would give.

   ```python
   # Setup Z-pass rendering
   bpy.context.scene.use_nodes = True
   tree = bpy.context.scene.node_tree
   ```

3. Alternatively, add two more cameras to the left and right of the main camera as its children. Then render the images from these cameras and use the disparity between the images to get the depth map. Use the python script `stereo_to_depth.py` to convert the stereo images to depth maps.

### 5. Converting to ROS Bag

1. **Extract Animation Data**:
    Run the `tf_extractor.py` script to extract the IMU data from the Blender scene. and store in `camera_transforms.json`.

2. **Create ROS Bag**:
    Use the `writer.ipynb` notebook to create a ROS bag from the extracted data.
    You can also analyze the data and visualize the data using the `analysis.ipynb` notebook.

## Data Format Specifications

The generated ROS bag will contain the following topics:
- `/camera/rgb/image_raw`: RGB images (sensor_msgs/Image)
- `/camera/depth/image_rect_raw`: Depth images (sensor_msgs/Image)
- `/camera/rgb/camera_info`: Camera calibration (sensor_msgs/CameraInfo)
- `/imu`: IMU measurements (sensor_msgs/Imu)
- `/tf`: Camera pose (geometry_msgs/TransformStamped)

## Best Practices

1. **Scene Setup**:
   - Use realistic textures and lighting
   - Maintain real-world scale
   - Include sufficient visual features for SLAM

2. **Camera Motion**:
   - Keep movements smooth and realistic
   - Avoid sudden acceleration
   - Include both rotational and translational motion

3. **Data Generation**:
   - Use high sample rates for IMU data (≥100Hz)
   - Ensure proper synchronization between sensors
   - Validate data before creating final ROS bag

## Troubleshooting

Common issues and solutions:
1. **Blender Python API errors**:
   - Ensure correct Blender version
   - Check Python path configurations
   - Verify script permissions

2. **Data Extraction Issues**:
   - Validate camera setup
   - Check animation keyframes
   - Verify output directories exist
