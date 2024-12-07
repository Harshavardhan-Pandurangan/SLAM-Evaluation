# RTAB-Map Installation and Usage in Docker

This guide provides step-by-step instructions for installing and using RTAB-Map (Real-Time Appearance-Based Mapping) in a Docker container. It also includes dataset preparation, SLAM testing, and troubleshooting common issues.

## Prerequisites

1. **Operating System**: A Linux-based OS such as Pop!_OS 22.04 or Ubuntu 22.04.
2. **Docker Installed**: Ensure Docker is installed and running on your system. For installation instructions, refer to the [Docker documentation](https://docs.docker.com/get-docker/).
3. **Basic ROS Knowledge**: Familiarity with ROS1 concepts like nodes, topics, and launch files.

---

## Step 1: Setup Docker Environment

### 1.1 Pull a ROS Docker Image

Pull a compatible ROS Docker image with the desired ROS distribution (e.g., `noetic`):

```bash
sudo docker pull ros:noetic
```

### 1.2 Run the Docker Container

Launch the Docker container with port forwarding and shared volumes:

```bash
sudo docker run -it --rm \
    --name rtabmap_container \
    --net=host \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    ros:noetic
```

### 1.3 Configure Display Access

Run this command on the host machine to enable GUI applications inside Docker:

```bash
xhost +local:
```

---

## Step 2: Install RTAB-Map in Docker

### 2.1 Update and Install Dependencies

Inside the Docker container, update package lists and install required dependencies:

```bash
apt update && apt install -y \
    sudo vim git wget python3-pip \
    ros-noetic-rtabmap-ros
```

### 2.2 Verify Installation

To confirm RTAB-Map is installed, check available ROS packages:

```bash
rospack find rtabmap_ros
```

---

## Step 3: Prepare a Dataset

### 3.1 Download a Dataset

For testing, use a dataset such as the TUM RGB-D dataset. Download it from [TUM RGB-D Dataset](https://vision.in.tum.de/data/datasets/rgbd-dataset).

### 3.2 Place Dataset in Docker

Copy the dataset to a shared location accessible from Docker:

```bash
sudo cp -r /path/to/dataset /path/to/shared_volume
```

Mount the shared volume when running the container:

```bash
-v /path/to/shared_volume:/datasets
```

### 3.3 Convert Dataset (if needed)

If the dataset lacks `.bag` files, convert it using a script (e.g., `tum_to_ros.py`). Ensure `rospy` and `rosbag` are installed inside Docker:

```bash
pip install rospy rosbag
```

Run the conversion script:

```bash
python3 tum_to_ros.py
```

---

## Step 4: Run RTAB-Map

### 4.1 Launch RTAB-Map

Start RTAB-Map with a suitable launch file:

```bash
roslaunch rtabmap_ros rgbd_mapping.launch
```

### 4.2 Test Talker and Listener

Verify communication between ROS nodes using example talker/listener scripts:

```bash
rosrun rospy_tutorials talker
rosrun rospy_tutorials listener
```

### 4.3 Play Dataset with `rosbag`

Play the dataset (in `.bag` format):

```bash
rosbag play /datasets/my_dataset.bag
```

---

## Step 5: Visualize Results in RViz

Launch RViz to visualize SLAM results:

```bash
rosrun rviz rviz
```

Ensure the necessary plugins are installed:

```bash
sudo apt install ros-noetic-octomap-rviz-plugins
```

---

## Troubleshooting

### Error: `Did not receive data since 5 seconds!`
- **Cause**: Input topics are not published or timestamps are not synchronized.
- **Solution**: Verify topic publication with `rostopic hz` and synchronize clocks with `ntpdate`.

### Error: `Plugin for class 'octomap_rviz_plugin/ColorOccupancyGrid' failed to load`
- **Cause**: Missing RViz plugins.
- **Solution**:
  ```bash
  sudo apt install ros-noetic-octomap-rviz-plugins
  ```

---

## Additional Notes

1. **Syncing Clocks**: If running ROS across multiple machines, synchronize clocks using:
   ```bash
   sudo ntpdate ntp.ubuntu.com
   ```

2. **Dataset Format**: Ensure datasets are in a format compatible with RTAB-Map (e.g., `.bag` files).

3. **Modify Parameters**: Adjust parameters in the launch files as needed for your application.

---

By following these steps, you should have RTAB-Map running inside a Docker container and be ready to perform SLAM tasks. For more advanced configurations or troubleshooting, refer to the [RTAB-Map Documentation](http://introlab.github.io/rtabmap/).

# **RTAB-Map Launch Files Guide**

This README provides an overview of ROS launch files, their purpose, and how to use them with RTAB-Map. It also includes detailed explanations of the provided launch files, instructions on how to run them, and additional resources for further learning.

---

## **📘 What are Launch Files?**

A **launch file** in ROS is an XML file used to automate the launch of multiple ROS nodes and set parameters in a single command. It saves time and simplifies the process of running complex robotics systems. Instead of running multiple `rosrun` commands, you can use a single `roslaunch` command to launch an entire system.

### **Why Use Launch Files?**
- **Automation**: Launch multiple nodes at once.
- **Configuration**: Set parameters and remap topics without editing code.
- **Reusability**: Use the same launch file across different projects.

---

## **🛠️ Resources to Learn ROS Launch Files**

If you're new to launch files or ROS, these resources can help you understand and build your own launch files:

- [ROS Launch Tips for Larger Projects](https://wiki.ros.org/ROS/Tutorials/Roslaunch%20tips%20for%20larger%20projects)
- [Using rqt_console and roslaunch](https://wiki.ros.org/ROS/Tutorials/UsingRqtconsoleRoslaunch)
- [RTAB-Map Setup on Your Robot](https://wiki.ros.org/rtabmap_ros/Tutorials/SetupOnYourRobot)

---

## **📂 Provided Launch Files**

Here’s an explanation of the three launch files included in this project. Each file serves a different purpose, depending on the type of dataset you're working with.

---

### **1️⃣ launch1.launch**

**Purpose**:
This launch file is designed to work with **RGB-D data and Odometry**. It expects the following topics to be available:
- **RGB Image**: From an RGB-D camera.
- **Depth Image**: Depth data from the RGB-D camera.
- **Odometry**: Position and orientation data for SLAM.

**Intended Dataset**:
- **RGB-D dataset** with odometry (e.g., datasets captured with Intel RealSense, Kinect, or similar sensors that also provide odometry).

**Key Parameters and Topics:**
- **/camera/rgb/image_rect_color**: The RGB image topic.
- **/camera/depth_registered/image_raw**: The depth image topic.
- **/rtabmap/odom**: Odometry topic for robot position and orientation.

**How to Run**:
```bash
roslaunch rtabmap_ros launch1.launch
```

**Example Use Case**:
If you have a dataset with an RGB-D camera (like a RealSense or Kinect) and odometry data (e.g., from a wheel encoder or an IMU), this launch file will align the RGB-D images with the odometry to create a 3D map.

---

### **2️⃣ launch3.launch**

**Purpose**:
This launch file is intended for **stereo vision datasets**. It uses two synchronized cameras (left and right) to compute depth information using stereo vision instead of relying on a depth sensor.

**Intended Dataset**:
- **Stereo Camera Dataset** (two synchronized cameras recording left and right images).

**Key Parameters and Topics:**
- **/stereo/left/image_raw**: Left camera image.
- **/stereo/right/image_raw**: Right camera image.
- **/stereo/left/camera_info**: Camera calibration info for the left image.
- **/stereo/right/camera_info**: Camera calibration info for the right image.

**How to Run**:
```bash
roslaunch rtabmap_ros launch3.launch
```

**Example Use Case**:
If you have a stereo camera dataset (like those captured with ZED or stereo webcam setups), this launch file will process the left and right camera streams to create a 3D map. This approach doesn’t require a dedicated depth sensor, as the depth is computed from the stereo pair.

---

### **3️⃣ new_launch.launch**

**Purpose**:
This launch file is likely a hybrid approach that supports **RGB-D datasets without odometry**. It relies on the RGB-D sensor but doesn’t require odometry input.

**Intended Dataset**:
- **RGB-D dataset** (no odometry required). This is ideal for datasets where odometry is not available, and RTAB-Map can estimate motion through visual odometry.

**Key Parameters and Topics:**
- **/camera/rgb/image_rect_color**: The RGB image topic.
- **/camera/depth_registered/image_raw**: The depth image topic.

**How to Run**:
```bash
roslaunch rtabmap_ros new_launch.launch
```

**Example Use Case**:
If you have an RGB-D dataset (like those captured with an RGB-D camera but without odometry), this launch file will use RTAB-Map’s built-in visual odometry to estimate camera motion and create a map.

---

## **🚀 How to Run the Launch Files**

1. **Navigate to your ROS workspace**:
   ```bash
   cd ~/catkin_ws
   ```

2. **Source the ROS environment**:
   ```bash
   source devel/setup.bash
   ```

3. **Run one of the launch files**:
   ```bash
   roslaunch rtabmap_ros launch1.launch
   ```

   Replace `launch1.launch` with `launch3.launch` or `new_launch.launch`, depending on the type of dataset you want to run.

4. **Play a dataset (if using a bag file)**:
   ```bash
   rosbag play your_dataset.bag --clock
   ```

---

## **🔍 How to Debug and Visualize the Node Connections**

When running a complex system like RTAB-Map, it’s helpful to visualize the system connections (which node subscribes to which topics). One useful tool for this is `rqt_graph`.

### **How to Install rqt_graph**
```bash
sudo apt-get install ros-$ROS_DISTRO-rqt-graph
```

Replace `$ROS_DISTRO` with your ROS distribution (like `noetic` or `melodic`).

### **How to Use rqt_graph**
1. Run rqt_graph in a new terminal:
   ```bash
   rqt_graph
   ```

2. It will display a graphical representation of the ROS system. You can see which topics are connected to which nodes.

3. Look for connections involving the following key topics:
   - **/camera/rgb/image_rect_color**
   - **/camera/depth_registered/image_raw**
   - **/rtabmap/odom**

   If these topics are not connected to RTAB-Map, it could mean the source node isn’t publishing the data.

---

## **🌐 Useful Resources**

If you want to dive deeper into ROS, launch files, and RTAB-Map, here are some recommended resources:

1. [RTAB-Map Setup on Your Robot](https://wiki.ros.org/rtabmap_ros/Tutorials/SetupOnYourRobot)
   This tutorial shows how to set up RTAB-Map to work on a real robot, including setting up launch files.

2. [ROS Launch Tips for Larger Projects](https://wiki.ros.org/ROS/Tutorials/Roslaunch%20tips%20for%20larger%20projects)
   Learn about launch files, reusable launch files, and best practices for large projects.

3. [Using rqt_console and roslaunch](https://wiki.ros.org/ROS/Tutorials/UsingRqtconsoleRoslaunch)
   A guide on how to use rqt_console and roslaunch to manage large systems.

---

## **💡 Summary**

- Launch files are essential for managing complex ROS projects.
- The provided launch files support different dataset types:
  - **launch1.launch**: RGB-D with odometry.
  - **launch3.launch**: Stereo camera datasets.
  - **new_launch.launch**: RGB-D datasets without odometry.
- Use `rqt_graph` to visualize system connections.
- Use `roslaunch` to run the provided launch files.

With these launch files, you can process various datasets (stereo, RGB-D, with or without odometry) using RTAB-Map. Modify and customize these files for your specific use case, and be sure to reference the resources provided to become more proficient with ROS and launch files.