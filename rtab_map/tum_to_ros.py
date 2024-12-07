import os
import rosbag
import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2

def create_rosbag_from_tum(dataset_dir, output_bag):
    rgb_file = os.path.join(dataset_dir, 'rgb.txt')
    depth_file = os.path.join(dataset_dir, 'depth.txt')
    rgb_images = []
    depth_images = []
    
    with open(rgb_file, 'r') as f:
        for line in f.readlines():
            if line[0] == '#': continue
            rgb_images.append(line.strip().split())
    
    with open(depth_file, 'r') as f:
        for line in f.readlines():
            if line[0] == '#': continue
            depth_images.append(line.strip().split())
    
    bridge = CvBridge()
    with rosbag.Bag(output_bag, 'w') as bag:
        for (rgb_stamp, rgb_file), (depth_stamp, depth_file) in zip(rgb_images, depth_images):
            rgb_image = cv2.imread(os.path.join(dataset_dir, rgb_file))
            depth_image = cv2.imread(os.path.join(dataset_dir, depth_file), cv2.IMREAD_UNCHANGED)
            
            # Create RGB message
            rgb_msg = bridge.cv2_to_imgmsg(rgb_image, encoding="bgr8")
            rgb_msg.header.stamp = rospy.Time.from_sec(float(rgb_stamp))
            
            # Create Depth message
            depth_msg = bridge.cv2_to_imgmsg(depth_image, encoding="16UC1")
            depth_msg.header.stamp = rospy.Time.from_sec(float(depth_stamp))
            
            # Create CameraInfo message (if available)
            camera_info_msg = CameraInfo()
            camera_info_msg.header.stamp = rospy.Time.from_sec(float(rgb_stamp))
            # Set camera intrinsic parameters here (dummy values for now)
            camera_info_msg.width = rgb_image.shape[1]
            camera_info_msg.height = rgb_image.shape[0]
            camera_info_msg.K = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
            camera_info_msg.P = [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0]

            # Write to the bag with the remapped topics
            bag.write('/camera/rgb/image_rect_color', rgb_msg, t=rgb_msg.header.stamp)
            bag.write('/camera/depth_registered/image_raw', depth_msg, t=depth_msg.header.stamp)
            bag.write('/camera/rgb/camera_info', camera_info_msg, t=rgb_msg.header.stamp)

# Run this script
dataset_dir = "./rgbd_dataset_freiburg1_desk"
output_bag = "./freiburg1_desk.bag"
create_rosbag_from_tum(dataset_dir, output_bag)
