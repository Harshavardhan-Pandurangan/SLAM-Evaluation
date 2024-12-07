import cv2
import numpy as np
from tqdm import tqdm

def load_video_capture(video_path):
    """
    Initialize video capture object and return basic properties
    """
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        raise ValueError(f"Unable to open video file: {video_path}")

    fps = int(cap.get(cv2.CAP_PROP_FPS))
    frame_count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    return cap, fps, frame_count, width, height

def rectify_stereo_frames(left_frame, right_frame, K_left, K_right, D_left, D_right, R, T):
    """
    Rectify stereo frames using camera calibration parameters
    """
    # Get image size
    h, w = left_frame.shape[:2]

    # Calculate stereo rectification parameters
    R1, R2, P1, P2, Q, roi1, roi2 = cv2.stereoRectify(
        K_left, D_left, K_right, D_right, (w, h), R, T)

    # Calculate rectification maps
    map1_left, map2_left = cv2.initUndistortRectifyMap(
        K_left, D_left, R1, P1, (w, h), cv2.CV_32FC1)
    map1_right, map2_right = cv2.initUndistortRectifyMap(
        K_right, D_right, R2, P2, (w, h), cv2.CV_32FC1)

    # Apply rectification
    rect_left = cv2.remap(left_frame, map1_left, map2_left, cv2.INTER_LINEAR)
    rect_right = cv2.remap(right_frame, map1_right, map2_right, cv2.INTER_LINEAR)

    return rect_left, rect_right, Q

def compute_depth_map(left_frame, right_frame):
    """
    Compute depth map using Semi-Global Block Matching
    """
    # Convert frames to grayscale
    left_gray = cv2.cvtColor(left_frame, cv2.COLOR_BGR2GRAY)
    right_gray = cv2.cvtColor(right_frame, cv2.COLOR_BGR2GRAY)

    # Create StereoSGBM object
    stereo = cv2.StereoSGBM_create(
        minDisparity=0,
        numDisparities=128,
        blockSize=5,
        P1=8 * 3 * 5**2,
        P2=32 * 3 * 5**2,
        disp12MaxDiff=1,
        uniquenessRatio=10,
        speckleWindowSize=100,
        speckleRange=32
    )

    # Compute disparity map
    disparity = stereo.compute(left_gray, right_gray)

    # Normalize disparity map for visualization
    disparity_normalized = cv2.normalize(disparity, None, alpha=0, beta=255,
                                       norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)

    return disparity_normalized

def generate_depth_map_video(left_video_path, right_video_path, output_path,
                           K_left, K_right, D_left, D_right, R, T):
    """
    Generate depth map video from stereo video inputs
    """
    # Load video captures
    left_cap, fps, frame_count, width, height = load_video_capture(left_video_path)
    right_cap, _, _, _, _ = load_video_capture(right_video_path)

    # Create video writer
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter(output_path, fourcc, fps, (width, height), False)

    try:
        # Process frames
        for _ in tqdm(range(frame_count), desc="Processing frames"):
            # Read frames
            ret_left, left_frame = left_cap.read()
            ret_right, right_frame = right_cap.read()

            if not ret_left or not ret_right:
                break

            # Rectify frames
            rect_left, rect_right, Q = rectify_stereo_frames(
                left_frame, right_frame, K_left, K_right, D_left, D_right, R, T)

            # Compute depth map
            depth_map = compute_depth_map(rect_left, rect_right)

            # Write frame
            out.write(depth_map)

    finally:
        # Release resources
        left_cap.release()
        right_cap.release()
        out.release()

# Example usage
if __name__ == "__main__":
    # Camera calibration parameters (these should be replaced with your actual values)
    K_left = np.array([[fx, 0, cx],
                       [0, fy, cy],
                       [0, 0, 1]])
    K_right = np.array([[fx, 0, cx],
                        [0, fy, cy],
                        [0, 0, 1]])
    D_left = np.zeros((1, 5))  # Distortion coefficients
    D_right = np.zeros((1, 5))
    R = np.eye(3)  # Rotation matrix between cameras
    T = np.array([baseline, 0, 0])  # Translation vector between cameras

    # Generate depth map video
    generate_depth_map_video(
        left_video_path="left_video.mp4",
        right_video_path="right_video.mp4",
        output_path="depth_map.mp4",
        K_left=K_left,
        K_right=K_right,
        D_left=D_left,
        D_right=D_right,
        R=R,
        T=T
    )