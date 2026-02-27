# ORB-SLAM3 Node Implementation Guide

## 1. Overview
The `StereoSlamNode` is a ROS2 C++ wrapper for the ORB-SLAM3 library. It performs robust Visual SLAM using synchronized stereo images to estimate the camera trajectory and sparse 3D map. The node handles coordinate system transformations between ORB-SLAM3's camera optical frame and the standard ROS world frame (ENU).

## 2. Implementation Details

### 2.1 Node Structure
The node is implemented as a class `StereoSlamNode` inheriting from `rclcpp::Node` in `stereo_slam_node.cpp`.

#### Key Components:
- **ORB-SLAM3 System**: Wraps the core `ORB_SLAM3::System` class.
- **Stereo Synchronizer**: Uses `message_filters::Synchronizer` with `ApproximateTime` policy to pair left and right images.
- **TF Broadcaster**: Publishes the estimated transform from map to camera/base_link.
- **Coordinate Transformer**: Handles the complex rotation logic to align the visual odometry with the robot's world frame.

### 2.2 Coordinate Systems & Transformations
An essential part of this node is mapping between Computer Vision (CV) convention and ROS convention.

- **World/RViz Frame (ROS)**: X-forward, Y-left, Z-up (ENU)
- **Camera Optical Frame (ORB-SLAM3)**: Z-forward (viewing), X-right, Y-down
- **Rover Body Frame**: X-forward, Y-left, Z-up (aligned with base_link)

#### Transformation Logic (`InitializeCoordinateTransform`)
The node computes a rotation matrix `R_cam_to_world_` that transforms vectors from the Camera Optical frame to the Rover Body frame.
1.  **Using Quaternion (Preferred)**: If `use_camera_quat` is true, it uses the quaternion from `/tf` (base_link -> camera) combined with a fixed Optical-to-Body rotation.
2.  **Using Tilt Angle (Legacy)**: Calculates rotation based on a pitch angle (`camera_tilt_deg`) assuming the camera looks down from the horizontal.

### 2.3 Initialization Process
1.  **Parameter Loading**: configuring paths, topics, and initial pose parameters.
2.  **Initial Pose Setup**:
    - **From TF**: Waits for `/tf_gt` (Ground Truth) to set the initial starting point (Sim mode).
    - **From Params**: Uses `init_rover_x/y/z/qw...` defined in config.
    - **From GT File**: Loads trajectory file (EuroC format) for dataset evaluation.
3.  **ORB-SLAM3 System Start**: Initializes the SLAM system with the Vocabulary and Settings file.
4.  **Trajectory Saving**: On shutdown, saves `CameraTrajectory_ROS2.txt` (TUM format).

### 2.4 Data Flow (`GrabStereo`)
1.  **Input**: Receives synchronized `sensor_msgs::Image` pair.
2.  **Conversion**: Converts to `cv::Mat` (Gray/RGB).
3.  **Tracking**: Calls `mpSLAM_->TrackStereo(imLeft, imRight, timestamp)`.
4.  **Transformation (`TransformToWorldFrame`)**:
    - Gets raw pose `Twc` (Camera in World) from ORB-SLAM3.
    - Computes relative motion from the first frame.
    - Rotates the relative motion into the ROS World frame using `R_cam_to_world_`.
    - Adds this relative motion to the **Initial Pose** established at startup.
5.  **Publication**:
    - Publishes `/slam/pose` (PoseStamped)
    - Publishes `/slam/odom` (Odometry)
    - Broadcasts TF (map -> camera_link or base_link depending on config)

## 3. Key Parameters

### General
| Parameter | Default | Description |
| :--- | :--- | :--- |
| `vocabulary_path` | Required | Path to ORBvoc.txt |
| `settings_path` | Required | Path to stereo yaml settings |
| `enable_viewer` | true | Show Pangolin viewer window |
| `pose_mode` | 'camera' | 'camera' (optical frame) or 'world' (body frame) output |

### Topics
| Parameter | Default | Description |
| :--- | :--- | :--- |
| `left_topic` | `/stereo/left/image_raw` | Left image topic |
| `right_topic`| `/stereo/right/image_raw`| Right image topic |
| `output_frame_id`| `map` | Frame ID for published pose |
| `camera_frame_id`| `camera_link` | Child frame ID for TF |

### Initial Pose & Extrinsics
| Parameter | Description |
| :--- | :--- |
| `camera_tilt_deg` | Camera pitch down angle (degrees) |
| `use_tf_gt_init` | Use `/tf_gt` topic to auto-initialize pose (Sim) |
| `init_rover_x/y/z`| Manual initial position (Sim/Real) |
| `camera_offset_x/y/z`| Translation from base_link to camera link |
| `use_camera_quat` | Use TF quaternion for cam-to-body rotation instead of tilt |

## 4. Topics

| Type | Topic Name | Description |
| :--- | :--- | :--- |
| **Sub** | `/stereo/left/image_raw` | Synchronized Left Image |
| **Sub** | `/stereo/right/image_raw`| Synchronized Right Image |
| **Sub** | `/tf_gt` | Ground Truth TF (for sim initialization) |
| **Pub** | `/slam/pose` | Current Estimated Pose `geometry_msgs/PoseStamped` |
| **Pub** | `/slam/odom` | Odometry `nav_msgs/Odometry` |
| **Pub** | `/tf` | TF broadcast (map -> camera_link/base_link) |
