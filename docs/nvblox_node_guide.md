# Nvblox Node Implementation Guide

## 1. Overview
The `NvbloxNode` is a GPU-accelerated volumetric mapping node part of the `isaac_ros_nvblox` package. It builds a dense 3D map of the environment in real-time by integrating depth images and/or LiDAR scans. It maintains a **Truncated Signed Distance Function (TSDF)** for reconstruction and an **Euclidean Signed Distance Function (ESDF)** for path planning. The node is designed to handle dynamic environments by separating static and dynamic objects (using `MultiMapper`).

## 2. Implementation Details

### 2.1 Node Structure
The node is implemented in C++ as `NvbloxNode` inheriting from `rclcpp::Node`. It heavily leverages NVIDIA's **NITROS** (NVIDIA Isaac Transport for ROS) for zero-copy image transfer, ensuring high performance on Jetson/GPU platforms.

#### Key Components:
- **MultiMapper**: The core mapping engine. It manages two internal mappers:
    - **Static Mapper**: Stores the static environment.
    - **Dynamic Mapper**: Stores moving objects (if `mapping_type` includes dynamics).
- **Transformer**: Helper class to handle TF transformations (Global Frame $\leftrightarrow$ Sensor Frame).
- **LayerPublisher**: Converts internal voxel layers (TSDF/ESDF/Mesh) into ROS messages (PointClouds, Occupancy Grids).
- **CudaStream**: Manages CUDA execution streams for asynchronous GPU processing.
- **Queue System**: Thread-safe queues (`depth_image_queue_`, `color_image_queue_`, `pointcloud_queue_`) decouple sensor callback execution from the main processing loop (`tick()`).

### 2.2 Data Flow & Synchronization
1.  **Input Subscription**:
    - **Depth/Color**: Uses `nvidia::isaac_ros::nitros::Subscriber` for optimized image transport.
    - **Sync**: Uses `message_filters` to synchronize:
        - Depth + CameraInfo
        - (Optional) Depth + Segmentation Mask + CameraInfos (for human/dynamic masking)
2.  **Queueing**: Callbacks push data into mutex-protected queues. This prevents the ROS executor from blocking during heavy mapping operations.
3.  **Processing Loop (`tick`)**:
    - Triggered by a wall timer (`queue_processing_timer_`).
    - Consumes data from queues.
    - **Integrate**: Projects depth/LiDAR into the 3D voxel grid.
    - **Decay**: Reduces weight of old observations to handle scene changes.
    - **Update ESDF**: Computes the distance map from the TSDF/Occupancy layer.
    - **Meshing**: Generates a 3D mesh from the TSDF for visualization.

### 2.3 Mapping Pipeline
1.  **Integration**:
    - Depth images are back-projected to 3D points.
    - Ray-casting updates voxels in the TSDF layer.
    - If `use_segmentation` is on, dynamic pixels (e.g., people) are integrated into the *Dynamic Mapper* or cleared from the *Static Mapper*.
2.  **ESDF Generation**:
    - ESDF is computed incrementally from the TSDF (or Occupancy) layer.
    - This provides the "distance to nearest obstacle" required for navigation (Nav2).
3.  **Output Generation**:
    - **Slices**: 2D cross-sections of the ESDF are published as `DistanceMapSlice` or `OccupancyGrid` for 2D costmaps.
    - **PointClouds**: Voxel centers are published as point clouds for 3D visualization.

## 3. Key Parameters

### General Configuration
| Parameter | Default | Description |
| :--- | :--- | :--- |
| `voxel_size` | 0.05 | Resolution of the voxel grid (meters) |
| `global_frame` | `map` | Fixed world frame ID |
| `pose_frame` | `odom` | Robot odometry frame ID |
| `mapping_type` | `static_tsdf` | Mode: `static_tsdf`, `static_occupancy`, `human_with_static_tsdf`, etc. |
| `esdf_mode` | `3d` | `2d` (faster) or `3d` (full volumetric) ESDF calculation |

### Input Configuration
| Parameter | Default | Description |
| :--- | :--- | :--- |
| `use_depth` | true | Subscribe to depth images |
| `use_lidar` | false | Subscribe to LiDAR point clouds |
| `use_color` | false | Subscribe to color images (for colored mesh) |
| `use_segmentation` | false | Use segmentation masks to filter dynamics |
| `num_cameras` | 1 | Number of cameras to subscribe to |

### Output Configuration
| Parameter | Default | Description |
| :--- | :--- | :--- |
| `tick_period_ms` | 10 | Period of the main processing loop |
| `layer_visualization_min_tsdf_weight` | 1e-4 | Min weight to visualize a voxel |
| `output_pessimistic_distance_map` | false | Output specific map for robust planning |

## 4. Topics

### Subscribed Topics (Sim Mode Config)
| Topic Name | Type | Description |
| :--- | :--- | :--- |
| `/depth/image` | `NitrosImage` | Depth image input (Remapped from `camera_0/depth/image`) |
| `/depth/camera_info` | `CameraInfo` | Depth camera intrinsics |
| `/color/image` | `NitrosImage` | Color image input (Remapped from `camera_0/color/image`) |
| `/color/camera_info` | `CameraInfo` | Color camera intrinsics |
| `/slam/pose` | `PoseStamped` | Camera pose from ORB-SLAM3 (Used if TF is not available) |
| `/tf` | `TFMessage` | Transform tree from SLAM/Robot |

**Note**: In the simulation launch (`nvblox.launch.py`):
- `use_depth`: True
- `use_color`: True
- `use_lidar`: False
- `use_tf_transforms`: True (Pose is primarily derived from TF broadcast by SLAM)

### Published Topics
| Topic Name | Type | Description |
| :--- | :--- | :--- |
| `~/static_occupancy_grid` | `OccupancyGrid` | 2D static traversability map |
| `~/combined_occupancy_grid` | `OccupancyGrid` | 2D combined (static+dynamic) map |
| `~/static_esdf_pointcloud` | `PointCloud2` | 3D ESDF visualization (Distance Field) |
| `~/mesh` | `Mesh` | Reconstructed 3D mesh |
| `~/map_slice` | `DistanceMapSlice` | Raw 2D slice of the distance field |
| `~/static_map_slice` | `DistanceMapSlice` | Static map 2D slice |

### Services
- `~/save_map`: Save current map to disk.
- `~/load_map`: Load a map from disk.
- `~/save_ply`: Export mesh as PLY file.
