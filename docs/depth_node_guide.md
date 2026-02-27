# FoundationStereo Depth Node Implementation Guide

## 1. Overview
The `FoundationStereoDepthNode` is a ROS2 node designed to perform high-quality stereo depth estimation using the FoundationStereo model. It subscribes to synchronized left and right stereo image pairs, computes a dense depth map, and publishes the result along with camera information. It also supports ground truth (GT) evaluation and various visualization modes.

### 1.1 About FoundationStereo
**FoundationStereo** is a cutting-edge deep learning framework for dense stereo matching. Unlike traditional block-matching algorithms (e.g., SGM), it leverages large-scale pre-training to achieve robust zero-shot generalization across diverse environments.

*   **Architecture**: It utilizes a recurrent refinement architecture (similar to RAFT-Stereo), where the disparity map is iteratively improved through a series of GRU-based updates. This allows users to trade off inference speed for accuracy by adjusting the number of iterations (`valid_iters`).
*   **Backbone**: The node supports standard CNN backbones as well as Transformer-based backbones (e.g., **Hiera**) via the `use_hiera` flag. Hiera backbones are particularly effective for processing high-resolution images and capturing global context, making them suitable for complex outdoor scenes.
*   **Output**: The model outputs a dense disparity map. The node then converts this to metric depth and optionally filters it to remove noise or outliers (using confidence checks or bilateral filtering) before publishing to ROS.

## 2. Implementation Details

### 2.1 Node Structure
The node is implemented as a class `FoundationStereoDepthNode` inheriting from `rclpy.node.Node`.

#### Key Components:
- **Model Loader**: Loads the FoundationStereo model from a checkpoint.
- **Stereo Synchronizer**: Uses `ApproximateTimeSynchronizer` to ensure left and right images correspond to the same timestamp.
- **Inference Engine**: Runs the model on GPU using PyTorch/CUDA.
- **Visualization Threads**: dedicated threads for OpenCV/Topic-based visualization and Open3D point cloud visualization to avoid blocking the main ROS execution loop.

### 2.2 Initialization Process
1.  **Parameter Declaration**: Configures topics, model paths, intrinsics, and operational flags (e.g., `use_hiera`, `enable_bilateral_filter`).
2.  **Import Setup**: Dynamically adds the FoundationStereo library path to `sys.path` and imports necessary modules.
3.  **Model Loading**: Loads the model configuration and weights, moving the model to GPU memory.
4.  **Intrinsics Loading**: Reads the camera intrinsic matrix ($K$) and baseline from a text file.
5.  **ROS Interface**: Sets up subscribers with `QoSProfile` and initializes the `ApproximateTimeSynchronizer`.

### 2.3 Data Flow & Synchronization
- **Subscribers**:
    - Left Image: `/stereo/left/image_raw`
    - Right Image: `/stereo/right/image_raw`
    - GT Depth (Optional): For evaluation against ground truth.
- **Synchronization**:
    - Uses `message_filters.ApproximateTimeSynchronizer` with a strict `slop` (default 0.033s) to pair images.
    - **Timestamp Validation**: Checks the exact timestamp difference between left and right images. If the difference exceeds `max_timestamp_diff_ms`, the frame pair can be skipped to prevent depth errors during fast motion.
- **Buffering**: The synchronization callback stores the latest valid pair in a thread-safe variable (`_latest_stereo`). This decouples the high-frequency camera input (e.g., 30Hz) from the potentially slower inference process.

### 2.4 Inference Pipeline (`_do_inference`)
1.  **Preprocessing**:
    - Converts ROS messages to OpenCV format (handling mono/rgb encodings).
    - Resizes images based on the `scale` parameter.
    - Ensures 3-channel RGB input.
    - Pads images to be divisible by 32 (requirement for the network).
    - **GPU Transfer**: Images are converted to tensors and moved to CUDA memory.
2.  **Model Execution**:
    - Runs `FoundationStereo.forward` (or `run_hierachical` if enabled).
    - Uses `torch.cuda.amp.autocast` for mixed-precision inference (Float16), which significantly speeds up computation on Tensor Core-enabled GPUs.
3.  **Depth Recovery**:
    - Output is disparity. Depth is calculated as:
      $$ Depth = \frac{f_{pixel} \times baseline}{disparity} $$
    - $f_{pixel}$ is scaled according to the image resize factor.
4.  **Post-Processing**:
    - **Clipping**: Values exceeding `z_far` or invalid disparities are set to 0.
    - **Bilateral Filter**: If enabled, applies a bilateral filter to smooth noise while preserving edges (`cv2.bilateralFilter`). This is crucial for NVBlox integration to prevent "ghost" obstacles from sensor noise.
5.  **Publication**:
    - Publishes `/depth/image` (32FC1 encoding).
    - Publishes `/depth/camera_info` with updated intrinsics (accounting for resizing).
    - Publishes `/color/image` aligned with depth timestamp.

### 2.5 Visualization & Evaluation
- **Visualization Thread**: A separate thread handles generating visualization images (depth maps, error maps, side-by-side comparisons). This design ensures that slow `cv2.imshow` or `cv2.applyColorMap` operations do not block the inference pipeline.
    - Supports 'cv2' mode (OpenCV window) and 'topic' mode (publishes to `/depth/image/viz`).
- **Point Cloud Thread**: Independent Open3D thread that visualizes 3D point clouds of predicted vs. GT depth. It runs at ~30Hz, decoupled from the inference rate.
- **GT Evaluation**: If GT depth is provided, the node computes metrics:
    - Absolute Relative Error (`abs_rel`)
    - Root Mean Squared Error (`rmse`)
    - Threshold Accuracy (`a1`, `a2`, `a3`)

### 2.6 Performance Optimization & Bottleneck Resolution
To achieve real-time performance with the heavy FoundationStereo model (which can take 100-300ms per frame), several architectural optimizations were implemented to minimize system latency and bottlenecks.

#### A. Decoupled Processing Loop (Frame Skipping Strategy)
Instead of processing frames strictly sequentially inside a callback (which creates a growing queue delay), the node employs a **Producer-Consumer** pattern with frame skipping:
1.  **Producer (ROS Callback)**: Updates the `_latest_stereo` variable with the newest synchronized pair and returns immediately.
2.  **Consumer (Timer)**: A 100Hz timer checks if the inference engine is idle.
    *   If **Idle**: It takes the current `_latest_stereo` and starts processing.
    *   If **Busy**: It skips the cycle.
    *   **Result**: This implicitly drops intermediate frames that arrive while the GPU is busy. This guarantees that **latency is minimized**—the depth map used for navigation always represents the most recent state of the world, rather than a state from seconds ago (throughput is sacrificed for latency).

#### B. Threaded Visualization
Visualization tasks are offloaded to separate daemon threads (`_viz_thread`, `_pc_viz_thread`).
-   The main threading loop only copies the result data (NumPy arrays) to shared variables inside a lock.
-   The visualization threads wake up independently to render images or point clouds.
-   This improves inference FPS by removing GUI overhead from the critical path.

#### C. Timestamp Management (`timestamp_mode`)
When inference latency is high, using the original camera timestamp for the output depth map can cause issues in ROS TF (Transform) lookups, as the data might be considered "too old" by the time it reaches the consumer (e.g., nvblox/Nav2).
-   **`timestamp_mode: 'current'`**: Re-stamps the outgoing depth/color messages with `get_clock().now()` at the moment of publication. This ensures downstream nodes can successfully look up the latest transforms, improving system robustness in simulation or high-latency scenarios.

## 3. Key Parameters

| Parameter | Default | Description |
| :--- | :--- | :--- |
| `model_path` | (auto) | Path to the model checkpoint (.pth) |
| `intrinsic_file` | required | Path to file containing K matrix and baseline |
| `scale` | 0.5 | Image downscaling factor (0.5 = half resolution) |
| `valid_iters` | 32 | Number of refinement iterations for the model |
| `use_hiera` | False | Enable hierarchical inference for high-res images |
| `z_far` | 20.0 | Maximum valid depth in meters |
| `enable_bilateral_filter`| True | Enable edge-preserving smoothing filter |
| `timestamp_mode` | 'current'| 'current' re-stamps outputs with now(), 'original' keeps source time |

## 4. Topics

| Type | Topic Name | Description |
| :--- | :--- | :--- |
| **Sub** | `/stereo/left/image_raw` | Left rgb/mono image |
| **Sub** | `/stereo/right/image_raw`| Right rgb/mono image |
| **Pub** | `/depth/image` | Computed depth map (meters, 32FC1) |
| **Pub** | `/depth/camera_info` | Camera intrinsics for the depth map |
| **Pub** | `/color/image` | Left color image synced with depth |
