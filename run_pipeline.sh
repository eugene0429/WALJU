#!/bin/bash
# =============================================================================
# Full SLAM + Depth + nvblox Pipeline Runner
# =============================================================================
#
# Usage:
#   ./run_pipeline.sh [OPTIONS]
#
# Examples:
#   ./run_pipeline.sh --all                    # Run all components
#   ./run_pipeline.sh --slam --depth           # Run only SLAM and depth
#   ./run_pipeline.sh --nvblox --rviz          # Run nvblox and rviz
#   ./run_pipeline.sh --all --no-slam-viz      # All components, SLAM viewer disabled
#   ./run_pipeline.sh --dataset --slam --slam-viz  # Dataset + SLAM with viewer
#
# =============================================================================

set -e

# =============================================================================
# Default Configuration
# =============================================================================
ROS2_WS="${HOME}/WALJU"

DATASET_PATH="${ROS2_WS}/data/LuSNAR/Moon_2"
LEFT_SUBDIR="image0/color"
RIGHT_SUBDIR="image1/color"
PUBLISH_RATE="1.0"
VOCABULARY_PATH="${ROS2_WS}/deps/ORB_SLAM3/Vocabulary/ORBvoc.txt"
VOXEL_SIZE="0.05"
# ESDF slice settings (relative to ground level, will be converted to absolute in sim mode)
# These are RELATIVE heights above ground - auto-calculated to absolute in sim mode
ESDF_SLICE_HEIGHT_RELATIVE="0.05"   # Slice visualization height above ground
ESDF_SLICE_MIN_HEIGHT_RELATIVE="0.10" # Min obstacle height above ground
ESDF_SLICE_MAX_HEIGHT_RELATIVE="3.0"  # Max obstacle height above ground
# Absolute ESDF values (calculated in configure_topics for sim mode)
ESDF_SLICE_HEIGHT=""
ESDF_SLICE_MIN_HEIGHT=""
ESDF_SLICE_MAX_HEIGHT=""
CAMERA_FRAME="camera_link"
GLOBAL_FRAME="map"

# Nav2 Navigation Settings
FOOTPRINT_SCALE="1.5"             # Footprint scale factor (1.0 = original, 1.5 = 50% larger)
XY_GOAL_TOLERANCE="0.5"         # Goal position tolerance (meters)
YAW_GOAL_TOLERANCE="3.0"        # Goal orientation tolerance (radians)
GROUND_HEIGHT="-1.0"            # Ground height (-1 = auto-detect from /tf_gt)

# Rover Velocity Settings (must match Isaac Sim rover configuration)
MAX_LINEAR_VEL="0.2"             # Max linear velocity (m/s)
MAX_ANGULAR_VEL="0.8"            # Max angular velocity (rad/s)
LINEAR_ACCEL="2.0"               # Linear acceleration (m/s^2)
ANGULAR_ACCEL="2.0"              # Angular acceleration (rad/s^2)

# Initial Rotation Controller Settings
INIT_ROTATION_MAX_ANGULAR_VEL="0.3"  # Max angular vel for initial rotation (rad/s)
INIT_ROTATION_MIN_ANGULAR_VEL="0.2"  # Min angular vel for initial rotation (rad/s)

# Camera intrinsic files
DATASET_INTRINSIC_FILE="${ROS2_WS}/data/LuSNAR/K.txt"
SIM_INTRINSIC_FILE="${ROS2_WS}/assets/sim_intrinsics.txt"
# Active intrinsic file (set in configure_topics())
INTRINSIC_FILE=""

# SLAM settings files
DATASET_SLAM_SETTINGS="${ROS2_WS}/deps/ORB_SLAM3/Examples/Stereo/Custom.yaml"
SIM_SLAM_SETTINGS="${ROS2_WS}/assets/sim_slam_settings.yaml"
# Active SLAM settings file (set in configure_topics())
SLAM_SETTINGS=""

# =============================================================================
# Initial Pose Configuration (World Frame: X-forward, Y-left, Z-up)
# =============================================================================
# Dataset mode defaults
DATASET_INIT_ROVER_X="0.0"
DATASET_INIT_ROVER_Y="0.0"
DATASET_INIT_ROVER_Z="0.0"
DATASET_INIT_ROVER_QW="1.0"
DATASET_INIT_ROVER_QX="0.0"
DATASET_INIT_ROVER_QY="0.0"
DATASET_INIT_ROVER_QZ="0.0"
DATASET_CAMERA_OFFSET_X="0.0"
DATASET_CAMERA_OFFSET_Y="0.0"
DATASET_CAMERA_OFFSET_Z="1.5"
DATASET_CAMERA_TILT_DEG="19.923"
# Dataset ESDF slice settings (ground at Z~0)
DATASET_ESDF_SLICE_HEIGHT="0.10"
DATASET_ESDF_SLICE_MIN_HEIGHT="0.15"
DATASET_ESDF_SLICE_MAX_HEIGHT="3.0"

# Sim mode defaults (Isaac Sim rover configuration)
SIM_INIT_ROVER_X="0.0"
SIM_INIT_ROVER_Y="0.0"
SIM_INIT_ROVER_Z="0.15"
SIM_INIT_ROVER_QW="1.0"
SIM_INIT_ROVER_QX="0.0"
SIM_INIT_ROVER_QY="0.0"
SIM_INIT_ROVER_QZ="0.0"
SIM_CAMERA_OFFSET_X="0.48"
SIM_CAMERA_OFFSET_Y="0.2"
SIM_CAMERA_OFFSET_Z="1.14"
SIM_CAMERA_TILT_DEG="20.0"
# Isaac Sim camera frame name (will be relayed to camera_link)
SIM_CAMERA_FRAME_NAME="Camera_OmniVision_OV9782_Left"
# Base link to ground offset (used to calculate ESDF slice height from /tf_gt)
SIM_BASE_LINK_GROUND_OFFSET="0.13"
# Sim ESDF slice settings (dynamically set from /tf_gt in sim mode)
SIM_ESDF_SLICE_HEIGHT=""  # Will be calculated: tf_gt_z - BASE_LINK_GROUND_OFFSET
SIM_ESDF_SLICE_MIN_HEIGHT="0.15"
SIM_ESDF_SLICE_MAX_HEIGHT="3.0"

# Active pose configuration (set in configure_topics())
INIT_ROVER_X=""
INIT_ROVER_Y=""
INIT_ROVER_Z=""
INIT_ROVER_QW=""
INIT_ROVER_QX=""
INIT_ROVER_QY=""
INIT_ROVER_QZ=""
CAMERA_OFFSET_X=""
CAMERA_OFFSET_Y=""
CAMERA_OFFSET_Z=""
CAMERA_TILT_DEG=""
SIM_CAMERA_FRAME=""  # Isaac Sim camera frame name
# Use parameter init pose (true) or read from GT file (false)
USE_PARAM_INIT_POSE="true"
# Use /tf_gt topic for init pose in sim mode
USE_TF_GT_INIT="false"
TF_GT_TOPIC="/tf_gt"
TF_GT_CHILD_FRAME="base_link"
TF_GT_TIMEOUT="10.0"
# Base link to ground offset for ESDF slice height calculation
BASE_LINK_GROUND_OFFSET="0.15"

# Component flags
RUN_STATIC_TF=false
RUN_DATASET=false
RUN_SLAM=false
RUN_DEPTH=false
RUN_NVBLOX=false
RUN_RVIZ=false
RUN_NAV2=false
RUN_GOAL_SENDER=false
RUN_GROUND_TRUTH=false
RUN_POSE_PLOT=false
RUN_METRICS=false
RUN_ALL=false
DISABLE_GOAL_STOP=false  # Set true to disable goal_reached_stop node (for debugging)

# Verbose logging mode
VERBOSE=false  # Set true for detailed logging output

# Data source mode: 'dataset' = use dataset_publisher, 'sim' = subscribe to Isaac Sim
DATA_SOURCE="dataset"

# Topic configuration (will be set based on DATA_SOURCE)
# Dataset publisher topics (default)
DATASET_LEFT_TOPIC="/stereo/left/image_raw"
DATASET_RIGHT_TOPIC="/stereo/right/image_raw"
# Isaac Sim topics
# Image type: raw, noisy, enhanced, adjusted (for sim mode)
SIM_IMAGE_TYPE="raw"
SIM_LEFT_TOPIC_RAW="/stereo/left/rgb"
SIM_RIGHT_TOPIC_RAW="/stereo/right/rgb"
SIM_LEFT_TOPIC_NOISY="/stereo/left/rgb_noisy"
SIM_RIGHT_TOPIC_NOISY="/stereo/right/rgb_noisy"
SIM_LEFT_TOPIC_ENHANCED="/stereo/left/enhanced"
SIM_RIGHT_TOPIC_ENHANCED="/stereo/right/enhanced"
SIM_LEFT_TOPIC_DENOISED="/stereo/left/rgb_denoised"
SIM_RIGHT_TOPIC_DENOISED="/stereo/right/rgb_denoised"
SIM_LEFT_TOPIC_ADJUSTED="/stereo/left/adjusted"
SIM_RIGHT_TOPIC_ADJUSTED="/stereo/right/adjusted"
# Active sim topics (set based on SIM_IMAGE_TYPE)
SIM_LEFT_TOPIC="${SIM_LEFT_TOPIC_RAW}"
SIM_RIGHT_TOPIC="${SIM_RIGHT_TOPIC_RAW}"
# Active topics (set in configure_topics())
LEFT_TOPIC=""
RIGHT_TOPIC=""
USE_BEST_EFFORT="false"

# Visualization flags
SLAM_VIZ=true
DEPTH_VIZ=true
DEPTH_PC_VIZ=false  # Open3D point cloud overlay visualization

# Depth filtering options
DEPTH_BILATERAL_FILTER=true  # Enable bilateral filter for depth noise reduction
DEPTH_BILATERAL_D=5          # Bilateral filter diameter (5-9 typical)
DEPTH_BILATERAL_SIGMA_COLOR=50.0  # Bilateral filter color sigma
DEPTH_BILATERAL_SIGMA_SPACE=50.0  # Bilateral filter space sigma

# RViz config
RVIZ_CONFIG="${ROS2_WS}/install/nvblox_integration/share/nvblox_integration/config/custom_example.rviz"

# Conda settings
CONDA_BASE="${HOME}/miniconda3"
DEPTH_CONDA_ENV="foundation_stereo_py312"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# =============================================================================
# Functions
# =============================================================================

print_banner() {
    echo -e "${CYAN}"
    echo "╔═══════════════════════════════════════════════════════════════╗"
    echo "║         SLAM + Depth + nvblox 3D Mapping Pipeline            ║"
    echo "╚═══════════════════════════════════════════════════════════════╝"
    echo -e "${NC}"
}

print_help() {
    echo -e "${GREEN}Usage:${NC} $0 [OPTIONS]"
    echo ""
    echo -e "${YELLOW}Component Options:${NC}"
    echo "  --all              Run all components"
    echo "  --static-tf        Run static TF publisher (camera_link -> base_link)"
    echo "  --dataset          Run dataset publisher"
    echo "  --slam             Run ORB-SLAM3 node"
    echo "  --depth            Run FoundationStereo depth node"
    echo "  --nvblox           Run nvblox 3D mapping node"
    echo "  --rviz             Run RViz2 visualization"
    echo "  --nav2             Run Nav2 navigation stack (sim mode only)"
    echo "  --goal-sender      Run interactive goal sender for Nav2"
    echo "  --ground-truth     Publish ground truth point cloud from simulation"
    echo "  --pose-plot        Real-time pose comparison plot (GT vs SLAM)"
    echo "  --metrics          Navigation metrics logger (CSV: localization/depth errors, time, distance)"
    echo ""
    echo -e "${YELLOW}Data Source Options:${NC}"
    echo "  --sim              Use Isaac Sim as data source (subscribe to /stereo/*/rgb)"
    echo "  --use-dataset      Use dataset publisher as data source (default)"
    echo "  --image-type TYPE  Set image type for sim mode (default: raw)"
    echo "                       raw          - /stereo/*/rgb (uncompressed)"
    echo "                       noisy        - /stereo/*/rgb_noisy/compressed"
    echo "                       enhanced     - /stereo/*/enhanced/compressed"
    echo "                       denoised     - /stereo/*/rgb_denoised"
    echo "                       adjusted     - /stereo/*/adjusted"
    echo "  --left-topic TOPIC     Override left image topic"
    echo "  --right-topic TOPIC    Override right image topic"
    echo ""
    echo -e "${YELLOW}Visualization Options:${NC}"
    echo "  --slam-viz         Enable SLAM viewer (default: enabled)"
    echo "  --no-slam-viz      Disable SLAM viewer"
    echo "  --depth-viz        Enable depth visualization window (default: enabled)"
    echo "  --no-depth-viz     Disable depth visualization window"
    echo "  --pc-viz           Enable Open3D point cloud overlay visualization"
    echo "  --no-pc-viz        Disable point cloud visualization (default)"
    echo ""
    echo -e "${YELLOW}Depth Filtering Options:${NC}"
    echo "  --depth-filter           Enable bilateral filter for depth (default: enabled)"
    echo "  --no-depth-filter        Disable bilateral filter (use raw depth)"
    echo "  --bilateral-d N          Filter diameter (default: ${DEPTH_BILATERAL_D}, range: 5-9)"
    echo "  --bilateral-sigma N      Filter sigma color/space (default: ${DEPTH_BILATERAL_SIGMA_COLOR})"
    echo ""
    echo -e "${YELLOW}Configuration Options:${NC}"
    echo "  --dataset-path PATH      Set dataset path (default: ${DATASET_PATH})"
    echo "  --publish-rate RATE      Set publish rate in Hz (default: ${PUBLISH_RATE})"
    echo "  --voxel-size SIZE        Set nvblox voxel size (default: ${VOXEL_SIZE})"
    echo "  --esdf-height HEIGHT     ESDF slice height relative to ground (default: ${ESDF_SLICE_HEIGHT_RELATIVE}m)"
    echo "  --esdf-min MIN           ESDF min obstacle height relative to ground (default: ${ESDF_SLICE_MIN_HEIGHT_RELATIVE}m)"
    echo "  --esdf-max MAX           ESDF max obstacle height relative to ground (default: ${ESDF_SLICE_MAX_HEIGHT_RELATIVE}m)"
    echo "  --rviz-config PATH       Set custom RViz config file"
    echo ""
    echo -e "${YELLOW}Initial Pose Options:${NC}"
    echo "  --init-rover-pos X Y Z           Set initial rover position (world frame)"
    echo "  --init-rover-quat QW QX QY QZ    Set initial rover orientation (quaternion)"
    echo "  --camera-offset X Y Z            Set camera offset from rover (body frame)"
    echo "  --camera-tilt DEG                Set camera tilt angle (0=horizontal, +=down)"
    echo "  --use-param-init                 Use parameter values for init pose"
    echo "  --use-gt-init                    Read init pose from GT file"
    echo "  --use-tf-gt-init                 Read init pose from /tf_gt topic (sim mode default)"
    echo "  --no-tf-gt-init                  Disable /tf_gt topic init, use params instead"
    echo "  --tf-gt-topic TOPIC              Set TF GT topic name (default: /tf_gt)"
    echo "  --tf-gt-timeout SEC              Set TF GT wait timeout (default: 10.0)"
    echo "  --ground-offset METERS           Base link to ground offset for ESDF (default: 0.15)"
    echo ""
    echo -e "${YELLOW}Navigation Options (sim mode):${NC}"
    echo "  --nav-goal X Y [YAW]     Send a single navigation goal and exit"
    echo "  --footprint-scale SCALE  Scale robot footprint (default: ${FOOTPRINT_SCALE})"
    echo "  --xy-tolerance METERS    Goal position tolerance (default: ${XY_GOAL_TOLERANCE}m)"
    echo "  --yaw-tolerance RADS     Goal orientation tolerance (default: ${YAW_GOAL_TOLERANCE}rad)"
    echo "  --ground-height METERS   Ground height for z-correction (default: auto-detect)"
    echo "  --max-linear-vel M/S     Max linear velocity (default: ${MAX_LINEAR_VEL} m/s)"
    echo "  --max-angular-vel RAD/S  Max angular velocity (default: ${MAX_ANGULAR_VEL} rad/s)"
    echo "  --no-goal-stop           Disable goal-reached stop controller"
    echo ""
    echo -e "${YELLOW}Logging Options:${NC}"
    echo "  --verbose          Enable verbose logging (detailed node output)"
    echo "  --quiet            Minimize terminal output (default)"
    echo ""
    echo -e "${YELLOW}Other Options:${NC}"
    echo "  -h, --help         Show this help message"
    echo "  --list             List running ROS2 nodes"
    echo "  --kill             Kill all related ROS2 nodes"
    echo ""
    echo -e "${YELLOW}Examples:${NC}"
    echo "  $0 --all --sim                     # Run complete pipeline with Isaac Sim raw images as input source"
    echo "  $0 --all --sim --image-type noisy  # Run complete pipeline with Isaac Sim noisy images as input source"
    echo "  $0 --sim --slam --depth --nvblox --rviz  # Sim manual driving mode (no Nav2)"
    echo "  $0 --dataset --slam --depth        # SLAM & depth with dataset as input source"
    echo ""
    echo -e "${YELLOW}Navigation Examples (sim mode):${NC}"
    echo "  $0 --sim --slam --depth --nvblox --nav2 --rviz       # Full nav pipeline"
    echo "  $0 --sim --slam --depth --nvblox --nav2 --goal-sender  # With interactive goals"
    echo "  $0 --sim --slam --depth --nvblox --nav2 --nav-goal 5 3 0  # Send single goal"
    echo "  $0 --sim --nav2 --footprint-scale 1.5                # 50% larger collision boundary"
    echo "  $0 --sim --nav2 --xy-tolerance 1.0 --yaw-tolerance 1.0  # Relaxed goal tolerance"
    echo "  $0 --sim --nav2 --ground-height 0.5                  # Set ground plane height"
    echo ""
    echo -e "${YELLOW}Ground Truth Visualization (sim mode):${NC}"
    echo "  $0 --sim --slam --depth --nvblox --ground-truth --rviz  # Compare with ground truth"
    echo "  # Ground truth point cloud is published from /shared_data/dem/sim_pointcloud_latest.ply"
    echo ""
}

setup_environment() {
    echo -e "${BLUE}[Setup]${NC} Sourcing ROS2 environment..."
    source /opt/ros/jazzy/setup.bash
    source "${ROS2_WS}/install/setup.bash"
    
    # Add CUDA to library path if available
    if [ -d "/usr/local/cuda-13.0/lib64" ]; then
        export LD_LIBRARY_PATH=/usr/local/cuda-13.0/lib64:$LD_LIBRARY_PATH
    elif [ -d "/usr/local/cuda/lib64" ]; then
        export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH
    fi
    
    # Configure ROS_DOMAIN_ID based on data source
    # - sim mode: Use domain 0 to communicate with Isaac Sim
    # - dataset mode: Use domain 1 to isolate from Isaac Sim's TF/joint_states
    if [ "${DATA_SOURCE}" = "sim" ]; then
        echo -e "${BLUE}[Setup]${NC} Configuring for cross-user DDS communication (DOMAIN_ID=0)..."
        export ROS_DOMAIN_ID=0
        export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
        export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
    else
        echo -e "${BLUE}[Setup]${NC} Using isolated ROS domain for dataset mode (DOMAIN_ID=1)..."
        export ROS_DOMAIN_ID=1
    fi
    
    echo -e "${GREEN}[Setup]${NC} Environment ready"
}

# Read base_link pose from /tf_gt topic (for sim mode initialization)
# Returns: "x y z qx qy qz qw" or empty string on failure
read_tf_gt_pose() {
    local timeout="${1:-5.0}"
    local topic="${TF_GT_TOPIC:-/tf_gt}"
    local child_frame="${TF_GT_CHILD_FRAME:-base_link}"
    
    echo -e "${BLUE}[TF_GT]${NC} Reading base_link pose from ${topic}..." >&2
    
    # Get one message from /tf_gt
    local tf_data
    tf_data=$(timeout "${timeout}" ros2 topic echo "${topic}" --once 2>/dev/null)
    
    if [ -z "${tf_data}" ]; then
        echo -e "${YELLOW}[TF_GT]${NC} Timeout reading from ${topic}" >&2
        return 1
    fi
    
    # Parse pose values from the transform message
    local transform_block
    transform_block=$(echo "${tf_data}" | grep -A 15 "child_frame_id: ${child_frame}")
    
    if [ -z "${transform_block}" ]; then
        echo -e "${YELLOW}[TF_GT]${NC} Could not find ${child_frame} in ${topic}" >&2
        return 1
    fi
    
    # Extract translation
    local x_val y_val z_val
    x_val=$(echo "${transform_block}" | grep -A 4 "translation:" | grep "x:" | head -1 | awk '{print $2}')
    y_val=$(echo "${transform_block}" | grep -A 4 "translation:" | grep "y:" | head -1 | awk '{print $2}')
    z_val=$(echo "${transform_block}" | grep -A 4 "translation:" | grep "z:" | head -1 | awk '{print $2}')
    
    # Extract rotation
    local qx_val qy_val qz_val qw_val
    qx_val=$(echo "${transform_block}" | grep -A 5 "rotation:" | grep "x:" | head -1 | awk '{print $2}')
    qy_val=$(echo "${transform_block}" | grep -A 5 "rotation:" | grep "y:" | head -1 | awk '{print $2}')
    qz_val=$(echo "${transform_block}" | grep -A 5 "rotation:" | grep "z:" | head -1 | awk '{print $2}')
    qw_val=$(echo "${transform_block}" | grep -A 5 "rotation:" | grep "w:" | head -1 | awk '{print $2}')
    
    if [ -z "${x_val}" ] || [ -z "${z_val}" ] || [ -z "${qw_val}" ]; then
        echo -e "${YELLOW}[TF_GT]${NC} Could not parse pose for ${child_frame}" >&2
        return 1
    fi
    
    echo -e "${GREEN}[TF_GT]${NC} Base link pose: pos=[${x_val}, ${y_val}, ${z_val}] rot=[${qx_val}, ${qy_val}, ${qz_val}, ${qw_val}]" >&2
    
    # Store in global variables for use by other functions
    TF_GT_X="${x_val}"
    TF_GT_Y="${y_val}"
    TF_GT_Z="${z_val}"
    TF_GT_QX="${qx_val}"
    TF_GT_QY="${qy_val}"
    TF_GT_QZ="${qz_val}"
    TF_GT_QW="${qw_val}"
    
    echo "${x_val} ${y_val} ${z_val} ${qx_val} ${qy_val} ${qz_val} ${qw_val}"
    return 0
}

# Read base_link Z height from /tf_gt topic (for sim mode ESDF slice height calculation)
read_tf_gt_height() {
    local timeout="${1:-5.0}"
    
    # If we already have the pose, just return Z
    if [ -n "${TF_GT_Z}" ]; then
        echo -e "${GREEN}[TF_GT]${NC} Using cached base link height: ${TF_GT_Z}m" >&2
        echo "${TF_GT_Z}"
        return 0
    fi
    
    # Otherwise read the full pose
    local pose_result
    pose_result=$(read_tf_gt_pose "${timeout}")
    
    if [ $? -eq 0 ]; then
        echo "${TF_GT_Z}"
        return 0
    fi
    
    return 1
}

# Read camera transform from /tf topic (base_link -> camera frame)
# Sets global variables: CAM_TF_X, CAM_TF_Y, CAM_TF_Z, CAM_TF_QX, CAM_TF_QY, CAM_TF_QZ, CAM_TF_QW, CAM_TF_TILT_DEG
read_camera_tf() {
    local timeout="${1:-10.0}"
    local parent_frame="${2:-base_link}"
    local child_frame="${3:-${SIM_CAMERA_FRAME_NAME}}"
    
    echo -e "${CYAN}[CamTF]${NC} Reading camera transform: ${parent_frame} -> ${child_frame}..." >&2
    
    # Use tf2_echo and capture the first complete transform block including the matrix
    local tf_output
    tf_output=$(timeout "${timeout}" bash -c "ros2 run tf2_ros tf2_echo '${parent_frame}' '${child_frame}' 2>&1" | grep -A 12 'At time' | head -14)
    
    if [ -z "${tf_output}" ]; then
        echo -e "${YELLOW}[CamTF]${NC} Timeout waiting for transform" >&2
        return 1
    fi
    
    echo -e "${CYAN}[CamTF]${NC} tf2_echo output received" >&2
    
    # Parse translation: "- Translation: [x, y, z]"
    local translation_line
    translation_line=$(echo "${tf_output}" | grep "Translation:" | head -1)
    if [ -z "${translation_line}" ]; then
        echo -e "${YELLOW}[CamTF]${NC} Could not find translation" >&2
        return 1
    fi
    
    # Extract x, y, z from "[x, y, z]"
    local x_val y_val z_val
    x_val=$(echo "${translation_line}" | sed 's/.*\[\([^,]*\),.*/\1/' | tr -d ' ')
    y_val=$(echo "${translation_line}" | sed 's/.*,\s*\([^,]*\),.*/\1/' | tr -d ' ')
    z_val=$(echo "${translation_line}" | sed 's/.*,.*,\s*\([^]]*\)\]/\1/' | tr -d ' ')
    
    # Parse quaternion: "- Rotation: in Quaternion [x, y, z, w]"
    local quat_line
    quat_line=$(echo "${tf_output}" | grep "Quaternion" | head -1)
    if [ -z "${quat_line}" ]; then
        echo -e "${YELLOW}[CamTF]${NC} Could not find quaternion" >&2
        return 1
    fi
    
    # Extract quaternion values from "[x, y, z, w]"
    local qx_val qy_val qz_val qw_val
    local quat_part
    quat_part=$(echo "${quat_line}" | sed 's/.*\[\(.*\)\]/\1/')
    qx_val=$(echo "${quat_part}" | cut -d',' -f1 | tr -d ' ')
    qy_val=$(echo "${quat_part}" | cut -d',' -f2 | tr -d ' ')
    qz_val=$(echo "${quat_part}" | cut -d',' -f3 | tr -d ' ')
    qw_val=$(echo "${quat_part}" | cut -d',' -f4 | tr -d ' ')
    
    if [ -z "${x_val}" ] || [ -z "${z_val}" ] || [ -z "${qw_val}" ]; then
        echo -e "${YELLOW}[CamTF]${NC} Could not parse transform values" >&2
        echo -e "${YELLOW}[CamTF]${NC} Raw output: ${tf_output}" >&2
        return 1
    fi
    
    # Store in global variables
    CAM_TF_X="$(echo -n "${x_val}" | tr -d '\n')"
    CAM_TF_Y="$(echo -n "${y_val}" | tr -d '\n')"
    CAM_TF_Z="$(echo -n "${z_val}" | tr -d '\n')"
    CAM_TF_QX="$(echo -n "${qx_val}" | tr -d '\n')"
    CAM_TF_QY="$(echo -n "${qy_val}" | tr -d '\n')"
    CAM_TF_QZ="$(echo -n "${qz_val}" | tr -d '\n')"
    CAM_TF_QW="$(echo -n "${qw_val}" | tr -d '\n')"
    
    # Calculate camera tilt from rotation matrix
    # The tilt is how much the camera looks down from horizontal
    # Camera Z-axis (forward/viewing direction) in body frame is the 3rd column of rotation matrix
    # We need to calculate: tilt = atan2(-z_component, sqrt(x^2 + y^2)) of camera forward direction
    # 
    # From tf2_echo matrix output, the 3rd column represents where camera Z points in body frame
    # Example matrix:
    #   0.000 -0.342  0.940  0.480
    #  -1.000 -0.000  0.000  0.200
    #   0.000 -0.940 -0.342  1.145
    # 3rd column = [0.940, 0.000, -0.342] = camera forward in body frame
    # tilt = atan2(0.342, 0.940) ≈ 20 degrees (looking down)
    
    CAM_TF_TILT_DEG=$(python3 -c "
import math

# Quaternion to rotation matrix
qx, qy, qz, qw = ${qx_val}, ${qy_val}, ${qz_val}, ${qw_val}

# Normalize quaternion
norm = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
qx, qy, qz, qw = qx/norm, qy/norm, qz/norm, qw/norm

# Rotation matrix elements (3rd column = camera Z axis direction in body frame)
# R[:,2] = camera forward direction
r02 = 2*(qx*qz + qy*qw)
r12 = 2*(qy*qz - qx*qw)
r22 = 1 - 2*(qx*qx + qy*qy)

# Camera forward direction in body frame: [r02, r12, r22]
# For body frame: X=forward, Y=left, Z=up
# Camera tilt = angle below horizontal = atan2(-z_component, x_component)
# Positive tilt = looking down

# The camera Z (forward) direction in body frame
cam_fwd_x = r02  # body X component
cam_fwd_z = r22  # body Z component

# Tilt angle: positive when looking down (camera Z has negative body Z component)
tilt_rad = math.atan2(-cam_fwd_z, cam_fwd_x)
tilt_deg = math.degrees(tilt_rad)

print(f'{tilt_deg:.1f}')
" 2>/dev/null || echo "20.0")
    
    # Ensure it's always a float
    if [[ "${CAM_TF_TILT_DEG}" =~ ^-?[0-9]+$ ]]; then
        CAM_TF_TILT_DEG="${CAM_TF_TILT_DEG}.0"
    fi
    
    echo -e "${GREEN}[CamTF]${NC} Camera transform: pos=[${x_val}, ${y_val}, ${z_val}] tilt=${CAM_TF_TILT_DEG}°" >&2
    echo -e "${GREEN}[CamTF]${NC} Camera quaternion: [${qx_val}, ${qy_val}, ${qz_val}, ${qw_val}]" >&2
    
    return 0
}

configure_topics() {
    # Set topics, intrinsic file, SLAM settings, and initial pose based on data source mode
    if [ "${DATA_SOURCE}" = "sim" ]; then
        # Resolve sim image topics based on image type
        case "${SIM_IMAGE_TYPE}" in
            raw)
                SIM_LEFT_TOPIC="${SIM_LEFT_TOPIC_RAW}"
                SIM_RIGHT_TOPIC="${SIM_RIGHT_TOPIC_RAW}"
                echo -e "${CYAN}[Config]${NC} Using raw (uncompressed) image topics"
                ;;
            noisy)
                SIM_LEFT_TOPIC="${SIM_LEFT_TOPIC_NOISY}"
                SIM_RIGHT_TOPIC="${SIM_RIGHT_TOPIC_NOISY}"
                echo -e "${CYAN}[Config]${NC} Using noisy image topics (~4-6 FPS, RELIABLE QoS)"
                ;;
            enhanced)
                SIM_LEFT_TOPIC="${SIM_LEFT_TOPIC_ENHANCED}"
                SIM_RIGHT_TOPIC="${SIM_RIGHT_TOPIC_ENHANCED}"
                echo -e "${CYAN}[Config]${NC} Using enhanced image topics (~4-6 FPS, RELIABLE QoS)"
                ;;
            denoised)
                SIM_LEFT_TOPIC="${SIM_LEFT_TOPIC_DENOISED}"
                SIM_RIGHT_TOPIC="${SIM_RIGHT_TOPIC_DENOISED}"
                # Use 512x512 resolution settings for denoised images
                INTRINSIC_FILE="${ROS2_WS}/assets/sim_intrinsics_512.txt"
                SLAM_SETTINGS="${ROS2_WS}/assets/sim_slam_settings_512.yaml"
                DEPTH_SCALE=1.0  # Keep 512x512 output (no downscale)
                echo -e "${CYAN}[Config]${NC} Using denoised image topics (512x512 resolution)"
                ;;
            adjusted)
                SIM_LEFT_TOPIC="${SIM_LEFT_TOPIC_ADJUSTED}"
                SIM_RIGHT_TOPIC="${SIM_RIGHT_TOPIC_ADJUSTED}"
                echo -e "${CYAN}[Config]${NC} Using adjusted image topics"
                ;;
            *)
                echo -e "${RED}[Error]${NC} Unknown image type: ${SIM_IMAGE_TYPE}"
                echo -e "${RED}[Error]${NC} Valid types: raw, noisy, enhanced, denoised, adjusted"
                exit 1
                ;;
        esac
        
        LEFT_TOPIC="${SIM_LEFT_TOPIC}"
        RIGHT_TOPIC="${SIM_RIGHT_TOPIC}"
        # Use default settings if not already set (for non-denoised modes)
        INTRINSIC_FILE="${INTRINSIC_FILE:-${SIM_INTRINSIC_FILE}}"
        SLAM_SETTINGS="${SLAM_SETTINGS:-${SIM_SLAM_SETTINGS}}"
        DEPTH_SCALE="${DEPTH_SCALE:-0.5}"  # Default scale for 1024->512
        # Enable tf_gt initialization for sim mode (if not explicitly disabled)
        if [ "${USE_TF_GT_INIT}" = "false" ] && [ "${EXPLICIT_TF_GT_DISABLED}" != "true" ]; then
            USE_TF_GT_INIT="true"
        fi
        # Use RELIABLE QoS for sim raw image topics (they are published as RELIABLE)
        USE_BEST_EFFORT="false"
        # Set sim mode initial pose (used as fallback if tf_gt fails)
        [ -z "${INIT_ROVER_X}" ] && INIT_ROVER_X="${SIM_INIT_ROVER_X}"
        [ -z "${INIT_ROVER_Y}" ] && INIT_ROVER_Y="${SIM_INIT_ROVER_Y}"
        [ -z "${INIT_ROVER_Z}" ] && INIT_ROVER_Z="${SIM_INIT_ROVER_Z}"
        [ -z "${INIT_ROVER_QW}" ] && INIT_ROVER_QW="${SIM_INIT_ROVER_QW}"
        [ -z "${INIT_ROVER_QX}" ] && INIT_ROVER_QX="${SIM_INIT_ROVER_QX}"
        [ -z "${INIT_ROVER_QY}" ] && INIT_ROVER_QY="${SIM_INIT_ROVER_QY}"
        [ -z "${INIT_ROVER_QZ}" ] && INIT_ROVER_QZ="${SIM_INIT_ROVER_QZ}"
        # Set base link ground offset
        [ -z "${BASE_LINK_GROUND_OFFSET}" ] && BASE_LINK_GROUND_OFFSET="${SIM_BASE_LINK_GROUND_OFFSET}"
        
        # Read /tf_gt pose once and cache it for use by ESDF calculation and static TF
        if [ -z "${TF_GT_Z}" ]; then
            read_tf_gt_pose 10.0 > /dev/null
        fi
        
        # Read camera transform from /tf (base_link -> Camera_OmniVision_OV9782_Left)
        # Increase timeout since Isaac Sim may take time to publish TF
        if read_camera_tf 15.0 base_link "${SIM_CAMERA_FRAME_NAME}"; then
            # Use values from Isaac Sim TF
            CAMERA_OFFSET_X="${CAM_TF_X}"
            CAMERA_OFFSET_Y="${CAM_TF_Y}"
            CAMERA_OFFSET_Z="${CAM_TF_Z}"
            CAMERA_TILT_DEG="${CAM_TF_TILT_DEG}"
            echo -e "${GREEN}[Config]${NC} Camera offset from /tf: [${CAMERA_OFFSET_X}, ${CAMERA_OFFSET_Y}, ${CAMERA_OFFSET_Z}]"
            echo -e "${GREEN}[Config]${NC} Camera tilt from /tf: ${CAMERA_TILT_DEG}°"
        else
            # Fallback to hardcoded parameters
            echo -e "${YELLOW}[Config]${NC} Using fallback camera parameters"
            [ -z "${CAMERA_OFFSET_X}" ] && CAMERA_OFFSET_X="${SIM_CAMERA_OFFSET_X}"
            [ -z "${CAMERA_OFFSET_Y}" ] && CAMERA_OFFSET_Y="${SIM_CAMERA_OFFSET_Y}"
            [ -z "${CAMERA_OFFSET_Z}" ] && CAMERA_OFFSET_Z="${SIM_CAMERA_OFFSET_Z}"
            [ -z "${CAMERA_TILT_DEG}" ] && CAMERA_TILT_DEG="${SIM_CAMERA_TILT_DEG}"
        fi
        
        # Set sim mode ESDF settings
        # Calculate ESDF heights relative to ground level (base_link_z - ground_offset)
        if [ -n "${TF_GT_Z}" ]; then
            # Ground level in map frame
            local ground_level=$(echo "scale=3; ${TF_GT_Z} - ${BASE_LINK_GROUND_OFFSET}" | bc)
            echo -e "${GREEN}[Config]${NC} Ground level calculated: ${TF_GT_Z} - ${BASE_LINK_GROUND_OFFSET} = ${ground_level}m"
            
            # ESDF slice height = ground + relative offset
            ESDF_SLICE_HEIGHT=$(echo "scale=3; ${ground_level} + ${ESDF_SLICE_HEIGHT_RELATIVE}" | bc)
            echo -e "${GREEN}[Config]${NC} ESDF slice height: ${ground_level} + ${ESDF_SLICE_HEIGHT_RELATIVE} = ${ESDF_SLICE_HEIGHT}m"
            
            # ESDF min height = ground + min offset (obstacles below this are ignored)
            ESDF_SLICE_MIN_HEIGHT=$(echo "scale=3; ${ground_level} + ${ESDF_SLICE_MIN_HEIGHT_RELATIVE}" | bc)
            echo -e "${GREEN}[Config]${NC} ESDF min height: ${ground_level} + ${ESDF_SLICE_MIN_HEIGHT_RELATIVE} = ${ESDF_SLICE_MIN_HEIGHT}m"
            
            # ESDF max height = ground + max offset (obstacles above this are ignored)
            ESDF_SLICE_MAX_HEIGHT=$(echo "scale=3; ${ground_level} + ${ESDF_SLICE_MAX_HEIGHT_RELATIVE}" | bc)
            echo -e "${GREEN}[Config]${NC} ESDF max height: ${ground_level} + ${ESDF_SLICE_MAX_HEIGHT_RELATIVE} = ${ESDF_SLICE_MAX_HEIGHT}m"
            
            echo -e "${GREEN}[Config]${NC} ESDF range: [${ESDF_SLICE_MIN_HEIGHT}, ${ESDF_SLICE_MAX_HEIGHT}]m (absolute in map frame)"
        else
            # Fallback to relative values as absolute if tf_gt not available
            ESDF_SLICE_HEIGHT="${ESDF_SLICE_HEIGHT_RELATIVE}"
            ESDF_SLICE_MIN_HEIGHT="${ESDF_SLICE_MIN_HEIGHT_RELATIVE}"
            ESDF_SLICE_MAX_HEIGHT="${ESDF_SLICE_MAX_HEIGHT_RELATIVE}"
            echo -e "${YELLOW}[Config]${NC} WARNING: tf_gt not available, using relative values as absolute!"
            echo -e "${YELLOW}[Config]${NC} ESDF settings: height=${ESDF_SLICE_HEIGHT}m, range=[${ESDF_SLICE_MIN_HEIGHT}, ${ESDF_SLICE_MAX_HEIGHT}]m"
        fi
        echo -e "${CYAN}[Config]${NC} Using Isaac Sim configuration (image type: ${SIM_IMAGE_TYPE}):"
    else
        LEFT_TOPIC="${DATASET_LEFT_TOPIC}"
        RIGHT_TOPIC="${DATASET_RIGHT_TOPIC}"
        INTRINSIC_FILE="${DATASET_INTRINSIC_FILE}"
        SLAM_SETTINGS="${DATASET_SLAM_SETTINGS}"
        DEPTH_SCALE="${DEPTH_SCALE:-1.0}"  # Default: no downscale for dataset
        # Set dataset mode initial pose (if not overridden by command line)
        [ -z "${INIT_ROVER_X}" ] && INIT_ROVER_X="${DATASET_INIT_ROVER_X}"
        [ -z "${INIT_ROVER_Y}" ] && INIT_ROVER_Y="${DATASET_INIT_ROVER_Y}"
        [ -z "${INIT_ROVER_Z}" ] && INIT_ROVER_Z="${DATASET_INIT_ROVER_Z}"
        [ -z "${INIT_ROVER_QW}" ] && INIT_ROVER_QW="${DATASET_INIT_ROVER_QW}"
        [ -z "${INIT_ROVER_QX}" ] && INIT_ROVER_QX="${DATASET_INIT_ROVER_QX}"
        [ -z "${INIT_ROVER_QY}" ] && INIT_ROVER_QY="${DATASET_INIT_ROVER_QY}"
        [ -z "${INIT_ROVER_QZ}" ] && INIT_ROVER_QZ="${DATASET_INIT_ROVER_QZ}"
        [ -z "${CAMERA_OFFSET_X}" ] && CAMERA_OFFSET_X="${DATASET_CAMERA_OFFSET_X}"
        [ -z "${CAMERA_OFFSET_Y}" ] && CAMERA_OFFSET_Y="${DATASET_CAMERA_OFFSET_Y}"
        [ -z "${CAMERA_OFFSET_Z}" ] && CAMERA_OFFSET_Z="${DATASET_CAMERA_OFFSET_Z}"
        [ -z "${CAMERA_TILT_DEG}" ] && CAMERA_TILT_DEG="${DATASET_CAMERA_TILT_DEG}"
        # Set dataset mode ESDF settings (dataset uses relative values as absolute since ground is at z=0)
        [ -z "${ESDF_SLICE_HEIGHT}" ] && ESDF_SLICE_HEIGHT="${DATASET_ESDF_SLICE_HEIGHT}"
        [ -z "${ESDF_SLICE_MIN_HEIGHT}" ] && ESDF_SLICE_MIN_HEIGHT="${DATASET_ESDF_SLICE_MIN_HEIGHT}"
        [ -z "${ESDF_SLICE_MAX_HEIGHT}" ] && ESDF_SLICE_MAX_HEIGHT="${DATASET_ESDF_SLICE_MAX_HEIGHT}"
        echo -e "${CYAN}[Config]${NC} Using dataset publisher configuration:"
    fi
    echo -e "  Left topic:     ${LEFT_TOPIC}"
    echo -e "  Right topic:    ${RIGHT_TOPIC}"
    echo -e "  Intrinsic file: ${INTRINSIC_FILE}"
    echo -e "  SLAM settings:  ${SLAM_SETTINGS}"
    if [ "${USE_TF_GT_INIT}" = "true" ]; then
        echo -e "  Init pose:      from ${TF_GT_TOPIC} topic (child_frame: ${TF_GT_CHILD_FRAME})"
    else
        echo -e "  Rover position: [${INIT_ROVER_X}, ${INIT_ROVER_Y}, ${INIT_ROVER_Z}]"
    fi
    echo -e "  Camera offset:  [${CAMERA_OFFSET_X}, ${CAMERA_OFFSET_Y}, ${CAMERA_OFFSET_Z}]"
    echo -e "  Camera tilt:    ${CAMERA_TILT_DEG}°"
    echo -e "  ESDF slice:     height=${ESDF_SLICE_HEIGHT}m, range=[${ESDF_SLICE_MIN_HEIGHT}, ${ESDF_SLICE_MAX_HEIGHT}]m"
    echo ""
}

run_static_tf() {
    echo -e "${BLUE}[StaticTF]${NC} Starting static transform publishers..."
    
    # map -> odom (identity - required for Nav2)
    ros2 run tf2_ros static_transform_publisher \
        0 0 0 0 0 0 map odom \
        --ros-args -r __node:=static_tf_map_odom &
    
    if [ "${DATA_SOURCE}" = "sim" ]; then
        # Sim mode: 
        # - SLAM publishes map -> camera_link
        # - Static TF: camera_link -> base_link (inverse of base_link -> camera from Isaac Sim)
        #
        # Isaac Sim provides: base_link -> Camera_* (stored in CAM_TF_X/Y/Z/QX/QY/QZ/QW)
        # We need the inverse: camera_link -> base_link
        #
        # Use Python for clean inverse transform calculation (numpy only, no scipy)
        
        if [ -n "${CAM_TF_QX}" ] && [ -n "${CAM_TF_QW}" ]; then
            # Calculate inverse transform using Python with numpy only
            local inv_tf
            inv_tf=$(python3 -c "
import numpy as np

# Forward transform: base_link -> camera
tx, ty, tz = ${CAM_TF_X}, ${CAM_TF_Y}, ${CAM_TF_Z}
qx, qy, qz, qw = ${CAM_TF_QX}, ${CAM_TF_QY}, ${CAM_TF_QZ}, ${CAM_TF_QW}

# Quaternion to rotation matrix
# R = [[1-2(qy^2+qz^2), 2(qxqy-qzqw), 2(qxqz+qyqw)],
#      [2(qxqy+qzqw), 1-2(qx^2+qz^2), 2(qyqz-qxqw)],
#      [2(qxqz-qyqw), 2(qyqz+qxqw), 1-2(qx^2+qy^2)]]
R = np.array([
    [1-2*(qy*qy+qz*qz), 2*(qx*qy-qz*qw), 2*(qx*qz+qy*qw)],
    [2*(qx*qy+qz*qw), 1-2*(qx*qx+qz*qz), 2*(qy*qz-qx*qw)],
    [2*(qx*qz-qy*qw), 2*(qy*qz+qx*qw), 1-2*(qx*qx+qy*qy)]
])
t = np.array([tx, ty, tz])

# Inverse: R_inv = R^T, t_inv = -R^T @ t
R_inv = R.T
t_inv = -R_inv @ t

# Inverse quaternion: conjugate = [-qx, -qy, -qz, qw]
qx_inv, qy_inv, qz_inv, qw_inv = -qx, -qy, -qz, qw

print(f'{t_inv[0]:.6f} {t_inv[1]:.6f} {t_inv[2]:.6f} {qx_inv:.6f} {qy_inv:.6f} {qz_inv:.6f} {qw_inv:.6f}')
" 2>/dev/null)
            
            if [ -n "${inv_tf}" ]; then
                local tx ty tz qx qy qz qw
                read tx ty tz qx qy qz qw <<< "${inv_tf}"
                echo -e "${BLUE}[StaticTF]${NC} camera_link -> base_link:"
                echo -e "${BLUE}[StaticTF]${NC}   Translation: [${tx}, ${ty}, ${tz}]"
                echo -e "${BLUE}[StaticTF]${NC}   Quaternion (xyzw): [${qx}, ${qy}, ${qz}, ${qw}]"
                
                ros2 run tf2_ros static_transform_publisher \
                    "${tx}" "${ty}" "${tz}" \
                    "${qx}" "${qy}" "${qz}" "${qw}" \
                    "${CAMERA_FRAME}" base_link \
                    --ros-args -r __node:=static_tf_camera_base &
            else
                echo -e "${YELLOW}[StaticTF]${NC} Python inverse failed, using simple negate"
                ros2 run tf2_ros static_transform_publisher \
                    "-${CAM_TF_X}" "-${CAM_TF_Y}" "-${CAM_TF_Z}" \
                    0 0 0 1 \
                    "${CAMERA_FRAME}" base_link \
                    --ros-args -r __node:=static_tf_camera_base &
            fi
        else
            # No camera TF available - use identity
            echo -e "${YELLOW}[StaticTF]${NC} No camera TF, using identity transform"
            ros2 run tf2_ros static_transform_publisher \
                0 0 0 0 0 0 \
                "${CAMERA_FRAME}" base_link \
                --ros-args -r __node:=static_tf_camera_base &
        fi
    else
        # Dataset mode: No external odometry source
        # camera_link is the primary frame, base_link is at same position
        ros2 run tf2_ros static_transform_publisher \
            0 0 0 0 0 0 "${CAMERA_FRAME}" base_link \
            --ros-args -r __node:=static_tf_camera_base &
        
        # odom -> base_link (identity - for Nav2 compatibility)
        ros2 run tf2_ros static_transform_publisher \
            0 0 0 0 0 0 odom base_link \
            --ros-args -r __node:=static_tf_odom_base &
    fi
    
    sleep 1
    echo -e "${GREEN}[StaticTF]${NC} Started"
}

run_dataset_publisher() {
    echo -e "${BLUE}[Dataset]${NC} Starting dataset publisher..."
    echo -e "${BLUE}[Dataset]${NC} Path: ${DATASET_PATH}"
    echo -e "${BLUE}[Dataset]${NC} Rate: ${PUBLISH_RATE} Hz"
    echo -e "${BLUE}[Dataset]${NC} GT depth publishing: enabled (depth subdir)"
    
    ros2 run orb_slam3_ros2 dataset_publisher \
        --ros-args \
        -p dataset_path:="${DATASET_PATH}" \
        -p left_subdir:="${LEFT_SUBDIR}" \
        -p right_subdir:="${RIGHT_SUBDIR}" \
        -p depth_subdir:="image0/depth" \
        -p publish_rate:="${PUBLISH_RATE}" \
        -p frame_id:="${CAMERA_FRAME}" \
        -p loop:=false \
        -p publish_gt_depth:=true &
    
    sleep 2
    echo -e "${GREEN}[Dataset]${NC} Started"
}

run_slam() {
    echo -e "${BLUE}[SLAM]${NC} Starting ORB-SLAM3..."
    if [ "${VERBOSE}" = "true" ]; then
        echo -e "${BLUE}[SLAM]${NC} Settings: ${SLAM_SETTINGS}"
        echo -e "${BLUE}[SLAM]${NC} Viewer: ${SLAM_VIZ}"
        echo -e "${BLUE}[SLAM]${NC} Camera tilt: ${CAMERA_TILT_DEG}°"
        echo -e "${BLUE}[SLAM]${NC} Init pose from tf_gt: ${USE_TF_GT_INIT}"
        echo -e "${BLUE}[SLAM]${NC} Init pose from params: ${USE_PARAM_INIT_POSE}"
        echo -e "${BLUE}[SLAM]${NC} Left topic: ${LEFT_TOPIC}"
        echo -e "${BLUE}[SLAM]${NC} Right topic: ${RIGHT_TOPIC}"
        if [ "${USE_TF_GT_INIT}" = "true" ]; then
            echo -e "${BLUE}[SLAM]${NC} TF GT topic: ${TF_GT_TOPIC} (child_frame: ${TF_GT_CHILD_FRAME})"
        elif [ "${USE_PARAM_INIT_POSE}" = "true" ]; then
            echo -e "${BLUE}[SLAM]${NC} Rover pos: [${INIT_ROVER_X}, ${INIT_ROVER_Y}, ${INIT_ROVER_Z}]"
            echo -e "${BLUE}[SLAM]${NC} Rover quat: [${INIT_ROVER_QW}, ${INIT_ROVER_QX}, ${INIT_ROVER_QY}, ${INIT_ROVER_QZ}]"
        fi
        echo -e "${BLUE}[SLAM]${NC} Camera offset: [${CAMERA_OFFSET_X}, ${CAMERA_OFFSET_Y}, ${CAMERA_OFFSET_Z}]"
    fi
    
    # Determine if we have camera quaternion from /tf
    local use_cam_quat="false"
    if [ -n "${CAM_TF_QX}" ] && [ -n "${CAM_TF_QW}" ]; then
        use_cam_quat="true"
    fi
    
    ros2 run orb_slam3_ros2 stereo_slam \
        --ros-args \
        -p vocabulary_path:="${VOCABULARY_PATH}" \
        -p settings_path:="${SLAM_SETTINGS}" \
        -p enable_viewer:="${SLAM_VIZ}" \
        -p left_topic:="${LEFT_TOPIC}" \
        -p right_topic:="${RIGHT_TOPIC}" \
        -p camera_tilt_deg:="${CAMERA_TILT_DEG}" \
        -p output_frame_id:="${GLOBAL_FRAME}" \
        -p camera_frame_id:="${CAMERA_FRAME}" \
        -p timestamp_mode:=current \
        -p pose_mode:=camera \
        -p use_tf_gt_init:="${USE_TF_GT_INIT}" \
        -p tf_gt_topic:="${TF_GT_TOPIC}" \
        -p tf_gt_child_frame:="${TF_GT_CHILD_FRAME}" \
        -p tf_gt_timeout:="${TF_GT_TIMEOUT}" \
        -p use_param_init_pose:="${USE_PARAM_INIT_POSE}" \
        -p init_rover_x:="${INIT_ROVER_X}" \
        -p init_rover_y:="${INIT_ROVER_Y}" \
        -p init_rover_z:="${INIT_ROVER_Z}" \
        -p init_rover_qw:="${INIT_ROVER_QW}" \
        -p init_rover_qx:="${INIT_ROVER_QX}" \
        -p init_rover_qy:="${INIT_ROVER_QY}" \
        -p init_rover_qz:="${INIT_ROVER_QZ}" \
        -p camera_offset_x:="${CAMERA_OFFSET_X}" \
        -p camera_offset_y:="${CAMERA_OFFSET_Y}" \
        -p camera_offset_z:="${CAMERA_OFFSET_Z}" \
        -p camera_qw:="${CAM_TF_QW:-1.0}" \
        -p camera_qx:="${CAM_TF_QX:-0.0}" \
        -p camera_qy:="${CAM_TF_QY:-0.0}" \
        -p camera_qz:="${CAM_TF_QZ:-0.0}" \
        -p use_camera_quat:="${use_cam_quat}" \
        -p use_best_effort:="${USE_BEST_EFFORT}" \
        -p gt_file_path:="${DATASET_PATH}/gt.txt" &
    
    sleep 3
    echo -e "${GREEN}[SLAM]${NC} Started"
}

run_depth() {
    echo -e "${BLUE}[Depth]${NC} Starting FoundationStereo depth node..."
    if [ "${VERBOSE}" = "true" ]; then
        echo -e "${BLUE}[Depth]${NC} Visualization: ${DEPTH_VIZ}"
        echo -e "${BLUE}[Depth]${NC} Bilateral filter: ${DEPTH_BILATERAL_FILTER} (d=${DEPTH_BILATERAL_D}, sigma=${DEPTH_BILATERAL_SIGMA_COLOR})"
        echo -e "${BLUE}[Depth]${NC} Conda env: ${DEPTH_CONDA_ENV}"
        echo -e "${BLUE}[Depth]${NC} Left topic: ${LEFT_TOPIC}"
        echo -e "${BLUE}[Depth]${NC} Right topic: ${RIGHT_TOPIC}"
        echo -e "${BLUE}[Depth]${NC} Intrinsic file: ${INTRINSIC_FILE}"
    fi
    
    # Run depth node in a subshell with conda environment
    (
        # Initialize conda in the subshell
        source "${CONDA_BASE}/etc/profile.d/conda.sh"
        conda activate "${DEPTH_CONDA_ENV}"
        
        # Re-source ROS2 after conda activation
        source /opt/ros/jazzy/setup.bash
        source "${ROS2_WS}/install/setup.bash"
        
        # Suppress PyTorch FutureWarning and xFormers UserWarning
        export PYTHONWARNINGS="ignore::FutureWarning,ignore::UserWarning"
        # Suppress xFormers detailed error messages
        export XFORMERS_MORE_DETAILS=0
        
        # Run depth node directly with conda Python (avoids cv_bridge NumPy conflict)
        # ros2 run uses system Python which causes NumPy version mismatch with cv_bridge
        # viz_mode: 'cv2' for direct OpenCV window (smoother), 'topic' for ROS2 topic
        # timestamp_mode: 'current' uses current wall time (ensures TF sync for nvblox)
        # GT depth evaluation: compares with GT depth and shows abs_rel metric
        # PC visualization: Open3D point cloud overlay (pred=orange, gt=cyan)
        python "${ROS2_WS}/install/foundation_stereo_ros2/lib/foundation_stereo_ros2/depth_node.py" \
            --ros-args \
            -p left_topic:="${LEFT_TOPIC}" \
            -p right_topic:="${RIGHT_TOPIC}" \
            -p intrinsic_file:="${INTRINSIC_FILE}" \
            -p scale:="${DEPTH_SCALE}" \
            -p valid_iters:=32 \
            -p z_far:=10.0 \
            -p camera_frame_id:="${CAMERA_FRAME}" \
            -p enable_visualization:="${DEPTH_VIZ}" \
            -p viz_mode:=cv2 \
            -p skip_frames_during_processing:=true \
            -p timestamp_mode:=current \
            -p enable_gt_evaluation:=true \
            -p data_source:="${DATA_SOURCE}" \
            -p gt_depth_topic:="/depth/gt_image" \
            -p sim_gt_depth_topic:="/front_camera/depth/depth" \
            -p enable_pc_visualization:="${DEPTH_PC_VIZ}" \
            -p pc_downsample_factor:=2 \
            -p verbose:="${VERBOSE}" \
            -p enable_bilateral_filter:="${DEPTH_BILATERAL_FILTER}" \
            -p bilateral_d:="${DEPTH_BILATERAL_D}" \
            -p bilateral_sigma_color:="${DEPTH_BILATERAL_SIGMA_COLOR}" \
            -p bilateral_sigma_space:="${DEPTH_BILATERAL_SIGMA_SPACE}" 2>&1 | grep -v -E "xFormers|XFORMERS|xformers|Traceback|File \"/home|OSError:|The above exception|using MLP layer|PyTorch 2\.|Python  3\.|Memory-efficient|torch\.ops\.|ctypes\.CDLL|self\._handle|_build_metadata|_register_extensions|cache found in|^\s+\^+\s*$|CUDA 1208"
    ) &
    
    sleep 5
    echo -e "${GREEN}[Depth]${NC} Started"
}


run_nvblox() {
    echo -e "${BLUE}[nvblox]${NC} Starting nvblox 3D mapping..."
    if [ "${VERBOSE}" = "true" ]; then
        echo -e "${BLUE}[nvblox]${NC} Voxel size: ${VOXEL_SIZE}m"
        echo -e "${BLUE}[nvblox]${NC} ESDF slice: height=${ESDF_SLICE_HEIGHT}m, range=[${ESDF_SLICE_MIN_HEIGHT}, ${ESDF_SLICE_MAX_HEIGHT}]m"
    fi
    
    ros2 launch nvblox_integration nvblox.launch.py \
        voxel_size:="${VOXEL_SIZE}" \
        global_frame:="${GLOBAL_FRAME}" \
        camera_frame:="${CAMERA_FRAME}" \
        esdf_slice_height:="${ESDF_SLICE_HEIGHT}" \
        esdf_slice_min_height:="${ESDF_SLICE_MIN_HEIGHT}" \
        esdf_slice_max_height:="${ESDF_SLICE_MAX_HEIGHT}" &
    
    sleep 3
    echo -e "${GREEN}[nvblox]${NC} Started"
}

run_rviz() {
    echo -e "${BLUE}[RViz]${NC} Starting RViz2..."
    
    # Use Nav2 RViz config if Nav2 is enabled
    local rviz_config="${RVIZ_CONFIG}"
    if [ "$RUN_NAV2" = true ]; then
        local nav2_rviz="${ROS2_WS}/install/nvblox_integration/share/nvblox_integration/config/nav2_visualization.rviz"
        if [ -f "${nav2_rviz}" ]; then
            rviz_config="${nav2_rviz}"
            echo -e "${BLUE}[RViz]${NC} Using Nav2 visualization config"
        fi
    fi
    
    echo -e "${BLUE}[RViz]${NC} Config: ${rviz_config}"
    
    if [ -f "${rviz_config}" ]; then
        rviz2 -d "${rviz_config}" &
    else
        echo -e "${YELLOW}[RViz]${NC} Config not found, starting with default config"
        rviz2 &
    fi
    
    sleep 2
    echo -e "${GREEN}[RViz]${NC} Started"
}

run_nav2() {
    echo -e "${BLUE}[Nav2]${NC} Starting Nav2 navigation stack..."
    
    # [FIX] OpenMP conflict with Conda environment (SIGSEGV -11 fix)
    export OMP_NUM_THREADS=1
    
    # Select params file and use_sim_time based on data source mode
    local base_params_file
    local use_sim_time_val
    local nav2_params_file
    
    if [ "${DATA_SOURCE}" = "sim" ]; then
        base_params_file="${ROS2_WS}/install/nvblox_integration/share/nvblox_integration/config/sim_nav2.yaml"
        use_sim_time_val="true"
        if [ "${VERBOSE}" = "true" ]; then
            echo -e "${BLUE}[Nav2]${NC} Mode: Simulation (use_sim_time=true)"
        fi
    else
        base_params_file="${ROS2_WS}/install/nvblox_integration/share/nvblox_integration/config/dataset_nav2.yaml"
        use_sim_time_val="false"
        echo -e "${BLUE}[Nav2]${NC} Mode: Dataset (costmap testing only)"
    fi
    
    # Generate scaled params file if footprint scale != 1.0 or tolerance was customized or velocity changed
    local gen_script="${ROS2_WS}/src/nvblox_integration/scripts/generate_nav2_params.py"
    
    # Always generate custom params file to ensure velocity consistency
    nav2_params_file="/tmp/nav2_params_scaled.yaml"
    if [ "${VERBOSE}" = "true" ]; then
        echo -e "${BLUE}[Nav2]${NC} Generating navigation parameters..."
        echo -e "${BLUE}[Nav2]${NC}   Footprint scale: ${FOOTPRINT_SCALE}x"
        echo -e "${BLUE}[Nav2]${NC}   XY tolerance: ${XY_GOAL_TOLERANCE}m"
        echo -e "${BLUE}[Nav2]${NC}   Yaw tolerance: ${YAW_GOAL_TOLERANCE}rad"
        echo -e "${BLUE}[Nav2]${NC}   Max linear vel: ${MAX_LINEAR_VEL} m/s"
        echo -e "${BLUE}[Nav2]${NC}   Max angular vel: ${MAX_ANGULAR_VEL} rad/s"
    fi
    
    python3 "${gen_script}" \
        --input "${base_params_file}" \
        --output "${nav2_params_file}" \
        --footprint-scale "${FOOTPRINT_SCALE}" \
        --xy-tolerance "${XY_GOAL_TOLERANCE}" \
        --yaw-tolerance "${YAW_GOAL_TOLERANCE}" \
        --max-linear-vel "${MAX_LINEAR_VEL}" \
        --max-angular-vel "${MAX_ANGULAR_VEL}" \
        --linear-accel "${LINEAR_ACCEL}" \
        --angular-accel "${ANGULAR_ACCEL}" \
        --bt-xml "${ROS2_WS}/install/nvblox_integration/share/nvblox_integration/config/bt/navigate_w_slow_replanning.xml"
    
    [ "$VERBOSE" = true ] && echo -e "${BLUE}[Nav2]${NC} Params file: ${nav2_params_file}"
    
    ros2 launch nvblox_integration sim_nav2.launch.py \
        use_sim_time:="${use_sim_time_val}" \
        params_file:="${nav2_params_file}" &
    
    [ "$VERBOSE" = true ] && echo -e "${BLUE}[Nav2]${NC} Waiting for nodes to start..."
    sleep 8
    
    # Ensure all Nav2 lifecycle nodes are fully activated
    [ "$VERBOSE" = true ] && echo -e "${BLUE}[Nav2]${NC} Waiting for lifecycle nodes to be ready..."
    sleep 3
    
    # Required Nav2 lifecycle nodes
    local nav2_nodes=(
        "controller_server"
        "planner_server"
        "smoother_server"
        "behavior_server"
        "bt_navigator"
        "waypoint_follower"
    )
    
    # Retry up to 3 times to activate all nodes
    for attempt in 1 2 3; do
        echo -e "${BLUE}[Nav2]${NC} Lifecycle activation attempt ${attempt}/3..."
        local all_active=true
        
        for node in "${nav2_nodes[@]}"; do
            local state=$(ros2 lifecycle get /$node 2>/dev/null | grep -o 'unconfigured\|inactive\|active' | head -1)
            
            if [ "$state" = "unconfigured" ]; then
                echo -e "${YELLOW}[Nav2]${NC} Configuring $node..."
                ros2 lifecycle set /$node configure 2>/dev/null || true
                sleep 0.3
                state="inactive"
            fi
            
            if [ "$state" = "inactive" ]; then
                echo -e "${YELLOW}[Nav2]${NC} Activating $node..."
                ros2 lifecycle set /$node activate 2>/dev/null || true
                sleep 0.3
                all_active=false
            elif [ "$state" != "active" ]; then
                all_active=false
            fi
        done
        
        if [ "$all_active" = true ]; then
            echo -e "${GREEN}[Nav2]${NC} All lifecycle nodes are active"
            break
        fi
        
        sleep 2
    done
    
    # Final state verification
    echo -e "${BLUE}[Nav2]${NC} Final lifecycle states:"
    for node in "${nav2_nodes[@]}"; do
        local state=$(ros2 lifecycle get /$node 2>/dev/null | grep -o 'unconfigured\|inactive\|active' | head -1)
        if [ "$state" = "active" ]; then
            echo -e "  ${GREEN}✓${NC} $node: $state"
        else
            echo -e "  ${RED}✗${NC} $node: ${state:-not found}"
        fi
    done
    
    echo -e "${GREEN}[Nav2]${NC} Started"
    
    # Force-set goal checker params (override Nav2 defaults)
    # Workaround: yaw_goal_tolerance not properly loaded from params file
    [ "$VERBOSE" = true ] && echo -e "${BLUE}[Nav2]${NC} Setting goal checker parameters..."
    [ "$VERBOSE" = true ] && echo -e "${BLUE}[Nav2]${NC}   XY tolerance: ${XY_GOAL_TOLERANCE}m"
    [ "$VERBOSE" = true ] && echo -e "${BLUE}[Nav2]${NC}   Yaw tolerance: ${YAW_GOAL_TOLERANCE}rad"
    ros2 param set /controller_server general_goal_checker.xy_goal_tolerance "${XY_GOAL_TOLERANCE}" 2>/dev/null || true
    ros2 param set /controller_server general_goal_checker.yaw_goal_tolerance "${YAW_GOAL_TOLERANCE}" 2>/dev/null || true
    [ "$VERBOSE" = true ] && echo -e "${GREEN}[Nav2]${NC} Goal checker parameters set"
    
    # Start Goal Reached Stop Controller
    # Sends forced stop commands after goal reached to prevent drift
    if [ "${DISABLE_GOAL_STOP}" = "true" ]; then
        [ "$VERBOSE" = true ] && echo -e "${YELLOW}[Nav2]${NC} Goal reached stop controller DISABLED (--no-goal-stop)"
        [ "$VERBOSE" = true ] && echo -e "${YELLOW}[Nav2]${NC}   → Using Nav2 native stop behavior for debugging"
    else
        [ "$VERBOSE" = true ] && echo -e "${BLUE}[Nav2]${NC} Starting goal reached stop controller..."
        conda run -n "${DEPTH_CONDA_ENV}" --no-capture-output \
            ros2 run nvblox_integration goal_reached_stop.py --ros-args \
            -p stop_duration:=1.0 \
            -p cmd_vel_topic:=/cmd_vel \
            -p use_sim_time:="${use_sim_time_val}" &
        sleep 1
        [ "$VERBOSE" = true ] && echo -e "${GREEN}[Nav2]${NC} Goal reached stop controller started"
    fi
    
    # Start Initial Rotation Controller (cmd_vel mux approach)
    # Nav2 controller_server publishes to /nav2_cmd_vel, this node relays to /cmd_vel
    # On rotation: pause relay -> send rotation cmd -> resume relay after completion
    # Advantage: does not touch Nav2 lifecycle, so actions won't fail
    [ "$VERBOSE" = true ] && echo -e "${BLUE}[Nav2]${NC} Starting initial rotation controller (cmd_vel mux)..."
    # Use foundation_stereo_py312 conda environment for Python 3.12 compatibility
    conda run -n "${DEPTH_CONDA_ENV}" --no-capture-output \
        ros2 run nvblox_integration initial_rotation_controller.py --ros-args \
        -p max_angular_velocity:="${INIT_ROTATION_MAX_ANGULAR_VEL}" \
        -p min_angular_velocity:="${INIT_ROTATION_MIN_ANGULAR_VEL}" \
        -p angular_threshold:=0.3 \
        -p angular_tolerance:=0.1 \
        -p control_frequency:=20.0 \
        -p use_path_direction:=false \
        -p path_lookahead_distance:=1.5 \
        -p nav2_cmd_vel_topic:=/nav2_cmd_vel \
        -p cmd_vel_topic:=/cmd_vel \
        -p use_sim_time:="${use_sim_time_val}" \
        -p verbose:="${VERBOSE}" &
    sleep 1
    [ "$VERBOSE" = true ] && echo -e "${GREEN}[Nav2]${NC} Initial rotation controller started"
    
    # Start Goal Pose Z-Corrector (for RViz 2D Nav Goal)
    # Corrects z=0 goals from RViz to actual ground height
    # ground_height=-1 -> auto-detect from /tf_gt base_link z
    if [ "$VERBOSE" = true ]; then
        local gh_msg="${GROUND_HEIGHT}m"
        if [ "$(echo "${GROUND_HEIGHT} < 0" | bc)" -eq 1 ]; then
            gh_msg="auto-detect from /tf_gt"
        fi
        echo -e "${BLUE}[Nav2]${NC} Starting goal pose corrector (ground_height=${gh_msg})..."
    fi
    # Use foundation_stereo_py312 conda environment for Python 3.12 compatibility
    conda run -n "${DEPTH_CONDA_ENV}" --no-capture-output \
        ros2 run nvblox_integration goal_pose_corrector.py --ros-args \
        -p ground_height:="${GROUND_HEIGHT}" \
        -p input_topic:=/goal_pose_raw \
        -p output_topic:=/goal_pose \
        -p use_sim_time:="${use_sim_time_val}" &
    sleep 1
    [ "$VERBOSE" = true ] && echo -e "${GREEN}[Nav2]${NC} Goal pose corrector started" || true
}

run_goal_sender() {
    [ "$VERBOSE" = true ] && echo -e "${BLUE}[GoalSender]${NC} Starting goal sender..."
    
    # Check if goal_sender is the only component running (for foreground interactive mode)
    local run_foreground=false
    if [ "$RUN_STATIC_TF" = false ] && [ "$RUN_DATASET" = false ] && \
       [ "$RUN_SLAM" = false ] && [ "$RUN_DEPTH" = false ] && \
       [ "$RUN_NVBLOX" = false ] && [ "$RUN_NAV2" = false ] && \
       [ "$RUN_RVIZ" = false ]; then
        run_foreground=true
    fi
    
    if [ -n "${NAV_GOAL_X}" ] && [ -n "${NAV_GOAL_Y}" ]; then
        # Single goal mode (always background, exits after sending)
        NAV_GOAL_YAW=${NAV_GOAL_YAW:-0.0}
        echo -e "${BLUE}[GoalSender]${NC} Sending goal: x=${NAV_GOAL_X}, y=${NAV_GOAL_Y}, yaw=${NAV_GOAL_YAW}°"
        
        # Convert yaw from degrees to radians
        NAV_GOAL_YAW_RAD=$(echo "scale=6; ${NAV_GOAL_YAW} * 3.14159265359 / 180" | bc)
        
        # Use foundation_stereo_py312 conda environment for Python 3.12 compatibility
        conda run -n "${DEPTH_CONDA_ENV}" --no-capture-output \
            ros2 run nvblox_integration goal_sender.py \
            --ros-args \
            -p x:="${NAV_GOAL_X}" \
            -p y:="${NAV_GOAL_Y}" \
            -p yaw:="${NAV_GOAL_YAW_RAD}" \
            -p use_sim_time:=true &
        sleep 2
        echo -e "${GREEN}[GoalSender]${NC} Goal sent"
    elif [ "$run_foreground" = true ]; then
        # Interactive mode - foreground (when goal_sender is the only component)
        echo -e "${BLUE}[GoalSender]${NC} Running in foreground interactive mode"
        echo -e "${CYAN}═══════════════════════════════════════════════════════════════${NC}"
        # Use foundation_stereo_py312 conda environment for Python 3.12 compatibility
        conda run -n "${DEPTH_CONDA_ENV}" --no-capture-output \
            ros2 run nvblox_integration goal_sender.py \
            --ros-args \
            -p interactive:=true \
            -p use_sim_time:=true
        # Foreground mode - script ends when goal_sender exits
        exit 0
    else
        # Interactive mode - background (when running with other components)
        [ "$VERBOSE" = true ] && echo -e "${BLUE}[GoalSender]${NC} Starting in background mode"
        [ "$VERBOSE" = true ] && echo -e "${YELLOW}[GoalSender]${NC} Note: For interactive input, run in a separate terminal:"
        [ "$VERBOSE" = true ] && echo -e "${YELLOW}[GoalSender]${NC}   ./run_pipeline.sh --goal-sender"
        # Use foundation_stereo_py312 conda environment for Python 3.12 compatibility
        conda run -n "${DEPTH_CONDA_ENV}" --no-capture-output \
            ros2 run nvblox_integration goal_sender.py \
            --ros-args \
            -p interactive:=true \
            -p use_sim_time:=true &
        sleep 2
        [ "$VERBOSE" = true ] && echo -e "${GREEN}[GoalSender]${NC} Started (background)" || true
    fi
}

run_ground_truth() {
    [ "$VERBOSE" = true ] && echo -e "${BLUE}[GroundTruth]${NC} Starting ground truth point cloud publisher..."
    
    local pointcloud_file="/shared_data/dem/sim_pointcloud_latest.ply"
    local npy_file="/shared_data/dem/sim_pointcloud_latest.npy"
    
    # Check which file format is available
    local use_ply="true"
    if [ ! -f "${pointcloud_file}" ]; then
        if [ -f "${npy_file}" ]; then
            use_ply="false"
            [ "$VERBOSE" = true ] && echo -e "${BLUE}[GroundTruth]${NC} Using NPY format: ${npy_file}"
        else
            echo -e "${YELLOW}[GroundTruth]${NC} Warning: No ground truth file found"
            [ "$VERBOSE" = true ] && echo -e "${YELLOW}[GroundTruth]${NC} Expected: ${pointcloud_file} or ${npy_file}"
            [ "$VERBOSE" = true ] && echo -e "${YELLOW}[GroundTruth]${NC} Start simulation first to generate ground truth data"
            return
        fi
    else
        [ "$VERBOSE" = true ] && echo -e "${BLUE}[GroundTruth]${NC} Using PLY format: ${pointcloud_file}"
    fi
    
    # Ensure ROS_DOMAIN_ID is set for sim mode
    if [ "${DATA_SOURCE}" = "sim" ]; then
        export ROS_DOMAIN_ID=0
    else
        export ROS_DOMAIN_ID=1
    fi
    
    # Use foundation_stereo_py312 conda environment for Python 3.12 compatibility
    conda run -n "${DEPTH_CONDA_ENV}" --no-capture-output \
        ros2 run nvblox_integration ground_truth_publisher.py \
        --ros-args \
        -p pointcloud_file:="${pointcloud_file}" \
        -p npy_file:="${npy_file}" \
        -p use_ply:="${use_ply}" \
        -p frame_id:="${GLOBAL_FRAME}" \
        -p publish_rate:=1.0 \
        -p point_color_r:=0 \
        -p point_color_g:=255 \
        -p point_color_b:=0 \
        -p alpha:=128 \
        -p auto_reload:=true \
        -p z_offset:=0.0 &
    
    sleep 2
    [ "$VERBOSE" = true ] && echo -e "${GREEN}[GroundTruth]${NC} Started - publishing to /ground_truth/pointcloud" || true
}

run_pose_plot() {
    [ "$VERBOSE" = true ] && echo -e "${BLUE}[PosePlot]${NC} Starting pose comparison plot..."
    [ "$VERBOSE" = true ] && echo -e "${BLUE}[PosePlot]${NC} Comparing GT (${TF_GT_TOPIC}) vs SLAM estimate"
    
    # Ensure ROS_DOMAIN_ID is set for sim mode
    if [ "${DATA_SOURCE}" = "sim" ]; then
        export ROS_DOMAIN_ID=0
    else
        export ROS_DOMAIN_ID=1
    fi
    
    # Use foundation_stereo_py312 conda environment (same as depth node)
    conda run -n foundation_stereo_py312 --no-capture-output \
        python "${ROS2_WS}/install/nvblox_integration/lib/nvblox_integration/pose_comparison_plot.py" \
        --ros-args \
        -p tf_gt_topic:="${TF_GT_TOPIC}" \
        -p tf_gt_child_frame:="${TF_GT_CHILD_FRAME}" \
        -p camera_frame:="${CAMERA_FRAME}" \
        -p history_length:=500 &
    
    sleep 2
    [ "$VERBOSE" = true ] && echo -e "${GREEN}[PosePlot]${NC} Started" || true
}

run_metrics_logger() {
    [ "$VERBOSE" = true ] && echo -e "${BLUE}[Metrics]${NC} Starting navigation metrics logger..."
    
    local metrics_file="${ROS2_WS}/data/navigation_metrics.csv"
    [ "$VERBOSE" = true ] && echo -e "${BLUE}[Metrics]${NC} Output file: ${metrics_file}"
    
    # Determine use_sim_time from DATA_SOURCE (same logic as run_nav2)
    local metrics_use_sim_time="false"
    if [ "${DATA_SOURCE}" = "sim" ]; then
        export ROS_DOMAIN_ID=0
        metrics_use_sim_time="true"
    else
        export ROS_DOMAIN_ID=1
        metrics_use_sim_time="false"
    fi
    
    # Use foundation_stereo_py312 conda environment for Python 3.12 compatibility
    conda run -n "${DEPTH_CONDA_ENV}" --no-capture-output \
        ros2 run nvblox_integration navigation_metrics_logger.py \
        --ros-args \
        -p output_file:="${metrics_file}" \
        -p tf_gt_topic:="${TF_GT_TOPIC}" \
        -p tf_gt_child_frame:="${TF_GT_CHILD_FRAME}" \
        -p camera_frame:="${CAMERA_FRAME}" \
        -p use_sim_time:="${metrics_use_sim_time}" &
    
    sleep 1
    [ "$VERBOSE" = true ] && echo -e "${GREEN}[Metrics]${NC} Navigation metrics logger started" || true
}

list_nodes() {
    echo -e "${BLUE}[Info]${NC} Running ROS2 nodes:"
    ros2 node list 2>/dev/null || echo "No nodes running"
    echo ""
    echo -e "${BLUE}[Info]${NC} Active topics:"
    ros2 topic list 2>/dev/null | grep -E "slam|depth|nvblox|stereo|nav|cmd_vel|ground_truth" || echo "No relevant topics"
}

kill_nodes() {
    echo -e "${YELLOW}[Kill]${NC} Stopping all pipeline nodes..."
    
    # Kill specific nodes
    pkill -f "dataset_publisher" 2>/dev/null || true
    pkill -f "stereo_slam" 2>/dev/null || true
    pkill -f "depth_node" 2>/dev/null || true
    pkill -f "nvblox_node" 2>/dev/null || true
    pkill -f "static_transform_publisher" 2>/dev/null || true
    pkill -f "rviz2" 2>/dev/null || true
    pkill -f "rqt_image_view" 2>/dev/null || true
    pkill -f "goal_sender" 2>/dev/null || true
    pkill -f "ground_truth_publisher" 2>/dev/null || true
    pkill -9 -f "pose_comparison_plot.py" 2>/dev/null || true
    pkill -f "navigation_metrics_logger" 2>/dev/null || true
    
    # Kill Nav2 nodes
    pkill -f "bt_navigator" 2>/dev/null || true
    pkill -f "controller_server" 2>/dev/null || true
    pkill -f "planner_server" 2>/dev/null || true
    pkill -f "behavior_server" 2>/dev/null || true
    pkill -f "smoother_server" 2>/dev/null || true
    pkill -f "waypoint_follower" 2>/dev/null || true
    pkill -f "lifecycle_manager" 2>/dev/null || true
    
    # Kill any remaining ros2 processes from this pipeline
    pkill -f "orb_slam3_ros2" 2>/dev/null || true
    pkill -f "foundation_stereo_ros2" 2>/dev/null || true
    pkill -f "nvblox_integration" 2>/dev/null || true
    pkill -f "nav2" 2>/dev/null || true
    
    sleep 2
    echo -e "${GREEN}[Kill]${NC} All nodes stopped"
}

wait_for_exit() {
    echo ""
    if [ "$VERBOSE" = true ]; then
        echo -e "${CYAN}═══════════════════════════════════════════════════════════════${NC}"
        echo -e "${GREEN}Pipeline is running. Press Ctrl+C to stop all nodes.${NC}"
        echo -e "${CYAN}═══════════════════════════════════════════════════════════════${NC}"
    else
        echo -e "${GREEN}[Pipeline]${NC} Running. Press Ctrl+C to stop."
    fi
    echo ""
    
    # Cleanup function
    cleanup() {
        # Prevent multiple calls
        trap - INT TERM
        echo -e "\n${YELLOW}[Shutdown]${NC} Stopping pipeline..."
        kill_nodes
        echo -e "${GREEN}[Shutdown]${NC} Done."
        exit 0
    }
    
    # Wait for user interrupt
    trap cleanup INT TERM
    
    # Keep script running
    while true; do
        sleep 1
    done
}

# =============================================================================
# Parse Arguments
# =============================================================================

while [[ $# -gt 0 ]]; do
    case $1 in
        --all)
            RUN_ALL=true
            shift
            ;;
        --static-tf)
            RUN_STATIC_TF=true
            shift
            ;;
        --dataset)
            RUN_DATASET=true
            shift
            ;;
        --slam)
            RUN_SLAM=true
            shift
            ;;
        --depth)
            RUN_DEPTH=true
            shift
            ;;
        --nvblox)
            RUN_NVBLOX=true
            shift
            ;;
        --rviz)
            RUN_RVIZ=true
            shift
            ;;
        --nav2)
            RUN_NAV2=true
            shift
            ;;
        --footprint-scale)
            FOOTPRINT_SCALE="$2"
            shift 2
            ;;
        --xy-tolerance)
            XY_GOAL_TOLERANCE="$2"
            shift 2
            ;;
        --yaw-tolerance)
            YAW_GOAL_TOLERANCE="$2"
            shift 2
            ;;
        --ground-height)
            GROUND_HEIGHT="$2"
            shift 2
            ;;
        --no-goal-stop)
            DISABLE_GOAL_STOP=true
            shift
            ;;
        --max-linear-vel)
            MAX_LINEAR_VEL="$2"
            shift 2
            ;;
        --max-angular-vel)
            MAX_ANGULAR_VEL="$2"
            shift 2
            ;;
        --goal-sender)
            RUN_GOAL_SENDER=true
            shift
            ;;
        --ground-truth)
            RUN_GROUND_TRUTH=true
            shift
            ;;
        --pose-plot)
            RUN_POSE_PLOT=true
            shift
            ;;
        --metrics)
            RUN_METRICS=true
            shift
            ;;
        --nav-goal)
            RUN_GOAL_SENDER=true
            NAV_GOAL_X="$2"
            NAV_GOAL_Y="$3"
            if [[ "$4" != -* ]] && [[ -n "$4" ]]; then
                NAV_GOAL_YAW="$4"
                shift 4
            else
                NAV_GOAL_YAW="0.0"
                shift 3
            fi
            ;;
        --slam-viz)
            SLAM_VIZ=true
            shift
            ;;
        --no-slam-viz)
            SLAM_VIZ=false
            shift
            ;;
        --depth-viz)
            DEPTH_VIZ=true
            shift
            ;;
        --no-depth-viz)
            DEPTH_VIZ=false
            shift
            ;;
        --pc-viz)
            DEPTH_PC_VIZ=true
            shift
            ;;
        --no-pc-viz)
            DEPTH_PC_VIZ=false
            shift
            ;;
        --depth-filter)
            DEPTH_BILATERAL_FILTER=true
            shift
            ;;
        --no-depth-filter)
            DEPTH_BILATERAL_FILTER=false
            shift
            ;;
        --bilateral-d)
            DEPTH_BILATERAL_D="$2"
            shift 2
            ;;
        --bilateral-sigma)
            DEPTH_BILATERAL_SIGMA_COLOR="$2"
            DEPTH_BILATERAL_SIGMA_SPACE="$2"
            shift 2
            ;;
        --verbose)
            VERBOSE=true
            shift
            ;;
        --quiet)
            VERBOSE=false
            shift
            ;;
        --sim)
            DATA_SOURCE="sim"
            shift
            ;;
        --image-type)
            SIM_IMAGE_TYPE="$2"
            shift 2
            ;;
        --use-dataset)
            DATA_SOURCE="dataset"
            shift
            ;;
        --left-topic)
            # Override both dataset and sim topics to use custom topic
            DATASET_LEFT_TOPIC="$2"
            SIM_LEFT_TOPIC="$2"
            shift 2
            ;;
        --right-topic)
            # Override both dataset and sim topics to use custom topic
            DATASET_RIGHT_TOPIC="$2"
            SIM_RIGHT_TOPIC="$2"
            shift 2
            ;;
        --dataset-path)
            DATASET_PATH="$2"
            shift 2
            ;;
        --publish-rate)
            PUBLISH_RATE="$2"
            shift 2
            ;;
        --voxel-size)
            VOXEL_SIZE="$2"
            shift 2
            ;;
        --esdf-height)
            ESDF_SLICE_HEIGHT_RELATIVE="$2"
            shift 2
            ;;
        --esdf-min)
            ESDF_SLICE_MIN_HEIGHT_RELATIVE="$2"
            shift 2
            ;;
        --esdf-max)
            ESDF_SLICE_MAX_HEIGHT_RELATIVE="$2"
            shift 2
            ;;
        --rviz-config)
            RVIZ_CONFIG="$2"
            shift 2
            ;;
        --init-rover-pos)
            INIT_ROVER_X="$2"
            INIT_ROVER_Y="$3"
            INIT_ROVER_Z="$4"
            shift 4
            ;;
        --init-rover-quat)
            INIT_ROVER_QW="$2"
            INIT_ROVER_QX="$3"
            INIT_ROVER_QY="$4"
            INIT_ROVER_QZ="$5"
            shift 5
            ;;
        --camera-offset)
            CAMERA_OFFSET_X="$2"
            CAMERA_OFFSET_Y="$3"
            CAMERA_OFFSET_Z="$4"
            shift 4
            ;;
        --camera-tilt)
            CAMERA_TILT_DEG="$2"
            shift 2
            ;;
        --use-param-init)
            USE_PARAM_INIT_POSE="true"
            USE_TF_GT_INIT="false"
            EXPLICIT_TF_GT_DISABLED="true"
            shift
            ;;
        --use-gt-init)
            USE_PARAM_INIT_POSE="false"
            USE_TF_GT_INIT="false"
            EXPLICIT_TF_GT_DISABLED="true"
            shift
            ;;
        --use-tf-gt-init)
            USE_TF_GT_INIT="true"
            shift
            ;;
        --no-tf-gt-init)
            USE_TF_GT_INIT="false"
            EXPLICIT_TF_GT_DISABLED="true"
            shift
            ;;
        --tf-gt-topic)
            TF_GT_TOPIC="$2"
            shift 2
            ;;
        --tf-gt-timeout)
            TF_GT_TIMEOUT="$2"
            shift 2
            ;;
        --ground-offset)
            BASE_LINK_GROUND_OFFSET="$2"
            shift 2
            ;;
        --list)
            setup_environment
            list_nodes
            exit 0
            ;;
        --kill)
            kill_nodes
            exit 0
            ;;
        -h|--help)
            print_help
            exit 0
            ;;
        *)
            echo -e "${RED}Unknown option: $1${NC}"
            print_help
            exit 1
            ;;
    esac
done

# =============================================================================
# Main Execution
# =============================================================================

print_banner

# If --all is specified, enable all components
if [ "$RUN_ALL" = true ]; then
    RUN_STATIC_TF=true
    RUN_DATASET=true
    RUN_SLAM=true
    RUN_DEPTH=true
    RUN_NVBLOX=true
    RUN_NAV2=true
    RUN_GROUND_TRUTH=true
    RUN_METRICS=true
    RUN_RVIZ=true
fi

# If sim mode is selected, disable dataset publisher (data comes from sim)
if [ "${DATA_SOURCE}" = "sim" ]; then
    if [ "$RUN_DATASET" = true ]; then
        echo -e "${YELLOW}[Info]${NC} Sim mode enabled - disabling dataset publisher (subscribing to Isaac Sim instead)"
        RUN_DATASET=false
    fi
fi

# Warn if nav2 is enabled without sim mode
if [ "$RUN_NAV2" = true ] && [ "${DATA_SOURCE}" != "sim" ]; then
    echo -e "${YELLOW}[Info]${NC} Nav2 in dataset mode: costmap testing only (robot won't move)."
fi

# Check if any component is selected
if [ "$RUN_STATIC_TF" = false ] && [ "$RUN_DATASET" = false ] && \
   [ "$RUN_SLAM" = false ] && [ "$RUN_DEPTH" = false ] && \
   [ "$RUN_NVBLOX" = false ] && [ "$RUN_RVIZ" = false ] && \
   [ "$RUN_NAV2" = false ] && [ "$RUN_GOAL_SENDER" = false ] && \
   [ "$RUN_GROUND_TRUTH" = false ] && [ "$RUN_POSE_PLOT" = false ] && 
   [ "$RUN_METRICS" = false ]; then
    echo -e "${YELLOW}No components selected. Use --help for usage information.${NC}"
    echo ""
    print_help
    exit 1
fi

# Print configuration summary
echo -e "${CYAN}Configuration:${NC}"
echo -e "  Data source: ${DATA_SOURCE}"
if [ "${DATA_SOURCE}" = "dataset" ]; then
    echo -e "  Dataset:    ${DATASET_PATH}"
    echo -e "  Rate:       ${PUBLISH_RATE} Hz"
fi
echo -e "  Voxel:      ${VOXEL_SIZE}m"
echo -e "  SLAM viz:   ${SLAM_VIZ}"
echo -e "  Depth viz:  ${DEPTH_VIZ}"
if [ "$RUN_NAV2" = true ]; then
    echo -e "  Nav2:       enabled"
fi
echo ""

echo -e "${CYAN}Components to run:${NC}"
[ "$RUN_STATIC_TF" = true ] && echo -e "  ✓ Static TF"
[ "$RUN_DATASET" = true ]   && echo -e "  ✓ Dataset Publisher"
[ "${DATA_SOURCE}" = "sim" ] && echo -e "  ✓ Isaac Sim Subscriber (external)"
[ "$RUN_SLAM" = true ]      && echo -e "  ✓ ORB-SLAM3"
[ "$RUN_DEPTH" = true ]     && echo -e "  ✓ FoundationStereo Depth"
[ "$RUN_NVBLOX" = true ]    && echo -e "  ✓ nvblox"
[ "$RUN_NAV2" = true ]      && echo -e "  ✓ Nav2 Navigation Stack"
[ "$RUN_GOAL_SENDER" = true ] && echo -e "  ✓ Goal Sender"
[ "$RUN_GROUND_TRUTH" = true ] && echo -e "  ✓ Ground Truth Publisher"
[ "$RUN_POSE_PLOT" = true ] && echo -e "  ✓ Pose Comparison Plot"
[ "$RUN_METRICS" = true ]   && echo -e "  ✓ Navigation Metrics Logger"
[ "$RUN_RVIZ" = true ]      && echo -e "  ✓ RViz2"
echo ""

# Setup environment
setup_environment

# Configure topics based on data source mode
configure_topics

# Run components in order
[ "$RUN_STATIC_TF" = true ] && run_static_tf
[ "$RUN_DATASET" = true ]   && run_dataset_publisher
[ "$RUN_SLAM" = true ]      && run_slam
[ "$RUN_DEPTH" = true ]     && run_depth
[ "$RUN_NVBLOX" = true ]    && run_nvblox
[ "$RUN_NAV2" = true ]      && run_nav2
[ "$RUN_GOAL_SENDER" = true ] && run_goal_sender
[ "$RUN_METRICS" = true ] && run_metrics_logger
[ "$RUN_GROUND_TRUTH" = true ] && run_ground_truth
[ "$RUN_POSE_PLOT" = true ] && run_pose_plot
[ "$RUN_RVIZ" = true ]      && run_rviz

# Wait for user to stop
wait_for_exit
