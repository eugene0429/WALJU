#!/bin/bash
# =============================================================================
# kill_pipeline.sh - ROS2 파이프라인 전체 프로세스 종료 스크립트
# =============================================================================

set -e

echo "=============================================="
echo "  ROS2 Pipeline Cleanup Script"
echo "=============================================="

# Color codes for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Function to kill processes by pattern
kill_by_pattern() {
    local pattern=$1
    local description=$2
    local pids=$(pgrep -f "$pattern" 2>/dev/null || true)
    
    if [ -n "$pids" ]; then
        echo -e "${YELLOW}Killing $description...${NC}"
        pkill -9 -f "$pattern" 2>/dev/null || true
        sleep 0.5
        echo -e "${GREEN}  ✓ $description terminated${NC}"
    fi
}

# Function to count remaining processes
count_processes() {
    local pattern=$1
    pgrep -f "$pattern" 2>/dev/null | wc -l || echo "0"
}

echo ""
echo "Step 1: Killing Nav2 processes..."
echo "----------------------------------------------"
kill_by_pattern "nav2" "Nav2 nodes"
kill_by_pattern "controller_server" "Controller Server"
kill_by_pattern "planner_server" "Planner Server"
kill_by_pattern "behavior_server" "Behavior Server"
kill_by_pattern "bt_navigator" "BT Navigator"
kill_by_pattern "waypoint_follower" "Waypoint Follower"
kill_by_pattern "lifecycle_manager" "Lifecycle Manager"
kill_by_pattern "map_server" "Map Server"
kill_by_pattern "amcl" "AMCL"
kill_by_pattern "costmap" "Costmap nodes"

echo ""
echo "Step 2: Killing SLAM processes..."
echo "----------------------------------------------"
kill_by_pattern "orb_slam" "ORB-SLAM3"
kill_by_pattern "slam_toolbox" "SLAM Toolbox"
kill_by_pattern "cartographer" "Cartographer"

echo ""
echo "Step 3: Killing nvblox processes..."
echo "----------------------------------------------"
kill_by_pattern "nvblox" "nvblox nodes"
kill_by_pattern "nvblox_node" "nvblox_node"
kill_by_pattern "nvblox_human" "nvblox_human"

echo ""
echo "Step 4: Killing depth/stereo processing..."
echo "----------------------------------------------"
kill_by_pattern "foundation_stereo" "Foundation Stereo"
kill_by_pattern "depth" "Depth processors"
kill_by_pattern "stereo" "Stereo processors"

echo ""
echo "Step 5: Killing visualization tools..."
echo "----------------------------------------------"
kill_by_pattern "rviz" "RViz2"
kill_by_pattern "foxglove" "Foxglove"
kill_by_pattern "rqt" "rqt tools"

echo ""
echo "Step 6: Killing custom nodes..."
echo "----------------------------------------------"
kill_by_pattern "initial_rotation_controller" "Initial Rotation Controller"
kill_by_pattern "nvblox_integration" "nvblox_integration nodes"
kill_by_pattern "pose_comparison_plot.py" "Pose Comparison Plot"

echo ""
echo "Step 7: Killing ROS2 daemon and launch..."
echo "----------------------------------------------"
kill_by_pattern "ros2.*launch" "ros2 launch processes"
kill_by_pattern "ros2.*run" "ros2 run processes"

# Kill ros2 daemon (will auto-restart if needed)
echo -e "${YELLOW}Stopping ROS2 daemon...${NC}"
ros2 daemon stop 2>/dev/null || true
sleep 1

echo ""
echo "Step 8: Killing remaining ROS2 processes..."
echo "----------------------------------------------"
kill_by_pattern "_ros2_daemon" "ROS2 daemon"
kill_by_pattern "robot_state_publisher" "Robot State Publisher"
kill_by_pattern "joint_state_publisher" "Joint State Publisher"
kill_by_pattern "tf2" "TF2 nodes"
kill_by_pattern "static_transform_publisher" "Static Transform Publisher"

echo ""
echo "Step 9: Killing simulation-related processes..."
echo "----------------------------------------------"
kill_by_pattern "gzserver" "Gazebo Server"
kill_by_pattern "gzclient" "Gazebo Client"
kill_by_pattern "gazebo" "Gazebo"
kill_by_pattern "isaac" "Isaac Sim bridge"

echo ""
echo "Step 10: Final cleanup..."
echo "----------------------------------------------"
# Kill any remaining Python ROS2 nodes
kill_by_pattern "python.*ros2" "Python ROS2 processes"
kill_by_pattern "python3.*ros2" "Python3 ROS2 processes"

# Kill component containers
kill_by_pattern "component_container" "Component Containers"

sleep 1

echo ""
echo "=============================================="
echo "  Verification"
echo "=============================================="

# Check for remaining processes
remaining_nav2=$(count_processes "nav2|controller_server|planner_server|behavior_server")
remaining_nvblox=$(count_processes "nvblox")
remaining_slam=$(count_processes "orb_slam|slam_toolbox")
remaining_rviz=$(count_processes "rviz")

if [ "$remaining_nav2" -gt 0 ] || [ "$remaining_nvblox" -gt 0 ] || [ "$remaining_slam" -gt 0 ]; then
    echo -e "${RED}Warning: Some processes may still be running${NC}"
    echo "  Nav2 related: $remaining_nav2"
    echo "  nvblox related: $remaining_nvblox"
    echo "  SLAM related: $remaining_slam"
    echo "  RViz related: $remaining_rviz"
    echo ""
    echo "Run this script again or manually check with:"
    echo "  ps aux | grep -E 'ros2|nav2|nvblox|slam|rviz'"
else
    echo -e "${GREEN}✓ All pipeline processes terminated successfully${NC}"
fi

echo ""
echo "=============================================="
echo "  Cleanup Complete"
echo "=============================================="
echo ""
echo "To restart the pipeline, run:"
echo "  ./run_pipeline.sh"
echo ""
