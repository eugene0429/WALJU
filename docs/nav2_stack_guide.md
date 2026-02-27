# Nav2 Navigation Stack Guide

## 1. Overview
This document describes the navigation stack setup used in the `nvblox_integration` package. It utilizes the ROS2 Navigation2 ("Nav2") framework, integrating `nvblox` seamlessly for mapless or map-based navigation using 2D slices of the 3D ESDF (Euclidean Signed Distance Field).

## 2. Nav2 Configuration

The configuration is defined in [`src/nvblox_integration/config/sim_nav2.yaml`](../src/nvblox_integration/config/sim_nav2.yaml).

### 2.1 Planner Server
The pipeline uses **SmacPlanner2D** (`nav2_smac_planner::SmacPlanner2D`), which provides A* planning with support for cost-aware routing.

- **Plugin Name**: `GridBased`
- **Algorithm**: A* hybrid state lattice or 2D grid search (configured as 2D).
- **Key Parameters**:
  - `tolerance` (0.5m): The tolerance for the goal point.
  - `allow_unknown` (true): Allows planning through unknown space, essential for exploration.
  - `max_planning_time` (5.0s): Generous timeout for complex environments.
  - `cost_travel_multiplier` (5.0): High penalty for moving near obstacles to prefer safer paths.
  - `lethal_cost` (252): Threshold for "untraversable".
  - `allow_reverse_expansion`: `false` (Robot drives forward).

### 2.2 Controller Server
The local trajectory planner is **Regulated Pure Pursuit (RPP)** (`nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController`). This controller is well-suited for path following with curvature constraints.

- **Plugin Name**: `FollowPath`
- **Key Parameters**:
  - **Velocities**:
    - `desired_linear_vel`: 0.5 m/s
    - `max_linear_accel`: 2.0 m/s²
    - `max_angular_vel`: 1.0 rad/s
  - **Lookahead**:
    - `lookahead_dist`: 2.0m (Adaptive range: 0.5m - 2.5m)
    - `use_velocity_scaled_lookahead_dist`: `true` (Lookahead scales with speed).
  - **Regulation**:
    - `use_regulated_linear_velocity_scaling`: `true` (Slows down on sharp curves).
    - `use_cost_regulated_linear_velocity_scaling`: `true` (Slows down when near obstacles).
  - **Rotation**:
    - `use_rotate_to_heading`: `false` (Handled by custom `initial_rotation_controller`).
    - `allow_reversing`: `false`.

### 2.3 Costmaps
Both global and local costmaps rely on the **NvbloxCostmapLayer** instead of traditional laser scan layers.

**Costmap Calculation Flow**:
1.  **Source**: `nvblox_node` calculates the 3D ESDF and publishes a 2D cross-section to `/nvblox_node/static_map_slice`.
2.  **Processing**: The `NvbloxCostmapLayer` (inside Nav2) subscribes to this slice, converts distance values to cost values (0-254), and applies the configured gradient.
3.  **Inflation**: The `InflationLayer` expands these lethal costs to create safety margins.
4.  **Output**: The final grid is used internally for planning and published to `/_costmap/costmap` for visualization.

- **Global Costmap**: `global_costmap/Costmap2D`
  - **Update/Publish Frequency**: 1.0 Hz (Reduced for GPU load management).
  - **Plugins**:
    - `nvblox_layer`: Subscribes to `/nvblox_node/static_map_slice`.
      - `max_obstacle_distance`: 2.5m (Gradient applied up to this distance).
      - `inflation_distance`: 0.8m (Safety margin).
    - `inflation_layer`: `cost_scaling_factor`: 0.5 (Conservative inflation).
  
- **Local Costmap**: `local_costmap/Costmap2D`
  - **Update/Publish Frequency**: 2.0 Hz.
  - **Rolling Window**: `true` (Centered on robot).
  - **Plugins**:
    - `nvblox_layer`: Subscribes to `/nvblox_node/static_map_slice`.
    - `inflation_layer`: `cost_scaling_factor`: 1.0.

### 2.4 Behavior Trees
- **Main Tree**: `navigate_w_slow_replanning.xml`
  - Uses a custom generic BT to control replanning frequency (0.2Hz / every 5s) to prevent "planning jitter" where the path constantly shifts.
- **Path Replanning Threshold**: 1.0m (Only replan if the robot deviates significantly).

---

## 3. Custom Nodes Implementation
To address specific behaviors required for the rover platform, several custom Python nodes are implemented in `nvblox_integration`.

### 3.1 Initial Rotation Controller
**File**: `initial_rotation_controller.py`
To fix the `RegulatedPurePursuit` controller's potential inability to handle large initial heading errors or turn-in-place requirements efficiently.

- **Mechanism**:
  - Intercepts `/nav2_cmd_vel` (output from RPP).
  - Publishes `/cmd_vel` (actual robot input).
- **Behavior**:
  - When a new goal is received, it halts the RPP output.
  - Performs a pure rotation until the robot faces the path.
  - Releases control back to Nav2 once the heading error is within tolerance.

### 3.2 Goal Reached Stop Controller
**File**: `goal_reached_stop.py`
Ensures the robot comes to a complete halt upon reaching the destination.

- **Behavior**: Monitors the action server status and publishes zero-velocity Twist messages to `/cmd_vel` for a fixed duration upon success.

---

## 4. Topics Summary

### 4.1 Subscribed Topics (Input)
The Nav2 stack in this configuration relies heavily on the `NvbloxCostmapLayer` and does not subscribe to standard 2D LaserScans or PointClouds directly in the costmap plugins.

| Topic | Type | Node / Plugin | Description |
| :--- | :--- | :--- | :--- |
| `/odom` | `nav_msgs/msg/Odometry` | `controller_server`, `bt_navigator`, `velocity_smoother` | Robot odometry for localization and velocity smoothing feedback. |
| `/nvblox_node/static_map_slice` | `nvblox_msgs/msg/DistanceMapSlice` | `NvbloxCostmapLayer` (Local & Global) | 2D slice of the ESDF map from Nvblox, used as the primary observation source for obstacle avoidance. |
| `/goal_pose` | `geometry_msgs/msg/PoseStamped` | `bt_navigator` | Destination for the robot (via Action Server). |
| `/tf`, `/tf_static` | `tf2_msgs/msg/TFMessage` | All | Coordinate frame transformations. |

> **Note**: The `global_costmap` and `local_costmap` are configured to use the `NvbloxCostmapLayer`. They do **not** use `ObstacleLayer` or `VoxelLayer` with `/scan` or `/points` inputs. The obstacle information comes solely from the Nvblox map slice.

### 4.2 Published Topics (Output)
| Topic | Type | Node | Description |
| :--- | :--- | :--- | :--- |
| `/nav2_cmd_vel` | `geometry_msgs/msg/Twist` | `controller_server` | Raw velocity command from RPP Controller. This is consumed by `initial_rotation_controller`. |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | `initial_rotation_controller` | **Final command** sent to the robot base. |
| `/plan` | `nav_msgs/msg/Path` | `planner_server` | The global path computed by SmacPlanner. |
| `/local_plan` | `nav_msgs/msg/Path` | `controller_server` | The local lookahead trajectory from RPP. |
| `/global_costmap/costmap` | `nav_msgs/msg/OccupancyGrid` | `global_costmap` | Visual representation of the global costmap cost values. |
| `/local_costmap/costmap` | `nav_msgs/msg/OccupancyGrid` | `local_costmap` | Visual representation of the local costmap cost values. |

### 4.3 Crucial Remappings
Based on `sim_nav2.launch.py` and `sim_nav2.yaml`:
1.  **Controller Output**: `controller_server` / `cmd_vel` → `/nav2_cmd_vel`.
2.  **Velocity Smoother**: This node receives `/odom` but its output is remapped to `cmd_vel` by the mux logic if enabled (in this config, `closed_loop: False`).
3.  **Mux Logic**: The `initial_rotation_controller` acts as the final gatekeeper for `/cmd_vel`.
