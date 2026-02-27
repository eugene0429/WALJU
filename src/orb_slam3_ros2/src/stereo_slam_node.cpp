/**
 * Stereo SLAM Node for ORB-SLAM3 ROS2
 * 
 * Subscribes to stereo image pairs, runs ORB-SLAM3, and publishes pose.
 * Coordinate transformation logic ported from stereo_custom.cc
 * 
 * Subscribed Topics:
 *   /stereo/left/image_raw  (sensor_msgs/Image)
 *   /stereo/right/image_raw (sensor_msgs/Image)
 * 
 * Published Topics:
 *   /slam/pose (geometry_msgs/PoseStamped)
 *   /slam/odom (nav_msgs/Odometry)
 *   /tf        (tf2_msgs/TFMessage) - map -> camera_link
 * 
 * Parameters:
 *   vocabulary_path: Path to ORB vocabulary file
 *   settings_path: Path to camera settings YAML
 *   left_topic: Left image topic name
 *   right_topic: Right image topic name
 *   camera_tilt_deg: Camera tilt angle in degrees
 *   enable_viewer: Enable ORB-SLAM3 Pangolin viewer
 *   output_frame_id: Output frame ID (default: "map")
 *   camera_frame_id: Camera frame ID (default: "camera_link")
 */

#include <chrono>
#include <fstream>
#include <sstream>
#include <cmath>
#include <mutex>
#include <future>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include <opencv2/core/core.hpp>

// ORB-SLAM3
#include "System.h"
#include "Frame.h"
#include "Tracking.h"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sophus/se3.hpp>

using namespace std::chrono_literals;

class StereoSlamNode : public rclcpp::Node
{
public:
    StereoSlamNode() : Node("stereo_slam_node"), bFirstValidPose_(false)
    {
        // Declare parameters
        this->declare_parameter<std::string>("vocabulary_path", "");
        this->declare_parameter<std::string>("settings_path", "");
        this->declare_parameter<std::string>("left_topic", "/stereo/left/image_raw");
        this->declare_parameter<std::string>("right_topic", "/stereo/right/image_raw");
        // Camera tilt angle: 0 = horizontal, + = looking down
        this->declare_parameter<double>("camera_tilt_deg", 19.923);
        this->declare_parameter<bool>("enable_viewer", true);
        this->declare_parameter<std::string>("output_frame_id", "map");
        this->declare_parameter<std::string>("camera_frame_id", "camera_link");
        this->declare_parameter<std::string>("gt_file_path", "");
        this->declare_parameter<std::string>("output_trajectory_file", "");
        // Timestamp mode: "original" uses image timestamp, "current" uses current time
        this->declare_parameter<std::string>("timestamp_mode", "current");
        // Pose mode: "world" for visualization, "camera" for nvblox (optical frame convention)
        this->declare_parameter<std::string>("pose_mode", "camera");
        
        // ============================================
        // Initial Pose Parameters (World Frame: X-forward, Y-left, Z-up)
        // ============================================
        // Initial rover position in world frame
        this->declare_parameter<double>("init_rover_x", 0.0);
        this->declare_parameter<double>("init_rover_y", 0.0);
        this->declare_parameter<double>("init_rover_z", 0.0);
        // Initial rover orientation (quaternion, world frame)
        this->declare_parameter<double>("init_rover_qw", 1.0);
        this->declare_parameter<double>("init_rover_qx", 0.0);
        this->declare_parameter<double>("init_rover_qy", 0.0);
        this->declare_parameter<double>("init_rover_qz", 0.0);
        // Camera position relative to rover body (in rover body frame: X-forward, Y-left, Z-up)
        this->declare_parameter<double>("camera_offset_x", 1.0);     // 1m forward
        this->declare_parameter<double>("camera_offset_y", -0.155);  // 0.155m right (negative Y)
        this->declare_parameter<double>("camera_offset_z", 1.5);     // 1.5m up
        // Camera orientation relative to rover body (quaternion from /tf: base_link -> camera)
        // If provided, these override camera_tilt_deg for computing R_cam_to_world
        this->declare_parameter<double>("camera_qw", 1.0);
        this->declare_parameter<double>("camera_qx", 0.0);
        this->declare_parameter<double>("camera_qy", 0.0);
        this->declare_parameter<double>("camera_qz", 0.0);
        this->declare_parameter<bool>("use_camera_quat", false);  // Use quaternion instead of tilt angle
        // Use parameters for init pose (true) or read from gt_file_path (false)
        this->declare_parameter<bool>("use_param_init_pose", false);
        // Use /tf_gt topic for init pose (sim mode)
        this->declare_parameter<bool>("use_tf_gt_init", false);
        this->declare_parameter<std::string>("tf_gt_topic", "/tf_gt");
        this->declare_parameter<std::string>("tf_gt_child_frame", "base_link");
        this->declare_parameter<double>("tf_gt_timeout", 10.0);
        // QoS setting: use BEST_EFFORT for sim, RELIABLE for dataset
        this->declare_parameter<bool>("use_best_effort", false);
        // Verbose logging: if false, suppress periodic frame count logs
        this->declare_parameter<bool>("verbose", false);

        // Get parameters
        vocabulary_path_ = this->get_parameter("vocabulary_path").as_string();
        settings_path_ = this->get_parameter("settings_path").as_string();
        left_topic_ = this->get_parameter("left_topic").as_string();
        right_topic_ = this->get_parameter("right_topic").as_string();
        camera_tilt_deg_ = this->get_parameter("camera_tilt_deg").as_double();
        enable_viewer_ = this->get_parameter("enable_viewer").as_bool();
        output_frame_id_ = this->get_parameter("output_frame_id").as_string();
        camera_frame_id_ = this->get_parameter("camera_frame_id").as_string();
        gt_file_path_ = this->get_parameter("gt_file_path").as_string();
        output_trajectory_file_ = this->get_parameter("output_trajectory_file").as_string();
        timestamp_mode_ = this->get_parameter("timestamp_mode").as_string();
        pose_mode_ = this->get_parameter("pose_mode").as_string();
        camera_offset_x_ = this->get_parameter("camera_offset_x").as_double();
        camera_offset_y_ = this->get_parameter("camera_offset_y").as_double();
        camera_offset_z_ = this->get_parameter("camera_offset_z").as_double();
        camera_qw_ = this->get_parameter("camera_qw").as_double();
        camera_qx_ = this->get_parameter("camera_qx").as_double();
        camera_qy_ = this->get_parameter("camera_qy").as_double();
        camera_qz_ = this->get_parameter("camera_qz").as_double();
        use_camera_quat_ = this->get_parameter("use_camera_quat").as_bool();
        use_param_init_pose_ = this->get_parameter("use_param_init_pose").as_bool();
        use_tf_gt_init_ = this->get_parameter("use_tf_gt_init").as_bool();
        tf_gt_topic_ = this->get_parameter("tf_gt_topic").as_string();
        tf_gt_child_frame_ = this->get_parameter("tf_gt_child_frame").as_string();
        tf_gt_timeout_ = this->get_parameter("tf_gt_timeout").as_double();
        use_best_effort_ = this->get_parameter("use_best_effort").as_bool();
        verbose_ = this->get_parameter("verbose").as_bool();
        
        // Get initial pose parameters
        init_rover_x_ = this->get_parameter("init_rover_x").as_double();
        init_rover_y_ = this->get_parameter("init_rover_y").as_double();
        init_rover_z_ = this->get_parameter("init_rover_z").as_double();
        init_rover_qw_ = this->get_parameter("init_rover_qw").as_double();
        init_rover_qx_ = this->get_parameter("init_rover_qx").as_double();
        init_rover_qy_ = this->get_parameter("init_rover_qy").as_double();
        init_rover_qz_ = this->get_parameter("init_rover_qz").as_double();

        // Validate required parameters
        if (vocabulary_path_.empty() || settings_path_.empty())
        {
            RCLCPP_ERROR(this->get_logger(), 
                "vocabulary_path and settings_path parameters are required!");
            rclcpp::shutdown();
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Vocabulary: %s", vocabulary_path_.c_str());
        RCLCPP_INFO(this->get_logger(), "Settings: %s", settings_path_.c_str());
        RCLCPP_INFO(this->get_logger(), "Left topic: %s", left_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "Right topic: %s", right_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "Camera tilt: %.3f deg", camera_tilt_deg_);
        RCLCPP_INFO(this->get_logger(), "Timestamp mode: %s", timestamp_mode_.c_str());
        RCLCPP_INFO(this->get_logger(), "Pose mode: %s", pose_mode_.c_str());
        RCLCPP_INFO(this->get_logger(), "Camera offset (body frame): [%.3f, %.3f, %.3f]",
            camera_offset_x_, camera_offset_y_, camera_offset_z_);

        // Initialize coordinate transformation matrix
        InitializeCoordinateTransform();

        // Initialize pose from tf_gt topic, parameters, or GT file
        if (use_tf_gt_init_)
        {
            // Wait for /tf_gt topic to get initial pose
            RCLCPP_INFO(this->get_logger(), "Waiting for initial pose from %s topic...", tf_gt_topic_.c_str());
            hasGTInit_ = InitializePoseFromTfGt();
            if (hasGTInit_)
            {
                RCLCPP_INFO(this->get_logger(), "Initialized pose from %s:", tf_gt_topic_.c_str());
                RCLCPP_INFO(this->get_logger(), "  Rover position: [%.3f, %.3f, %.3f]",
                    t_gt_rover_(0), t_gt_rover_(1), t_gt_rover_(2));
                RCLCPP_INFO(this->get_logger(), "  Rover orientation (qw,qx,qy,qz): [%.4f, %.4f, %.4f, %.4f]",
                    q_gt_rover_.w(), q_gt_rover_.x(), q_gt_rover_.y(), q_gt_rover_.z());
                RCLCPP_INFO(this->get_logger(), "  Camera position: [%.3f, %.3f, %.3f]",
                    t_gt_init_(0), t_gt_init_(1), t_gt_init_(2));
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Failed to get initial pose from %s, falling back to parameters", tf_gt_topic_.c_str());
                hasGTInit_ = InitializePoseFromParams();
            }
        }
        else if (use_param_init_pose_)
        {
            // Use parameter values for initial pose
            hasGTInit_ = InitializePoseFromParams();
            if (hasGTInit_)
            {
                RCLCPP_INFO(this->get_logger(), "Initialized pose from parameters:");
                RCLCPP_INFO(this->get_logger(), "  Rover position: [%.3f, %.3f, %.3f]",
                    t_gt_rover_(0), t_gt_rover_(1), t_gt_rover_(2));
                RCLCPP_INFO(this->get_logger(), "  Rover orientation (qw,qx,qy,qz): [%.4f, %.4f, %.4f, %.4f]",
                    q_gt_rover_.w(), q_gt_rover_.x(), q_gt_rover_.y(), q_gt_rover_.z());
                RCLCPP_INFO(this->get_logger(), "  Camera position: [%.3f, %.3f, %.3f]",
                    t_gt_init_(0), t_gt_init_(1), t_gt_init_(2));
            }
        }
        else if (!gt_file_path_.empty())
        {
            // Load from GT file
            hasGTInit_ = LoadInitialPoseFromGT(gt_file_path_);
            if (hasGTInit_)
            {
                RCLCPP_INFO(this->get_logger(), "Loaded GT initial pose from: %s", 
                    gt_file_path_.c_str());
                RCLCPP_INFO(this->get_logger(), "  Rover position: [%.3f, %.3f, %.3f]",
                    t_gt_rover_(0), t_gt_rover_(1), t_gt_rover_(2));
                RCLCPP_INFO(this->get_logger(), "  Camera position: [%.3f, %.3f, %.3f]",
                    t_gt_init_(0), t_gt_init_(1), t_gt_init_(2));
            }
        }

        // Open output trajectory file if specified
        if (!output_trajectory_file_.empty())
        {
            trajectory_file_.open(output_trajectory_file_);
            if (trajectory_file_.is_open())
            {
                trajectory_file_ << std::fixed << std::setprecision(9);
                RCLCPP_INFO(this->get_logger(), "Saving trajectory to: %s", 
                    output_trajectory_file_.c_str());
            }
        }

        // Create ORB-SLAM3 system
        RCLCPP_INFO(this->get_logger(), "Initializing ORB-SLAM3...");
        mpSLAM_ = std::make_unique<ORB_SLAM3::System>(
            vocabulary_path_, 
            settings_path_, 
            ORB_SLAM3::System::STEREO, 
            enable_viewer_);
        
        imageScale_ = mpSLAM_->GetImageScale();
        RCLCPP_INFO(this->get_logger(), "ORB-SLAM3 initialized. Image scale: %.2f", imageScale_);

        // Create publishers
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/slam/pose", 10);
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
            "/slam/odom", 10);
        
        // TF broadcaster
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // Create synchronized subscribers using message_filters
        rclcpp::QoS image_qos = use_best_effort_ ? rclcpp::SensorDataQoS() : rclcpp::QoS(10);
        left_sub_.subscribe(this, left_topic_, image_qos.get_rmw_qos_profile());
        right_sub_.subscribe(this, right_topic_, image_qos.get_rmw_qos_profile());

        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(30), left_sub_, right_sub_);
        
        // Set max age (how long to wait for matching messages) - critical for slow topics
        // For ~1-2Hz topics with ~0.5s timestamp offset between left/right
        sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(2.0));
        
        sync_->registerCallback(
            std::bind(&StereoSlamNode::GrabStereo, this, 
                std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "Stereo SLAM node started!");
    }

    ~StereoSlamNode()
    {
        if (mpSLAM_)
        {
            RCLCPP_INFO(this->get_logger(), "Shutting down ORB-SLAM3...");
            mpSLAM_->Shutdown();
            
            // Save trajectories
            mpSLAM_->SaveTrajectoryTUM("CameraTrajectory_ROS2.txt");
            mpSLAM_->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory_ROS2.txt");
        }

        if (trajectory_file_.is_open())
        {
            trajectory_file_.close();
        }

        RCLCPP_INFO(this->get_logger(), "Stereo SLAM node shutdown complete.");
    }

private:
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::Image, sensor_msgs::msg::Image>;

    void InitializeCoordinateTransform()
    {
        // =================================================================
        // Coordinate System Definitions:
        // =================================================================
        // World/RViz (ROS standard): X-forward, Y-left, Z-up
        // Camera Optical: Z-forward(viewing), X-right, Y-down
        // 
        // Camera tilt: 0° = horizontal, positive = looking down
        // Example: 19.923° means camera looks 19.923° below horizontal
        //
        // ORB-SLAM3 outputs camera pose in optical frame convention
        // =================================================================
        
        if (use_camera_quat_)
        {
            // Use quaternion from /tf (base_link -> camera) to compute R_cam_to_world
            // This quaternion represents the camera orientation in the rover body frame
            
            // q_bc: base_link -> camera (from Isaac Sim /tf)
            Eigen::Quaternionf q_bc(camera_qw_, camera_qx_, camera_qy_, camera_qz_);
            q_bc.normalize();
            
            // R_bc: rotation matrix from camera to base_link frame
            Eigen::Matrix3f R_bc = q_bc.toRotationMatrix();
            
            // For camera optical frame: we need to account for the optical frame convention
            // Camera optical: Z-forward, X-right, Y-down
            // Rover body: X-forward, Y-left, Z-up
            //
            // If camera is pointing forward with no tilt:
            //   cam_Z (forward) -> body_X
            //   cam_X (right)   -> body_-Y
            //   cam_Y (down)    -> body_-Z
            //
            // This is a fixed transform from optical to body, then R_bc is applied
            Eigen::Matrix3f R_optical_to_body;
            R_optical_to_body <<  0.0f,  0.0f,  1.0f,   // cam_Z -> body_X
                                 -1.0f,  0.0f,  0.0f,   // cam_X -> body_-Y
                                  0.0f, -1.0f,  0.0f;   // cam_Y -> body_-Z
            
            // R_bc from /tf includes the camera mounting orientation (tilt, etc.)
            // R_cam_to_world (in body frame) = R_bc * R_optical_to_body
            // Since we're working in body frame initially (before rover orientation is applied):
            R_cam_to_world_ = R_bc;
            
            RCLCPP_INFO(this->get_logger(), 
                "Coordinate transform initialized from quaternion:");
            RCLCPP_INFO(this->get_logger(), 
                "  Camera quat (xyzw): [%.4f, %.4f, %.4f, %.4f]", 
                camera_qx_, camera_qy_, camera_qz_, camera_qw_);
        }
        else
        {
            // Use tilt angle (legacy method)
            float tilt_rad = camera_tilt_deg_ * M_PI / 180.0f;
            float cos_t = std::cos(tilt_rad);
            float sin_t = std::sin(tilt_rad);
            
            // R_cam_to_world: transforms vectors from camera optical frame to world frame
            // 
            // Camera optical axes expressed in world coordinates:
            //   cam_X (right)   -> world: [0, -1, 0]  (world -Y is right)
            //   cam_Y (down)    -> world: [-sin_t, 0, -cos_t]
            //   cam_Z (forward) -> world: [cos_t, 0, -sin_t] (tilted down from X toward -Z)
            //
            // Matrix columns = where each camera axis points in world frame
            R_cam_to_world_ <<  0.0f,   -sin_t,   cos_t,
                               -1.0f,    0.0f,    0.0f,
                                0.0f,   -cos_t,  -sin_t;

            RCLCPP_INFO(this->get_logger(), 
                "Coordinate transform initialized from tilt angle:");
            RCLCPP_INFO(this->get_logger(), 
                "  Camera tilt from horizontal: %.3f deg (+ = looking down)", camera_tilt_deg_);
        }
        
        RCLCPP_INFO(this->get_logger(), 
            "  R_cam_to_world:\n    [%6.3f %6.3f %6.3f]\n    [%6.3f %6.3f %6.3f]\n    [%6.3f %6.3f %6.3f]",
            R_cam_to_world_(0,0), R_cam_to_world_(0,1), R_cam_to_world_(0,2),
            R_cam_to_world_(1,0), R_cam_to_world_(1,1), R_cam_to_world_(1,2),
            R_cam_to_world_(2,0), R_cam_to_world_(2,1), R_cam_to_world_(2,2));
    }

    bool InitializePoseFromTfGt()
    {
        // Subscribe to /tf_gt and wait for the first message using spin_until_future_complete
        RCLCPP_INFO(this->get_logger(), "Subscribing to %s topic for initial pose...", tf_gt_topic_.c_str());
        
        std::promise<geometry_msgs::msg::TransformStamped> promise;
        auto future = promise.get_future();
        bool promise_set = false;
        std::mutex promise_mutex;

        auto tf_callback = [&](const tf2_msgs::msg::TFMessage::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(promise_mutex);
            if (promise_set) return;  // Already got what we need
            
            for (const auto& transform : msg->transforms) {
                if (transform.child_frame_id == tf_gt_child_frame_) {
                    RCLCPP_INFO(this->get_logger(), "Received transform from %s", tf_gt_topic_.c_str());
                    promise.set_value(transform);
                    promise_set = true;
                    return;
                }
            }
        };

        auto sub = this->create_subscription<tf2_msgs::msg::TFMessage>(
            tf_gt_topic_, rclcpp::QoS(10), tf_callback);

        // Spin until we receive the message or timeout
        auto start_time = std::chrono::steady_clock::now();
        auto timeout = std::chrono::duration<double>(tf_gt_timeout_);
        
        while (rclcpp::ok()) {
            rclcpp::spin_some(this->get_node_base_interface());
            
            if (future.wait_for(std::chrono::milliseconds(10)) == std::future_status::ready) {
                break;
            }
            
            auto elapsed = std::chrono::steady_clock::now() - start_time;
            if (elapsed > timeout) {
                RCLCPP_WARN(this->get_logger(), "Timeout waiting for %s (%.1f seconds)", 
                    tf_gt_topic_.c_str(), tf_gt_timeout_);
                return false;
            }
        }

        if (!rclcpp::ok()) {
            return false;
        }

        geometry_msgs::msg::TransformStamped tf_msg = future.get();

        // Extract pose from transform message
        t_gt_rover_(0) = tf_msg.transform.translation.x;
        t_gt_rover_(1) = tf_msg.transform.translation.y;
        t_gt_rover_(2) = tf_msg.transform.translation.z;
        
        q_gt_rover_.w() = tf_msg.transform.rotation.w;
        q_gt_rover_.x() = tf_msg.transform.rotation.x;
        q_gt_rover_.y() = tf_msg.transform.rotation.y;
        q_gt_rover_.z() = tf_msg.transform.rotation.z;
        q_gt_rover_.normalize();
        
        RCLCPP_INFO(this->get_logger(), "Received initial pose from %s (frame: %s -> %s)",
            tf_gt_topic_.c_str(), tf_msg.header.frame_id.c_str(), tf_msg.child_frame_id.c_str());
        
        // Compute camera pose from rover pose
        return ComputeCameraPoseFromRover();
    }

    bool InitializePoseFromParams()
    {
        // Set rover position and orientation from parameters
        t_gt_rover_(0) = init_rover_x_;
        t_gt_rover_(1) = init_rover_y_;
        t_gt_rover_(2) = init_rover_z_;
        
        q_gt_rover_.w() = init_rover_qw_;
        q_gt_rover_.x() = init_rover_qx_;
        q_gt_rover_.y() = init_rover_qy_;
        q_gt_rover_.z() = init_rover_qz_;
        q_gt_rover_.normalize();
        
        // Compute camera position and orientation (same logic as LoadInitialPoseFromGT)
        return ComputeCameraPoseFromRover();
    }

    bool ComputeCameraPoseFromRover()
    {
        // Compute camera position from rover position + offset
        // And compute camera orientation from rover orientation + camera tilt
        
        // Camera offset in body frame
        Eigen::Vector3f cam_offset_body(camera_offset_x_, camera_offset_y_, camera_offset_z_);
        
        // Rover rotation matrix (body -> world)
        Eigen::Matrix3f R_rover = q_gt_rover_.toRotationMatrix();
        
        // Transform camera offset to world frame and add to rover position
        t_gt_init_ = t_gt_rover_ + R_rover * cam_offset_body;
        
        // Compute camera orientation: rover_orientation * R_cam_to_world
        // R_rover: body frame -> world frame (rover orientation)
        // R_cam_to_world_: camera optical -> rover body (includes tilt)
        // Combined: camera optical -> world = R_rover * R_cam_to_world_
        Eigen::Matrix3f R_init = R_rover * R_cam_to_world_;
        q_gt_init_ = Eigen::Quaternionf(R_init);
        q_gt_init_.normalize();
        
        // Build SE3 with full rotation and translation
        T_gt_init_ = Sophus::SE3f(R_init, t_gt_init_);
        
        RCLCPP_INFO(this->get_logger(), "GT initialization computed:");
        RCLCPP_INFO(this->get_logger(), "  Rover pos: [%.3f, %.3f, %.3f]",
            t_gt_rover_(0), t_gt_rover_(1), t_gt_rover_(2));
        RCLCPP_INFO(this->get_logger(), "  Rover quat (xyzw): [%.3f, %.3f, %.3f, %.3f]",
            q_gt_rover_.x(), q_gt_rover_.y(), q_gt_rover_.z(), q_gt_rover_.w());
        RCLCPP_INFO(this->get_logger(), "  Camera offset (body): [%.3f, %.3f, %.3f]",
            cam_offset_body(0), cam_offset_body(1), cam_offset_body(2));
        RCLCPP_INFO(this->get_logger(), "  Camera init pos: [%.3f, %.3f, %.3f]",
            t_gt_init_(0), t_gt_init_(1), t_gt_init_(2));
        RCLCPP_INFO(this->get_logger(), "  Camera init quat (xyzw): [%.3f, %.3f, %.3f, %.3f]",
            q_gt_init_.x(), q_gt_init_.y(), q_gt_init_.z(), q_gt_init_.w());
        
        return true;
    }

    bool LoadInitialPoseFromGT(const std::string& gt_file)
    {
        std::ifstream fGT(gt_file);
        if (!fGT.is_open())
        {
            return false;
        }

        std::string line;
        while (std::getline(fGT, line))
        {
            // Skip empty lines and comments
            if (line.empty() || line[0] == '#')
                continue;

            // Parse EuRoC IMU format: timestamp,x,y,z,qw,qx,qy,qz,...
            std::stringstream ss(line);
            std::string token;
            std::vector<double> values;

            while (std::getline(ss, token, ','))
            {
                token.erase(std::remove_if(token.begin(), token.end(), ::isspace), 
                    token.end());
                if (!token.empty())
                {
                    try {
                        values.push_back(std::stod(token));
                    } catch (...) {
                        continue;
                    }
                }
            }

            // Need at least timestamp + position (3) + quaternion (4) = 8 values
            if (values.size() >= 8)
            {
                // GT gives rover body position in world frame (Z-up)
                t_gt_rover_(0) = values[1];  // x
                t_gt_rover_(1) = values[2];  // y
                t_gt_rover_(2) = values[3];  // z
                q_gt_rover_.w() = values[4]; // qw
                q_gt_rover_.x() = values[5]; // qx
                q_gt_rover_.y() = values[6]; // qy
                q_gt_rover_.z() = values[7]; // qz
                q_gt_rover_.normalize();

                fGT.close();
                
                // Compute camera pose from rover pose
                return ComputeCameraPoseFromRover();
            }
        }

        fGT.close();
        return false;
    }

    Sophus::SE3f TransformToWorldFrame(const Sophus::SE3f& Twc)
    {
        // Get relative pose from first frame
        Sophus::SE3f T_rel = T_slam_first_.inverse() * Twc;

        // Apply coordinate transformation
        Eigen::Vector3f t_rel_cam = T_rel.translation();
        Eigen::Matrix3f R_rel_cam = T_rel.rotationMatrix();

        // Get rover initial orientation (identity if no GT init)
        Eigen::Matrix3f R_rover = hasGTInit_ ? q_gt_rover_.toRotationMatrix() : Eigen::Matrix3f::Identity();
        
        // Full transform: R_rover * R_cam_to_world_ 
        // R_cam_to_world_: camera optical -> rover body (includes camera tilt)
        // R_rover: rover body -> world (initial rover orientation)
        Eigen::Matrix3f R_cam_to_world_full = R_rover * R_cam_to_world_;

        // Transform to world coordinate system
        Eigen::Vector3f t_rel_world = R_cam_to_world_full * t_rel_cam;
        Eigen::Matrix3f R_rel_world = R_cam_to_world_full * R_rel_cam * R_cam_to_world_full.transpose();

        // Re-orthogonalize the rotation matrix using SVD
        Eigen::JacobiSVD<Eigen::Matrix3f> svd(R_rel_world, 
            Eigen::ComputeFullU | Eigen::ComputeFullV);
        R_rel_world = svd.matrixU() * svd.matrixV().transpose();
        
        // Ensure proper rotation (det = +1)
        if (R_rel_world.determinant() < 0) {
            R_rel_world = -R_rel_world;
        }

        Sophus::SE3f T_rel_world(R_rel_world, t_rel_world);

        // Apply GT initial pose if available
        if (hasGTInit_)
        {
            // T_gt_init_ now contains full SE3 with rotation
            return T_gt_init_ * T_rel_world;
        }
        else
        {
            return T_rel_world;
        }
    }

    Sophus::SE3f TransformForNvblox(const Sophus::SE3f& Twc)
    {
        // For nvblox: Transform BOTH translation AND rotation to world frame
        // This keeps them consistent - depth will project along world-transformed Z axis
        //
        // nvblox computes 3D points as: P_world = R * P_cam + t
        // where P_cam = [(u-cx)*d/fx, (v-cy)*d/fy, d] (optical convention, Z=depth)
        //
        // By transforming R to world frame, the depth projection direction
        // also gets transformed, maintaining consistency.
        
        Sophus::SE3f T_rel = T_slam_first_.inverse() * Twc;
        
        Eigen::Vector3f t_cam = T_rel.translation();
        Eigen::Matrix3f R_cam = T_rel.rotationMatrix();
        
        // Get rover initial orientation (identity if no GT init)
        Eigen::Matrix3f R_rover = hasGTInit_ ? q_gt_rover_.toRotationMatrix() : Eigen::Matrix3f::Identity();
        
        // Full transform: R_rover * R_cam_to_world_ 
        // R_cam_to_world_: camera optical -> rover body (includes camera tilt)
        // R_rover: rover body -> world (initial rover orientation)
        Eigen::Matrix3f R_cam_to_world_full = R_rover * R_cam_to_world_;
        
        // Transform both to world coordinates
        Eigen::Vector3f t_world = R_cam_to_world_full * t_cam;
        Eigen::Matrix3f R_world = R_cam_to_world_full * R_cam;
        
        // Re-orthogonalize rotation matrix
        Eigen::JacobiSVD<Eigen::Matrix3f> svd(R_world, 
            Eigen::ComputeFullU | Eigen::ComputeFullV);
        R_world = svd.matrixU() * svd.matrixV().transpose();
        if (R_world.determinant() < 0) {
            R_world = -R_world;
        }
        
        Sophus::SE3f T_rel_world(R_world, t_world);
        
        // Apply GT initial position if available (places camera at correct world position)
        if (hasGTInit_)
        {
            // Add translation offset from GT initial pose
            // t_gt_init_ is already the camera position in world frame
            Eigen::Vector3f t_final = t_gt_init_ + t_world;
            
            return Sophus::SE3f(R_world, t_final);
        }
        
        return T_rel_world;
    }

    void GrabStereo(
        const sensor_msgs::msg::Image::ConstSharedPtr& msgLeft,
        const sensor_msgs::msg::Image::ConstSharedPtr& msgRight)
    {
        // Convert ROS images to cv::Mat
        cv_bridge::CvImageConstPtr cv_ptrLeft, cv_ptrRight;
        try
        {
            cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
            cv_ptrRight = cv_bridge::toCvShare(msgRight);
        }
        catch (cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat imLeft = cv_ptrLeft->image;
        cv::Mat imRight = cv_ptrRight->image;

        // Resize if needed
        if (imageScale_ != 1.0f)
        {
            int width = imLeft.cols * imageScale_;
            int height = imLeft.rows * imageScale_;
            cv::resize(imLeft, imLeft, cv::Size(width, height));
            cv::resize(imRight, imRight, cv::Size(width, height));
        }

        // Get timestamp
        double timestamp = msgLeft->header.stamp.sec + 
            msgLeft->header.stamp.nanosec * 1e-9;

        // Track stereo
        Sophus::SE3f Tcw = mpSLAM_->TrackStereo(imLeft, imRight, timestamp);

        // Check if tracking is valid
        if (Tcw.matrix().isZero(0))
        {
            RCLCPP_DEBUG(this->get_logger(), "Tracking lost or not initialized");
            return;
        }

        // Convert to world pose (camera-in-world)
        Sophus::SE3f Twc = Tcw.inverse();

        // Capture first valid pose for alignment
        if (!bFirstValidPose_)
        {
            T_slam_first_ = Twc;
            bFirstValidPose_ = true;
            RCLCPP_INFO(this->get_logger(), "First valid SLAM pose captured");
        }

        // Transform based on pose_mode
        Sophus::SE3f Twc_out;
        if (pose_mode_ == "camera") {
            // For nvblox: transform to world frame (X-forward, Y-left, Z-up)
            Twc_out = TransformForNvblox(Twc);
        } else {
            // For visualization: full world frame transform with GT alignment
            Twc_out = TransformToWorldFrame(Twc);
        }

        // Extract position and orientation
        Eigen::Vector3f position = Twc_out.translation();
        Eigen::Quaternionf orientation = Twc_out.unit_quaternion();

        // Determine timestamp: 'current' for real-time sync, 'original' for rosbag replay
        rclcpp::Time output_stamp;
        if (timestamp_mode_ == "current") {
            output_stamp = this->now();
        } else {
            output_stamp = msgLeft->header.stamp;
        }

        // Create and publish PoseStamped
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.stamp = output_stamp;
        pose_msg.header.frame_id = output_frame_id_;
        pose_msg.pose.position.x = position.x();
        pose_msg.pose.position.y = position.y();
        pose_msg.pose.position.z = position.z();
        pose_msg.pose.orientation.x = orientation.x();
        pose_msg.pose.orientation.y = orientation.y();
        pose_msg.pose.orientation.z = orientation.z();
        pose_msg.pose.orientation.w = orientation.w();
        pose_pub_->publish(pose_msg);

        // Create and publish Odometry
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header = pose_msg.header;
        odom_msg.child_frame_id = camera_frame_id_;
        odom_msg.pose.pose = pose_msg.pose;
        // Velocity not estimated by ORB-SLAM3 directly
        odom_pub_->publish(odom_msg);

        // Broadcast TF
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header = pose_msg.header;
        tf_msg.child_frame_id = camera_frame_id_;
        tf_msg.transform.translation.x = position.x();
        tf_msg.transform.translation.y = position.y();
        tf_msg.transform.translation.z = position.z();
        tf_msg.transform.rotation = pose_msg.pose.orientation;
        tf_broadcaster_->sendTransform(tf_msg);

        // Save to trajectory file if enabled
        if (trajectory_file_.is_open())
        {
            trajectory_file_ << timestamp << " "
                << position.x() << " " << position.y() << " " << position.z() << " "
                << orientation.x() << " " << orientation.y() << " " 
                << orientation.z() << " " << orientation.w() << std::endl;
            trajectory_file_.flush();
        }

        frame_count_++;
        if (verbose_ && frame_count_ % 100 == 0)
        {
            RCLCPP_INFO(this->get_logger(), "Processed %zu frames", frame_count_);
        }
    }

    // Parameters
    std::string vocabulary_path_;
    std::string settings_path_;
    std::string left_topic_;
    std::string right_topic_;
    double camera_tilt_deg_;
    bool enable_viewer_;
    std::string output_frame_id_;
    std::string camera_frame_id_;
    std::string gt_file_path_;
    std::string output_trajectory_file_;
    std::string timestamp_mode_;
    std::string pose_mode_;
    double camera_offset_x_;
    double camera_offset_y_;
    double camera_offset_z_;
    double camera_qw_;
    double camera_qx_;
    double camera_qy_;
    double camera_qz_;
    bool use_camera_quat_;
    bool use_param_init_pose_;
    bool use_tf_gt_init_;
    std::string tf_gt_topic_;
    std::string tf_gt_child_frame_;
    double tf_gt_timeout_;
    bool use_best_effort_;
    bool verbose_;
    double init_rover_x_;
    double init_rover_y_;
    double init_rover_z_;
    double init_rover_qw_;
    double init_rover_qx_;
    double init_rover_qy_;
    double init_rover_qz_;

    // ORB-SLAM3
    std::unique_ptr<ORB_SLAM3::System> mpSLAM_;
    float imageScale_ = 1.0f;

    // Coordinate transformation
    Eigen::Matrix3f R_cam_to_world_;
    bool bFirstValidPose_;
    Sophus::SE3f T_slam_first_;
    
    // GT alignment
    bool hasGTInit_ = false;
    Eigen::Vector3f t_gt_rover_ = Eigen::Vector3f::Zero();  // Rover position from GT
    Eigen::Quaternionf q_gt_rover_ = Eigen::Quaternionf::Identity();  // Rover orientation from GT
    Eigen::Vector3f t_gt_init_ = Eigen::Vector3f::Zero();  // Camera position (rover + offset)
    Eigen::Quaternionf q_gt_init_ = Eigen::Quaternionf::Identity();
    Sophus::SE3f T_gt_init_;

    // ROS
    message_filters::Subscriber<sensor_msgs::msg::Image> left_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> right_sub_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
    
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // Trajectory output
    std::ofstream trajectory_file_;
    size_t frame_count_ = 0;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StereoSlamNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
