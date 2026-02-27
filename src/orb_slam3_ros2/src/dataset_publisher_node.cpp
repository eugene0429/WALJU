/**
 * Dataset Publisher Node for ORB-SLAM3 ROS2 Testing
 * 
 * Publishes stereo image pairs from LuSNAR dataset sequentially.
 * This node is used for testing SLAM node before Isaac Sim integration.
 * 
 * Published Topics:
 *   /stereo/left/image_raw  (sensor_msgs/Image)
 *   /stereo/right/image_raw (sensor_msgs/Image)
 *   /depth/gt_image         (sensor_msgs/Image, 32FC1) - Ground truth depth from PFM files
 * 
 * Parameters:
 *   dataset_path: Path to dataset root (e.g., ~/WALJU/data/LuSNAR/Moon_2)
 *   left_subdir: Subdirectory for left images (default: "left")
 *   right_subdir: Subdirectory for right images (default: "right")
 *   depth_subdir: Subdirectory for GT depth PFM files (default: "depth")
 *   publish_rate: Publishing rate in Hz (default: 10.0)
 *   loop: Whether to loop the dataset (default: false)
 *   start_index: Starting image index (default: 0)
 *   publish_gt_depth: Whether to publish GT depth (default: true)
 */

#include <chrono>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <filesystem>
#include <cstring>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <cv_bridge/cv_bridge.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>

/**
 * Read PFM (Portable Float Map) file format commonly used for depth maps
 * Returns a CV_32FC1 matrix (single channel float)
 * PFM format: http://netpbm.sourceforge.net/doc/pfm.html
 */
cv::Mat ReadPFM(const std::string& filename)
{
    std::ifstream file(filename, std::ios::binary);
    if (!file.is_open())
    {
        return cv::Mat();
    }

    // Read header
    std::string type;
    std::getline(file, type);
    
    // PF = color (3 channels), Pf = grayscale (1 channel)
    int channels = 0;
    if (type == "PF")
        channels = 3;
    else if (type == "Pf")
        channels = 1;
    else
    {
        return cv::Mat();
    }

    // Read dimensions
    int width, height;
    file >> width >> height;
    
    // Read scale factor (negative means little-endian)
    float scale;
    file >> scale;
    
    // Skip single whitespace character after scale
    file.get();
    
    bool little_endian = (scale < 0);
    scale = std::abs(scale);

    // Read pixel data
    cv::Mat img(height, width, channels == 1 ? CV_32FC1 : CV_32FC3);
    
    // PFM stores rows from bottom to top
    for (int y = height - 1; y >= 0; --y)
    {
        file.read(reinterpret_cast<char*>(img.ptr<float>(y)), 
                  width * channels * sizeof(float));
        
        // Handle endianness if needed (assuming system is little-endian)
        if (!little_endian)
        {
            float* row = img.ptr<float>(y);
            for (int x = 0; x < width * channels; ++x)
            {
                // Swap bytes for big-endian to little-endian
                char* bytes = reinterpret_cast<char*>(&row[x]);
                std::swap(bytes[0], bytes[3]);
                std::swap(bytes[1], bytes[2]);
            }
        }
    }
    
    // Apply scale if not 1.0
    if (scale != 1.0f)
    {
        img *= scale;
    }
    
    // If color, convert to single channel (use first channel as depth)
    if (channels == 3)
    {
        cv::Mat gray;
        cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
        return gray;
    }
    
    return img;
}

using namespace std::chrono_literals;

class DatasetPublisher : public rclcpp::Node
{
public:
    DatasetPublisher() : Node("dataset_publisher"), current_idx_(0)
    {
        // Declare parameters
        this->declare_parameter<std::string>("dataset_path", "");
        this->declare_parameter<std::string>("left_subdir", "left");
        this->declare_parameter<std::string>("right_subdir", "right");
        this->declare_parameter<std::string>("depth_subdir", "depth");
        this->declare_parameter<double>("publish_rate", 10.0);
        this->declare_parameter<bool>("loop", false);
        this->declare_parameter<int>("start_index", 0);
        this->declare_parameter<std::string>("frame_id", "camera_link");
        this->declare_parameter<bool>("publish_gt_depth", true);

        // Get parameters
        dataset_path_ = this->get_parameter("dataset_path").as_string();
        left_subdir_ = this->get_parameter("left_subdir").as_string();
        right_subdir_ = this->get_parameter("right_subdir").as_string();
        depth_subdir_ = this->get_parameter("depth_subdir").as_string();
        publish_rate_ = this->get_parameter("publish_rate").as_double();
        loop_ = this->get_parameter("loop").as_bool();
        current_idx_ = this->get_parameter("start_index").as_int();
        frame_id_ = this->get_parameter("frame_id").as_string();
        publish_gt_depth_ = this->get_parameter("publish_gt_depth").as_bool();

        if (dataset_path_.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "dataset_path parameter is required!");
            rclcpp::shutdown();
            return;
        }

        // Build full paths
        left_path_ = dataset_path_ + "/" + left_subdir_;
        right_path_ = dataset_path_ + "/" + right_subdir_;
        depth_path_ = dataset_path_ + "/" + depth_subdir_;

        RCLCPP_INFO(this->get_logger(), "Dataset path: %s", dataset_path_.c_str());
        RCLCPP_INFO(this->get_logger(), "Left images: %s", left_path_.c_str());
        RCLCPP_INFO(this->get_logger(), "Right images: %s", right_path_.c_str());
        if (publish_gt_depth_)
        {
            RCLCPP_INFO(this->get_logger(), "GT depth path: %s", depth_path_.c_str());
        }

        // Load images list
        if (!LoadImages())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to load images!");
            rclcpp::shutdown();
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Loaded %zu image pairs", timestamps_.size());
        RCLCPP_INFO(this->get_logger(), "Publishing at %.1f Hz", publish_rate_);

        // Create publishers
        left_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "/stereo/left/image_raw", 10);
        right_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "/stereo/right/image_raw", 10);
        
        // GT depth publisher
        if (publish_gt_depth_)
        {
            gt_depth_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
                "/depth/gt_image", 10);
            RCLCPP_INFO(this->get_logger(), "GT depth publishing enabled on /depth/gt_image");
        }

        // Create timer
        auto period = std::chrono::duration<double>(1.0 / publish_rate_);
        timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            std::bind(&DatasetPublisher::TimerCallback, this));

        RCLCPP_INFO(this->get_logger(), "Dataset publisher started!");
    }

private:
    bool LoadImages()
    {
        // Try to find timestamp file
        std::string timestamp_file = FindTimestampFile();
        if (timestamp_file.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Cannot find color_timestamp.txt");
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "Loading timestamps from: %s", timestamp_file.c_str());

        std::ifstream fTimes(timestamp_file);
        if (!fTimes.is_open())
        {
            RCLCPP_ERROR(this->get_logger(), "Cannot open timestamp file");
            return false;
        }

        std::string line;
        while (std::getline(fTimes, line))
        {
            // Skip empty lines and comments
            if (line.empty() || line[0] == '#')
                continue;

            // Parse CSV: timestamp,filename
            std::stringstream ss(line);
            std::string timestamp_str, filename;

            if (std::getline(ss, timestamp_str, ',') && std::getline(ss, filename, ','))
            {
                // Remove whitespace
                timestamp_str.erase(
                    std::remove_if(timestamp_str.begin(), timestamp_str.end(), ::isspace),
                    timestamp_str.end());
                filename.erase(
                    std::remove_if(filename.begin(), filename.end(), ::isspace),
                    filename.end());

                // Convert timestamp (nanoseconds to seconds)
                double timestamp = std::stod(timestamp_str) / 1e9;
                timestamps_.push_back(timestamp);

                // Build full paths for stereo images
                left_images_.push_back(left_path_ + "/" + filename);
                right_images_.push_back(right_path_ + "/" + filename);
                
                // Build GT depth path: replace extension with .pfm
                if (publish_gt_depth_)
                {
                    std::string depth_filename = filename;
                    size_t dot_pos = depth_filename.rfind('.');
                    if (dot_pos != std::string::npos)
                    {
                        depth_filename = depth_filename.substr(0, dot_pos) + ".pfm";
                    }
                    else
                    {
                        depth_filename += ".pfm";
                    }
                    depth_images_.push_back(depth_path_ + "/" + depth_filename);
                }
            }
        }
        fTimes.close();
        
        // Check if GT depth files exist for the first frame
        if (publish_gt_depth_ && !depth_images_.empty())
        {
            if (!std::filesystem::exists(depth_images_[0]))
            {
                RCLCPP_WARN(this->get_logger(), 
                    "GT depth file not found: %s. GT depth publishing disabled.", 
                    depth_images_[0].c_str());
                publish_gt_depth_ = false;
                depth_images_.clear();
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "Found %zu GT depth files", depth_images_.size());
            }
        }

        return !timestamps_.empty();
    }

    std::string FindTimestampFile()
    {
        std::vector<std::string> candidates = {
            left_path_ + "/color_timestamp.txt",
            dataset_path_ + "/color_timestamp.txt",
            right_path_ + "/color_timestamp.txt",
            dataset_path_ + "/image0/color_timestamp.txt",  // LuSNAR dataset structure
            dataset_path_ + "/image1/color_timestamp.txt",
        };

        for (const auto& path : candidates)
        {
            if (std::filesystem::exists(path))
            {
                return path;
            }
        }
        return "";
    }

    void TimerCallback()
    {
        if (current_idx_ >= timestamps_.size())
        {
            if (loop_)
            {
                current_idx_ = 0;
                RCLCPP_INFO(this->get_logger(), "Looping dataset...");
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "Dataset finished. Published %zu frames.", timestamps_.size());
                timer_->cancel();
                return;
            }
        }

        // Load images
        cv::Mat left_img = cv::imread(left_images_[current_idx_], cv::IMREAD_UNCHANGED);
        cv::Mat right_img = cv::imread(right_images_[current_idx_], cv::IMREAD_UNCHANGED);

        if (left_img.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Failed to load left image: %s", 
                        left_images_[current_idx_].c_str());
            current_idx_++;
            return;
        }

        if (right_img.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Failed to load right image: %s", 
                        right_images_[current_idx_].c_str());
            current_idx_++;
            return;
        }

        // Create header with timestamp
        std_msgs::msg::Header header;
        header.stamp = rclcpp::Time(static_cast<int64_t>(timestamps_[current_idx_] * 1e9));
        header.frame_id = frame_id_;

        // Convert RGBA to BGR if needed (LuSNAR dataset uses RGBA PNG)
        cv::Mat left_bgr, right_bgr;
        if (left_img.channels() == 4)
        {
            cv::cvtColor(left_img, left_bgr, cv::COLOR_BGRA2BGR);
            cv::cvtColor(right_img, right_bgr, cv::COLOR_BGRA2BGR);
        }
        else if (left_img.channels() == 3)
        {
            left_bgr = left_img;
            right_bgr = right_img;
        }
        else
        {
            // Grayscale
            left_bgr = left_img;
            right_bgr = right_img;
        }

        // Convert to ROS messages
        std::string encoding = (left_bgr.channels() == 1) ? "mono8" : "bgr8";
        
        auto left_msg = cv_bridge::CvImage(header, encoding, left_bgr).toImageMsg();
        auto right_msg = cv_bridge::CvImage(header, encoding, right_bgr).toImageMsg();

        // Publish stereo images
        left_pub_->publish(*left_msg);
        right_pub_->publish(*right_msg);
        
        // Publish GT depth if enabled
        if (publish_gt_depth_ && current_idx_ < depth_images_.size())
        {
            cv::Mat depth_img = ReadPFM(depth_images_[current_idx_]);
            if (!depth_img.empty())
            {
                // Ensure single channel float32
                if (depth_img.type() != CV_32FC1)
                {
                    depth_img.convertTo(depth_img, CV_32FC1);
                }
                auto depth_msg = cv_bridge::CvImage(header, "32FC1", depth_img).toImageMsg();
                gt_depth_pub_->publish(*depth_msg);
            }
            else
            {
                RCLCPP_WARN_ONCE(this->get_logger(), "Failed to load GT depth: %s", 
                            depth_images_[current_idx_].c_str());
            }
        }

        // Progress logging
        if (current_idx_ % 100 == 0)
        {
            RCLCPP_INFO(this->get_logger(), "Published frame %zu/%zu (t=%.3f)", 
                        current_idx_, timestamps_.size(), timestamps_[current_idx_]);
        }

        current_idx_++;
    }

    // Parameters
    std::string dataset_path_;
    std::string left_subdir_;
    std::string right_subdir_;
    std::string depth_subdir_;
    std::string left_path_;
    std::string right_path_;
    std::string depth_path_;
    std::string frame_id_;
    double publish_rate_;
    bool loop_;
    bool publish_gt_depth_;

    // Data
    std::vector<std::string> left_images_;
    std::vector<std::string> right_images_;
    std::vector<std::string> depth_images_;
    std::vector<double> timestamps_;
    size_t current_idx_;

    // ROS
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr right_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr gt_depth_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DatasetPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
