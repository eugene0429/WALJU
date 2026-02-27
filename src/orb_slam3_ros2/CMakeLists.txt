cmake_minimum_required(VERSION 3.8)
project(orb_slam3_ros2)

# C++ 17 required for ORB-SLAM3
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ============================================================================
# Find dependencies
# ============================================================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# OpenCV
find_package(OpenCV 4 REQUIRED)

# Eigen3
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# Pangolin (for ORB-SLAM3 viewer)
find_package(Pangolin REQUIRED)

# ============================================================================
# ORB-SLAM3 paths
# ============================================================================
set(ORB_SLAM3_ROOT "$ENV{HOME}/WALJU/deps/ORB_SLAM3")

# Include directories
include_directories(
  include
  ${ORB_SLAM3_ROOT}
  ${ORB_SLAM3_ROOT}/include
  ${ORB_SLAM3_ROOT}/include/CameraModels
  ${ORB_SLAM3_ROOT}/Thirdparty/Sophus
  ${EIGEN3_INCLUDE_DIR}
  ${Pangolin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# ORB-SLAM3 libraries
set(ORB_SLAM3_LIBS
  ${ORB_SLAM3_ROOT}/lib/libORB_SLAM3.so
  ${ORB_SLAM3_ROOT}/Thirdparty/DBoW2/lib/libDBoW2.so
  ${ORB_SLAM3_ROOT}/Thirdparty/g2o/lib/libg2o.so
)

# ============================================================================
# Dataset Publisher Node
# ============================================================================
add_executable(dataset_publisher
  src/dataset_publisher_node.cpp
)

ament_target_dependencies(dataset_publisher
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
  image_transport
)

target_link_libraries(dataset_publisher
  ${OpenCV_LIBS}
)

# ============================================================================
# Stereo SLAM Node
# ============================================================================
add_executable(stereo_slam
  src/stereo_slam_node.cpp
)

ament_target_dependencies(stereo_slam
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  std_msgs
  cv_bridge
  image_transport
  message_filters
  tf2
  tf2_ros
  tf2_geometry_msgs
)

target_link_libraries(stereo_slam
  ${ORB_SLAM3_LIBS}
  ${OpenCV_LIBS}
  ${Pangolin_LIBRARIES}
  Eigen3::Eigen
  -lboost_system
  -lboost_serialization
  -lcrypto
)

# ============================================================================
# Install
# ============================================================================
install(TARGETS
  dataset_publisher
  stereo_slam
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  scripts/tf_gt_relay.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

# ============================================================================
# Testing
# ============================================================================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
