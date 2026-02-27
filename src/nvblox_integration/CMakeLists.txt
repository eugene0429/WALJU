cmake_minimum_required(VERSION 3.8)
project(nvblox_integration)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install config files
install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install Python scripts
# We install them twice: once with .py and once without for ROS2 run convenience
install(PROGRAMS
  scripts/goal_sender.py
  scripts/print_rviz_goal.py
  scripts/ground_truth_publisher.py
  scripts/pose_comparison_plot.py
  scripts/generate_nav2_params.py
  scripts/navigation_metrics_logger.py
  DESTINATION lib/${PROJECT_NAME}
)

# Also install without .py extension for cleaner 'ros2 run'
install(PROGRAMS scripts/goal_sender.py DESTINATION lib/${PROJECT_NAME} RENAME goal_sender)
install(PROGRAMS scripts/print_rviz_goal.py DESTINATION lib/${PROJECT_NAME} RENAME print_rviz_goal)
install(PROGRAMS scripts/ground_truth_publisher.py DESTINATION lib/${PROJECT_NAME} RENAME ground_truth_publisher)
install(PROGRAMS scripts/pose_comparison_plot.py DESTINATION lib/${PROJECT_NAME} RENAME pose_comparison_plot)
install(PROGRAMS scripts/generate_nav2_params.py DESTINATION lib/${PROJECT_NAME} RENAME generate_nav2_params)
install(PROGRAMS scripts/navigation_metrics_logger.py DESTINATION lib/${PROJECT_NAME} RENAME navigation_metrics_logger)

# Install internal nodes
install(PROGRAMS
  nvblox_integration/goal_pose_corrector.py
  nvblox_integration/goal_reached_stop.py
  nvblox_integration/initial_rotation_controller.py
  DESTINATION lib/${PROJECT_NAME}
)
# Also without .py
install(PROGRAMS nvblox_integration/goal_pose_corrector.py DESTINATION lib/${PROJECT_NAME} RENAME goal_pose_corrector)
install(PROGRAMS nvblox_integration/goal_reached_stop.py DESTINATION lib/${PROJECT_NAME} RENAME goal_reached_stop)
install(PROGRAMS nvblox_integration/initial_rotation_controller.py DESTINATION lib/${PROJECT_NAME} RENAME initial_rotation_controller)

ament_package()
