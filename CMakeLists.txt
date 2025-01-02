cmake_minimum_required(VERSION 3.5)
project(odom_merger)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(target_odom_merger SHARED
  DIRECTORY src
)
target_include_directories(target_odom_merger
  SYSTEM PUBLIC
)

rclcpp_components_register_node(target_odom_merger
  PLUGIN "odom_merger::OdomMerger"
  EXECUTABLE odom_merger_node
)

ament_auto_package(
  INSTALL_TO_SHARE
)