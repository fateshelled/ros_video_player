cmake_minimum_required(VERSION 3.5)
project(ros_video_player)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(OpenCV REQUIRED)

# executable
add_executable(ros_video_player
  src/ros_video_player.cpp
)
ament_target_dependencies(ros_video_player
  rclcpp
  rclcpp_components
  cv_bridge
  image_transport
  camera_info_manager
  builtin_interfaces
  std_msgs
  sensor_msgs
  OpenCV
)
target_include_directories(ros_video_player PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
install(TARGETS
ros_video_player
  DESTINATION lib/${PROJECT_NAME}
)

# component
add_library(ros_video_player_component SHARED
  src/ros_video_player.cpp
)
rclcpp_components_register_nodes(ros_video_player_component
 "ros_video_player::VideoPlayerNode")
target_compile_definitions(ros_video_player_component
  PRIVATE "ROS_VIDEO_PLAYER_LIBRARY"
)
target_compile_options(ros_video_player_component PUBLIC -Wall)

target_include_directories(ros_video_player_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(ros_video_player_component
  rclcpp
  rclcpp_components
  cv_bridge
  image_transport
  camera_info_manager
  builtin_interfaces
  std_msgs
  sensor_msgs
  OpenCV
)

install(TARGETS ros_video_player_component
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib
)


install(DIRECTORY
  launch
  video
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
