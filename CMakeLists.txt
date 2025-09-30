cmake_minimum_required(VERSION 3.8)
project(turtlebot_project)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)        
find_package(image_transport REQUIRED) 
find_package(tf2_ros REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(yolo_msgs REQUIRED)

add_executable(cmd_vel_bridge src/cmd_vel_bridge.cpp)
ament_target_dependencies(cmd_vel_bridge rclcpp geometry_msgs)

add_executable(realsense_bridge src/realsense_bridge.cpp)
ament_target_dependencies(realsense_bridge
  rclcpp
  geometry_msgs
  sensor_msgs
  cv_bridge
  image_transport
  std_msgs
)
add_executable(static_tf_bridge src/static_tf_bridge.cpp)
ament_target_dependencies(static_tf_bridge rclcpp std_msgs tf2_ros )

add_executable(task_node src/task_node.cpp)
ament_target_dependencies(task_node
  rclcpp
  rclcpp_action
  geometry_msgs
  std_msgs
  nav2_msgs
  yolo_msgs
)
target_link_libraries(task_node nlohmann_json::nlohmann_json)
install(TARGETS
  cmd_vel_bridge
  realsense_bridge 
  static_tf_bridge
  task_node
  DESTINATION lib/${PROJECT_NAME}
  )

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
