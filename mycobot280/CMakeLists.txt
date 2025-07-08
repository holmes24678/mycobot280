cmake_minimum_required(VERSION 3.8)
project(mycobot280)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

add_executable(send_joint_goal_node src/send_joint_goal_node.cpp)

ament_target_dependencies(send_joint_goal_node
  rclcpp
  moveit_ros_planning_interface
  tf2_geometry_msgs
)

add_executable(send_pose_node src/send_pose_node.cpp)

ament_target_dependencies(send_pose_node
  rclcpp
  moveit_ros_planning_interface
  tf2_geometry_msgs
)



install(TARGETS
  send_pose_node send_joint_goal_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config urdf meshes
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/detect.py
    scripts/image_display.py
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
