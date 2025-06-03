cmake_minimum_required(VERSION 3.10)
project(mujoco_accessories)

# Required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(controller_manager REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(joint_state_broadcaster REQUIRED)
find_package(velocity_controllers REQUIRED)
find_package(mujoco_ros2_control REQUIRED)
find_package(geometry_msgs REQUIRED)


# Install folders
install(DIRECTORY
  launch
  configs
  mujoco
  meshes
  robot
  urdf
  DESTINATION share/${PROJECT_NAME}
)

# Install executables
add_executable(ridgeback_velocity_publisher src/ridgeback_velocity_publisher.cpp)
ament_target_dependencies(ridgeback_velocity_publisher rclcpp geometry_msgs)

install(TARGETS
  ridgeback_velocity_publisher
  DESTINATION lib/${PROJECT_NAME}
)
add_executable(ur5e_trajectory_publisher src/ur5e_trajectory_publisher.cpp)
ament_target_dependencies(ur5e_trajectory_publisher rclcpp trajectory_msgs)

install(TARGETS
  ur5e_trajectory_publisher
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
