import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    # Paths
    pkg_path = get_package_share_directory('mujoco_accessories')
    xacro_file = os.path.join(pkg_path, 'urdf', 'ur5e.urdf.xacro')
    controller_yaml = os.path.join(pkg_path, 'configs', 'ur_controllers.yaml')
    mujoco_model_path = os.path.join(pkg_path, 'robot', 'ur5e_default.xml')

    # Process xacro to robot_description
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    robot_description = {'robot_description': doc.toxml()}

    # MuJoCo + ROS 2 Control Node
    mujoco_node = Node(
        package='mujoco_ros2_control',
        executable='mujoco_ros2_control',
        output='screen',
        parameters=[
            robot_description,
            controller_yaml,
            {'mujoco_model_path': mujoco_model_path},
            {'use_sim_time': True}
        ]
    )

    # Robot State Publisher
    state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}]
    )

    # Spawner: Joint State Broadcaster
    joint_state_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    # Spawner: Joint Trajectory Controller
    jtc_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_trajectory_controller', '--param-file', controller_yaml],
        output='screen'
    )

    return LaunchDescription([
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=mujoco_node,
                on_start=[joint_state_spawner]
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_spawner,
                on_exit=[jtc_spawner]
            )
        ),
        mujoco_node,
        state_publisher
    ])
