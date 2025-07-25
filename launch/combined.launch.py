import os
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
    pkg_path = get_package_share_directory('mujoco_accessories')

    # File paths
    xacro_file = os.path.join(pkg_path, 'urdf', 'ridgeback_ur5e.urdf.xacro')
    mujoco_model_path = os.path.join(pkg_path, 'mujoco', 'combined', 'combined_default.xml')
    controller_yaml = os.path.join(pkg_path, 'configs', 'ridgeback_ur5e_controllers.yaml')

    # Xacro processing
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    robot_description = {'robot_description': doc.toxml()}

    # Nodes
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

    state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}]
    )

    joint_state_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    ur5e_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['ur5e_arm_controller', '--param-file', controller_yaml],
        output='screen'
    )

    ridgeback_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['ridgeback_velocity_controller', '--param-file', controller_yaml],
        remappings=[("/cmd_vel", "/ridgeback_velocity_controller/cmd_vel")],
        output='screen'
    )

    return LaunchDescription([
        mujoco_node,
        state_publisher,
        TimerAction(period=2.0, actions=[joint_state_spawner]),
        TimerAction(period=5.0, actions=[ur5e_spawner]),
        TimerAction(period=8.0, actions=[ridgeback_spawner])
    ])
