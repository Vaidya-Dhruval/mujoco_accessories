import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_path = get_package_share_directory('mujoco_accessories')

    # Xacro and model paths
    xacro_file = os.path.join(pkg_path, 'urdf', 'ridgeback_ur5e.urdf.xacro')
    mujoco_model_path = os.path.join(pkg_path, 'mujoco', 'combined', 'combined_default.xml')
    controller_yaml = os.path.join(pkg_path, 'configs', 'ridgeback_ur5e_controllers.yaml')
    ur5e_yaml = os.path.join(pkg_path, 'configs', 'ur_controllers.yaml')
    ridgeback_yaml = os.path.join(pkg_path, 'configs', 'ridgeback_controllers.yaml')

    # Process Xacro (no mappings because prefix is removed)
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    robot_description = {'robot_description': doc.toxml()}

    # MuJoCo + ROS 2 Control node
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

    # Robot state publisher
    state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}]
    )

    # Joint State Broadcaster
    joint_state_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )
    # Spawner: Joint Trajectory Controller
    ur5e_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['ur5e_arm_controller', '--param-file', controller_yaml],
        output='screen'
    )

    # Ridgeback diff drive controller
    ridgeback_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['ridgeback_velocity_controller', '--param-file', controller_yaml],
        remappings=[
            ("/cmd_vel", "/ridgeback_velocity_controller/cmd_vel")
        ],
        output='screen'
    )

    return LaunchDescription([
        mujoco_node,
        state_publisher,
        RegisterEventHandler(
            OnProcessStart(
                target_action=mujoco_node,
                on_start=[joint_state_spawner]
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=joint_state_spawner,
                on_exit=[ridgeback_spawner]
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=ur5e_spawner,
                on_exit=[ur5e_spawner]
            )
        ),
    ])
