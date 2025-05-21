import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch_ros.actions import Node
import xacro
 
 
def generate_launch_description():
    mujoco_accesories_example_path = os.path.join(
        get_package_share_directory('mujoco_accessories'))
 
    xacro_file = os.path.join(mujoco_accesories_example_path,
                              'urdf',
                              'ridgeback.urdf.xacro')
 
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    robot_description = {'robot_description': doc.toxml()}
 
    controller_config_file = os.path.join(mujoco_accesories_example_path, 'configs', 'ridgeback_controllers.yaml')
 
    node_mujoco_ros2_control = Node(
        package='mujoco_ros2_control',
        executable='mujoco_ros2_control',
        output='screen',
        parameters=[
            robot_description,
            controller_config_file,
            {'mujoco_model_path':os.path.join(mujoco_accesories_example_path, 'robot', 'ridgeback_default.xml')},
            {"use_sim_time": True}
        ]
    )
 
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description,{"use_sim_time": True}]
    )
 
    load_joint_state_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )
 
    load_diff_drive_base_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["ridgeback_velocity_controller", "--param-file", controller_config_file],
        remappings=[
            ("/cmd_vel", "/ridgeback_velocity_controller/cmd_vel")
        ],
        output="screen"
    )
 
    return LaunchDescription([
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=node_mujoco_ros2_control,
                on_start=[load_joint_state_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_diff_drive_base_controller],
            )
        ),
        node_mujoco_ros2_control,
        node_robot_state_publisher
    ])