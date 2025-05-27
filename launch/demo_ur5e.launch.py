import os

from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnProcessStart

from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import FindExecutable


def generate_launch_description():
    pkg_share = FindPackageShare("mujoco_accessories").find("mujoco_accessories")
    ur_description_share = FindPackageShare("ur_description").find("ur_description")

    # URDF Description from ur_description with arguments
    xacro_path = os.path.join(ur_description_share, "urdf", "ur.urdf.xacro")
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ", xacro_path,
        " ur_type:=ur5e",
        " name:=ur5e",
        " safety_limits:=true",
        " safety_pos_margin:=0.15",
        " safety_k_position:=20"
    ])
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    # Controller configuration
    controller_config_file = os.path.join(pkg_share, "configs", "ur_controllers.yaml")

    # MuJoCo XML path
    mujoco_model_path = os.path.join(pkg_share, "robot", "ur5e_default.xml")

    # MuJoCo node
    node_mujoco = Node(
        package='mujoco_ros2_control',
        executable='mujoco_ros2_control',
        output='screen',
        parameters=[
            robot_description,
            controller_config_file,
            {"mujoco_model_path": mujoco_model_path},
            {"use_sim_time": True}
        ]
    )

    # Robot State Publisher
    node_rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": True}]
    )

    # Joint state broadcaster spawner
    load_joint_state_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    # (Optional) Arm controller spawner, replace with your actual controller
    # load_ur5e_controller = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["ur5e_arm_controller", "--param-file", controller_config_file],
    #     output="screen"
    # )

    return LaunchDescription([
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=node_mujoco,
                on_start=[load_joint_state_controller]
            )
        ),
        # RegisterEventHandler(
        #     event_handler=OnProcessExit(
        #         target_action=load_joint_state_controller,
        #         on_exit=[load_ur5e_controller]
        #     )
        # ),
        node_mujoco,
        node_rsp
    ])
