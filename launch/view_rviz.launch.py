from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    use_gui_arg = DeclareLaunchArgument("use_gui", default_value="false",
                                        description="Open joint_state_publisher_gui")
    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="false")
    rviz_arg = DeclareLaunchArgument("rviz", default_value="true")

    use_gui = LaunchConfiguration("use_gui")
    use_sim_time = LaunchConfiguration("use_sim_time")
    rviz = LaunchConfiguration("rviz")

    pkg = FindPackageShare("mujoco_accessories")
    ridgeback_xacro = PathJoinSubstitution([pkg, "urdf", "ridgeback.urdf.xacro"])


    robot_description = Command(["xacro ", ridgeback_xacro])

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description,
                     "use_sim_time": use_sim_time}],
    )

    jsp_gui = Node(
        condition=IfCondition(use_gui),
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        output="screen",
    )
    jsp = Node(
        condition=UnlessCondition(use_gui),
        package="joint_state_publisher",
        executable="joint_state_publisher",
        output="screen",
    )

    rviz2 = Node(
        condition=IfCondition(rviz),
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=[],
    )

    return LaunchDescription([
        use_gui_arg, use_sim_time_arg, rviz_arg,
        rsp, jsp_gui, jsp, rviz2
    ])
