from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Args
    use_gui_arg   = DeclareLaunchArgument("use_gui", default_value="true")
    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="true")
    ur_type_arg   = DeclareLaunchArgument("ur_type", default_value="ur5e")
    name_arg      = DeclareLaunchArgument("name",    default_value="ur5e")
    prefix_arg    = DeclareLaunchArgument("prefix",  default_value='""')  
    rviz_arg      = DeclareLaunchArgument("rviz",    default_value="true")

    use_gui    = LaunchConfiguration("use_gui")
    use_sim    = LaunchConfiguration("use_sim_time")
    ur_type    = LaunchConfiguration("ur_type")
    name       = LaunchConfiguration("name")
    prefix     = LaunchConfiguration("prefix")
    rviz       = LaunchConfiguration("rviz")

    ur_desc = FindPackageShare("ur_description")
    ur_xacro = PathJoinSubstitution([ur_desc, "urdf", "ur.urdf.xacro"])

    robot_description = Command([
        "xacro ",
        ur_xacro,
        " name:=", name,
        " ur_type:=", ur_type,
        " prefix:=", prefix,
        " safety_limits:=true",
        " safety_pos_margin:=0.15",
        " safety_k_position:=20",
    ])

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": use_sim}],
    )

    # Joint states: GUI when requested; otherwise headless publisher
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
        use_gui_arg, use_sim_time_arg, ur_type_arg, name_arg, prefix_arg, rviz_arg,
        rsp, jsp_gui, jsp, rviz2
    ])
