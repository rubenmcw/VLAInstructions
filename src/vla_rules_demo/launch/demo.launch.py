from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

def generate_launch_description():
    robot_x = LaunchConfiguration("robot_x")
    robot_y = LaunchConfiguration("robot_y")
    robot_z = LaunchConfiguration("robot_z")
    robot_roll = LaunchConfiguration("robot_roll")
    robot_pitch = LaunchConfiguration("robot_pitch")
    robot_yaw = LaunchConfiguration("robot_yaw")

    pkg_share = FindPackageShare("vla_rules_demo")
    params_file = PathJoinSubstitution([pkg_share, "config", "demo_params.yaml"])
    world_file = PathJoinSubstitution([pkg_share, "worlds", "table_objects.sdf"])

    ur_sim_pkg = FindPackageShare("ur_simulation_gz")
    ur_sim_launch = PathJoinSubstitution([ur_sim_pkg, "launch", "ur_sim_moveit.launch.py"])

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ur_sim_launch),
        launch_arguments={
            "ur_type": "ur5e",
            "world_file": world_file,
            "x": robot_x,
            "y": robot_y,
            "z": robot_z,
            "roll": robot_roll,
            "pitch": robot_pitch,
            "yaw": robot_yaw,
        }.items(),
    )

    fake_vla = Node(
        package="vla_rules_demo",
        executable="fake_vla_stub",
        name="fake_vla_stub",
        output="screen",
        parameters=[params_file],
    )

    executor = Node(
        package="vla_rules_demo",
        executable="arm_executor",
        name="arm_executor",
        output="screen",
        parameters=[params_file],
    )

    return LaunchDescription([
        # Default spawn puts the base slightly back from origin, facing the table at +x.
        DeclareLaunchArgument("robot_x", default_value="-0.2"),
        DeclareLaunchArgument("robot_y", default_value="0.0"),
        DeclareLaunchArgument("robot_z", default_value="0.0"),
        DeclareLaunchArgument("robot_roll", default_value="0.0"),
        DeclareLaunchArgument("robot_pitch", default_value="0.0"),
        DeclareLaunchArgument("robot_yaw", default_value="0.0"),
        sim,
        fake_vla,
        executor,
    ])
