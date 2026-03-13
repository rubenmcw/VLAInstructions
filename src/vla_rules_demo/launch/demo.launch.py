from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
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

    return LaunchDescription([sim, fake_vla, executor])