"""Go2 MuJoCo terrain demo with Nav2 and RViz.

The node is deliberately kept in this package (rather than in
workshop_navigation) so the Go1 flat-floor demo and the Go2 terrain demo can be
run independently without changing each other's Nav2 lifecycle.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory("go2_mujoco_locomotion")
    nav2_share = get_package_share_directory("nav2_bringup")
    map_yaml = os.path.join(share, "maps", "maze_steps.yaml")
    params = os.path.join(share, "config", "go2_terrain_nav2.yaml")
    rviz_cfg = os.path.join(share, "config", "go2_terrain.rviz")

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_share, "launch", "navigation_launch.py")),
        launch_arguments={
            "use_sim_time": "false",
            "params_file": params,
            "autostart": "true",
            "use_composition": "False",
        }.items(),
    )
    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[{"yaml_filename": map_yaml, "use_sim_time": False}],
    )
    map_lifecycle = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_map",
        output="screen",
        parameters=[{"autostart": True, "node_names": ["map_server"], "use_sim_time": False}],
    )
    static_map_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
        output="screen",
    )
    go2 = Node(
        package="go2_mujoco_locomotion",
        executable="policy_node",
        name="go2_locomotion",
        output="screen",
        parameters=[{
            "config_path": os.path.join(share, "config", "go2_locomotion.yaml"),
            "xml_path": os.path.join(share, "models", "maze_steps.xml"),
            "policy_path": os.path.join(share, "models", "policy.pt"),
            "cmd_topic": "/cmd_vel",
            "max_vx": 1.50,
            "max_wz": 0.50,
            "hold_when_zero": True,
            "hold_delay": 0.8,
            "enable_viewer": ParameterValue(LaunchConfiguration("mujoco_viewer"), value_type=bool),
        }],
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_cfg],
        condition=IfCondition(LaunchConfiguration("rviz")),
        output="screen",
    )
    return LaunchDescription([
        DeclareLaunchArgument("mujoco_viewer", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true"),
        static_map_odom,
        map_server,
        map_lifecycle,
        go2,
        TimerAction(period=3.0, actions=[nav2]),
        TimerAction(period=6.0, actions=[rviz]),
    ])
