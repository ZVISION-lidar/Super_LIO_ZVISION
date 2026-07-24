import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def flatten(prefix, value, out):
    if isinstance(value, dict):
        for key, child in value.items():
            flatten(f"{prefix}.{key}" if prefix else key, child, out)
    else:
        out[prefix] = value


def load_flat_yaml(path):
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    if "/**" in data:
        data = data["/**"].get("ros__parameters", {})
    out = {}
    flatten("", data, out)
    return out


def generate_launch_description():
    pkg_share = get_package_share_directory("super_lio")
    default_config_dir = os.path.join(pkg_share, "config")
    default_lio_config = os.path.join(default_config_dir, "zvision.yaml")
    default_localization_config = os.path.join(default_config_dir, "zvision_localization.yaml")
    default_rviz_config = os.path.join(pkg_share, "rviz", "zvision_localization.rviz")

    map_path = LaunchConfiguration("map")
    map_period = LaunchConfiguration("map_publish_period")
    use_sim_time = LaunchConfiguration("use_sim_time")
    rviz = LaunchConfiguration("rviz")

    localization_params = load_flat_yaml(default_localization_config)

    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("lio_config", default_value=default_lio_config),
        DeclareLaunchArgument("map", default_value="/home/tzyros2/map01.pcd"),
        DeclareLaunchArgument("map_publish_period", default_value="100.0"),
        DeclareLaunchArgument("rviz_config", default_value=default_rviz_config),


        Node(
            package="super_lio",
            executable="super_lio_node",
            name="super_lio_node",
            output="screen",
            parameters=[default_lio_config, {"use_sim_time": use_sim_time}],
        ),

        Node(
            package="super_lio",
            executable="global_localization.py",
            name="global_localization",
            output="screen",
            parameters=[localization_params, {"use_sim_time": use_sim_time}],
        ),
        Node(
            package="super_lio",
            executable="transform_fusion.py",
            name="transform_fusion",
            output="screen",
            parameters=[localization_params, {"use_sim_time": use_sim_time}],
        ),
        Node(
            package="super_lio",
            executable="map_publisher.py",
            name="map_publisher",
            output="screen",
            arguments=["--map", map_path, "--period", map_period],
            parameters=[{"use_sim_time": use_sim_time}],
        ),
        Node(
            condition=IfCondition(rviz),
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", LaunchConfiguration("rviz_config")],
            parameters=[{"use_sim_time": use_sim_time}],
        ),
    ])
