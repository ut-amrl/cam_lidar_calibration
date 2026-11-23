"""Assess saved captures without ROS 1, matplotlib, or a GUI."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackagePrefix


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("csv", description="Calibration CSV beside poses.csv and camera_info.yaml"),
        ExecuteProcess(cmd=["python3", PathJoinSubstitution([
            FindPackagePrefix("cam_lidar_calibration"), "lib", "cam_lidar_calibration", "assess_jazzy.py"
        ]), LaunchConfiguration("csv")], output="screen"),
    ])
