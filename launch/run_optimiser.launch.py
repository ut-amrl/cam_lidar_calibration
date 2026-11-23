"""Native Jazzy capture with optional RViz and explicit session paths."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare("cam_lidar_calibration")
    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=PathJoinSubstitution([share, "cfg", "params.yaml"])),
        DeclareLaunchArgument("rviz_config", default_value=PathJoinSubstitution([share, "rviz", "cam_lidar_calibration.rviz"])),
        DeclareLaunchArgument("gui", default_value="false"),
        DeclareLaunchArgument("decode_compressed", default_value="false"),
        DeclareLaunchArgument("compressed_topic", default_value="/camera/rgb/image_raw/compressed"),
        DeclareLaunchArgument("image_topic", default_value="/calibration/image_raw"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("output_dir", default_value="calibration_data"),
        DeclareLaunchArgument("num_lowestvoq", default_value="10"),
        Node(package="cam_lidar_calibration", executable="feature_extraction_node",
             name="feature_extraction", output="screen", parameters=[LaunchConfiguration("params_file"), {
                 "import_samples": False,
                 "output_dir": LaunchConfiguration("output_dir"),
                 "num_lowestvoq": ParameterValue(LaunchConfiguration("num_lowestvoq"), value_type=int),
                 "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
             }]),
        Node(package="rviz2", executable="rviz2", name="rviz2",
             arguments=["-d", LaunchConfiguration("rviz_config")], condition=IfCondition(LaunchConfiguration("gui"))),
        Node(package="cam_lidar_calibration", executable="decompress_image.py",
             condition=IfCondition(LaunchConfiguration("decode_compressed")), parameters=[{
                 "compressed_topic": LaunchConfiguration("compressed_topic"),
                 "image_topic": LaunchConfiguration("image_topic"),
                 "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
             }]),
    ])
