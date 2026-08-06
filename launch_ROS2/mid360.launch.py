from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("livox_ros_driver2")

    default_user_config = PathJoinSubstitution([pkg_share, "config", "MID360_config.json"])
    default_rviz_config = PathJoinSubstitution([pkg_share, "config", "display_point_cloud_ROS2.rviz"])

    declared_args = [
        DeclareLaunchArgument("xfer_format", default_value="0"),
        DeclareLaunchArgument("multi_topic", default_value="0"),
        DeclareLaunchArgument("data_src", default_value="0"),
        DeclareLaunchArgument("publish_freq", default_value="10.0"),
        DeclareLaunchArgument("output_data_type", default_value="0"),
        DeclareLaunchArgument("frame_id", default_value="mid360_link"),
        DeclareLaunchArgument("lvx_file_path", default_value=""),
        DeclareLaunchArgument("cmdline_input_bd_code", default_value="livox0000000001"),
        DeclareLaunchArgument("user_config_path", default_value=default_user_config),
        DeclareLaunchArgument("lidar_qos_reliability", default_value="best_effort"),
        DeclareLaunchArgument("lidar_qos_depth", default_value="2"),
        DeclareLaunchArgument("imu_qos_reliability", default_value="best_effort"),
        DeclareLaunchArgument("imu_qos_depth", default_value="50"),
        DeclareLaunchArgument("raw_packet_queue_capacity", default_value="512"),
        DeclareLaunchArgument("imu_packet_queue_capacity", default_value="50"),
        DeclareLaunchArgument("lidar_frame_queue_capacity", default_value="2"),
        DeclareLaunchArgument("ptp_max_offset_seconds", default_value="1.0"),
        DeclareLaunchArgument("visualization_publish_frequency", default_value="4.0"),

        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("rviz_config", default_value=default_rviz_config),

        DeclareLaunchArgument("container_name", default_value="livox_container"),
        DeclareLaunchArgument("use_intra_process", default_value="true"),
    ]

    livox_container = ComposableNodeContainer(
        name=LaunchConfiguration("container_name"),
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        output="screen",
        composable_node_descriptions=[
            ComposableNode(
                package="livox_ros_driver2",
                plugin="livox_ros::DriverNode",
                name="livox_ros_driver2",
                parameters=[{
                    "xfer_format": LaunchConfiguration("xfer_format"),
                    "multi_topic": LaunchConfiguration("multi_topic"),
                    "data_src": LaunchConfiguration("data_src"),
                    "publish_freq": LaunchConfiguration("publish_freq"),
                    "output_data_type": LaunchConfiguration("output_data_type"),
                    "frame_id": LaunchConfiguration("frame_id"),
                    "lvx_file_path": LaunchConfiguration("lvx_file_path"),
                    "user_config_path": LaunchConfiguration("user_config_path"),
                    "cmdline_input_bd_code": LaunchConfiguration("cmdline_input_bd_code"),
                    "lidar_qos_reliability": LaunchConfiguration("lidar_qos_reliability"),
                    "lidar_qos_depth": LaunchConfiguration("lidar_qos_depth"),
                    "imu_qos_reliability": LaunchConfiguration("imu_qos_reliability"),
                    "imu_qos_depth": LaunchConfiguration("imu_qos_depth"),
                    "raw_packet_queue_capacity": LaunchConfiguration("raw_packet_queue_capacity"),
                    "imu_packet_queue_capacity": LaunchConfiguration("imu_packet_queue_capacity"),
                    "lidar_frame_queue_capacity": LaunchConfiguration("lidar_frame_queue_capacity"),
                    "ptp_max_offset_seconds": LaunchConfiguration("ptp_max_offset_seconds"),
                }],
                extra_arguments=[{
                    "use_intra_process_comms": LaunchConfiguration("use_intra_process"),
                }],
            ),
            ComposableNode(
                package="livox_ros_driver2",
                plugin="livox_ros::PointCloudThrottleNode",
                name="livox_pointcloud_viz_throttle",
                remappings=[
                    ("input", "/livox/lidar"),
                    ("output", "/livox/lidar_viz_input"),
                ],
                parameters=[{
                    "max_rate_hz": LaunchConfiguration("visualization_publish_frequency"),
                }],
                extra_arguments=[{
                    "use_intra_process_comms": LaunchConfiguration("use_intra_process"),
                }],
            ),
            ComposableNode(
                package="point_cloud_transport",
                plugin="point_cloud_transport::Republisher",
                name="livox_pointcloud_compressor",
                remappings=[
                    ("in", "/livox/lidar_viz_input"),
                    ("/out/draco", "/livox/lidar_viz/draco"),
                ],
                parameters=[{
                    "in_transport": "raw",
                    "out.enable_pub_plugins": ["point_cloud_transport/draco"],
                    "qos_overrides./livox/lidar_viz_input.subscription.history": "keep_last",
                    "qos_overrides./livox/lidar_viz_input.subscription.depth": 1,
                    "qos_overrides./livox/lidar_viz_input.subscription.reliability": "best_effort",
                    "qos_overrides./livox/lidar_viz/draco.publisher.history": "keep_last",
                    "qos_overrides./livox/lidar_viz/draco.publisher.depth": 1,
                    "qos_overrides./livox/lidar_viz/draco.publisher.reliability": "best_effort",
                    # Livox also carries uint8 tag/line and a float64 timestamp.
                    # Forced KD-tree quantization cannot encode that mixed layout.
                    "livox.lidar_viz.draco.encode_method": 0,
                    "livox.lidar_viz.draco.force_quantization": False,
                    "livox.lidar_viz.draco.encode_speed": 5,
                    "livox.lidar_viz.draco.decode_speed": 5,
                    "livox.lidar_viz.draco.deduplicate": True,
                }],
                extra_arguments=[{
                    "use_intra_process_comms": LaunchConfiguration("use_intra_process"),
                }],
            ),
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    return LaunchDescription([
        *declared_args,
        livox_container,
        rviz,
    ])
