from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    static_base_to_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_lidar',
        arguments=[
            '0', '0', '0',      # x y z
            '0', '0', '0',      # roll pitch yaw
            'base_link',        # parent frame
            'mid360_link',      # child frame
        ],
    )

    static_base_to_body = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_body',
        arguments=[
            '0', '0', '0',
            '0', '0', '0',
            'base_link',
            'body',
        ],
    )

    static_base_to_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_imu',
        arguments=[
            '0', '0', '0',
            '0', '0', '0',
            'base_link',
            'imu_link',
        ],
    )

    return LaunchDescription([
        static_base_to_lidar,
        static_base_to_body,
        static_base_to_imu,
    ])

