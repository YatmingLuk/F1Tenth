from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # VESC is started separately through its configurable driver launch file.
    # Hokuyo LiDAR driver.
    urg_node = Node(
        package='urg_node',
        executable='urg_node_driver',
        name='urg_node',
        parameters=[{
            'serial_port': '/dev/sensors/hokuyo', 
            # Keep the driver's native "laser" frame.
        }],
        remappings=[
            ('scan', '/scan'), 
        ]
    )
    # Static transform: base_link -> laser.
    static_tf_base_to_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_laser',
        arguments=['0.11', '0', '0.12', '0', '0', '0', 'base_link', 'laser']
    )
    
    # Static transform: base_link -> imu. The translation matches the current
    # vehicle installation; verify the rotation against the physical mounting.
    static_tf_base_to_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_imu',
        arguments=['-0.1', '0', '0.12', '0', '0', '0', 'base_link', 'imu']
    )

    return LaunchDescription([
        urg_node,
        static_tf_base_to_laser,
        static_tf_base_to_imu,
    ])
