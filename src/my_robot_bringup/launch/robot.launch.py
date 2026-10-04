from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # The VESC driver is normally started separately; see docs/OPERATIONS.md.
    vesc_driver = Node(
        package='vesc_driver',
        executable='vesc_driver_node',
        name='vesc_driver_node',
        parameters=[{
            'port': '/dev/ttyACM1',
            'imu_frame_id': 'imu',
            'frame_id': 'imu',
            # 必須加入以下參數，否則輪子不會轉
            'speed_max': 30000.0,
            'speed_min': -30000.0,
            'servo_max': 0.85,
            'servo_min': 0.15,
        }]
    )

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
        # vesc_driver,
        urg_node,
        static_tf_base_to_laser,
        static_tf_base_to_imu,
    ])
