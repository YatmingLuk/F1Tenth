import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 1. VESC 驅動
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

	# 2. Hokuyo 雷達驅動
    urg_node = Node(
        package='urg_node',
        executable='urg_node_driver',
        name='urg_node',
        parameters=[{
            'serial_port': '/dev/sensors/hokuyo', 
            # 暫時拿掉 frame_id 設定，讓它噴它最想噴的 laser
        }],
        remappings=[
            ('scan', '/scan'), 
        ]
    )
    # 3. 靜態座標轉換：ego_racecar/base_link -> ego_racecar/laser
    static_tf_base_to_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_laser',
        arguments=['0.11', '0', '0.12', '0', '0', '0', 'base_link', 'laser']
    )
    
    # 4. 靜態座標轉換：ego_racecar/base_link -> imu
    static_tf_base_to_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_imu',
        arguments=['-0.1', '0', '0.12', '0', '0', '0', 'base_link', 'imu']
    )

    return LaunchDescription([
 #       vesc_driver,
        urg_node,
        static_tf_base_to_laser,
 #       static_tf_base_to_imu
    ])
