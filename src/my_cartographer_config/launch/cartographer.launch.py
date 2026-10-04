import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 定義路徑：指向你 config 資料夾中的 Lua 設定檔
    pkg_share_dir = get_package_share_directory('my_cartographer_config')
    lua_config_path = os.path.join(pkg_share_dir, 'config')
    
    # 2. 定義 Cartographer 核心節點
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{
            'use_sim_time': False  # 實體車運作，不使用模擬時間
        }],
        # 關鍵參數：指定 Lua 設定檔所在資料夾與檔案名稱
        arguments=[
            '-configuration_directory', lua_config_path,
            '-configuration_basename', 'hokuyo_2d.lua'
        ],
        # Connect the physical sensor topics to Cartographer's expected names.
        remappings=[
            ('scan', '/scan'),
            ('imu', '/sensors/imu/raw'),
        ]
    )

    # 3. 定義地圖節點 (Occupancy Grid)，負責將 SLAM 結果轉為常見的 2D 柵格地圖
    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'resolution': 0.05,             # 地圖解析度：5cm 一格
            'publish_period_sec': 1.0       # 每秒更新一次地圖
        }]
    )

    return LaunchDescription([
        cartographer_node,
        occupancy_grid_node
    ])
