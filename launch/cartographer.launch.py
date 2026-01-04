import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 获取各个功能包的路径
    # 注意：请确保这些包名在你的系统中是正确的
    pkg_my_slam = get_package_share_directory('my_slam')
    pkg_sllidar = get_package_share_directory('sllidar_ros2')
    pkg_witmotion = get_package_share_directory('witmotion_ros2')

    # 2. Cartographer 配置文件路径
    configuration_directory = os.path.join(pkg_my_slam, 'config')
    configuration_basename = 'lidar_2d.lua'

    # ================= 3. 启动雷达驱动 (usb_port3) =================
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_sllidar, 'launch', 'sllidar_c1_launch.py')
        ),
        launch_arguments={
            'serial_port': '/dev/usb_port3',
            'frame_id': 'laser'
        }.items()
    )

    # ================= 4. 启动 IMU 驱动 (usb_imu) =================
    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_witmotion, 'launch', 'witmotion.launch.py')
        ),
        launch_arguments={
            'port': '/dev/usb_imu'
        }.items()
    )

    # ================= 5. 静态 TF 变换 (替代 URDF) =================
    # 这里的顺序必须是：父级 -> 子级
    
    # A. base_footprint 到 base_link (高度补偿)
    tf_footprint_to_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_footprint_to_base_link',
        arguments=['0', '0', '0.08', '0', '0', '0', 'base_footprint', 'base_link']
    )

    # B. base_link 到 laser (雷达中心偏移 5cm，且左偏 90度 1.5708)
    tf_base_to_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_laser',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser']
    )


    # ================= 6. 启动 Cartographer 核心节点 =================
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': False}],
        arguments=[
            '-configuration_directory', configuration_directory,
            '-configuration_basename', configuration_basename
        ],
        remappings=[
            ('/imu', '/imu/data'), # 匹配你之前 ros2 topic list 看到的名称
            ('/odom', '/odom')
        ]
    )

    # 地图栅格化节点 (为了在 RViz 看到黑白地图)
    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': False}],
        arguments=['-resolution', '0.05', '-publish_period_sec', '1.0']
    )

    # ================= 7. 返回启动描述 =================
    return LaunchDescription([
        lidar_launch,
        imu_launch,
        tf_footprint_to_link,
        tf_base_to_laser,
        # 延迟 2 秒启动算法，等待驱动稳定
        TimerAction(
            period=2.0,
            actions=[cartographer_node, occupancy_grid_node]
        )
    ])