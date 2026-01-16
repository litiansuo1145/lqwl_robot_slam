import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 1. 获取包路径
    pkg_my_slam = get_package_share_directory('my_slam')
    pkg_driver = get_package_share_directory('car_driver')
    pkg_sllidar = get_package_share_directory('sllidar_ros2')
    pkg_witmotion = get_package_share_directory('witmotion_ros2')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    # 2. 声明地图和参数路径（请确保你的地图文件名正确）
    map_yaml_file = os.path.join(pkg_my_slam, 'maps', 'my_map.yaml')
    nav2_params_file = os.path.join(pkg_my_slam, 'config', 'mppi_nav_params.yaml')

    # 3. 机器人状态发布 (URDF)
    urdf_file = os.path.join(pkg_my_slam, 'urdf', 'robot.urdf')
    with open(urdf_file, 'r') as f:
        robot_desc = f.read()
    
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    # 4. C++ 驱动节点 (使用你指定的 dock_32car 端口)
    car_node = Node(
        package='car_driver',
        executable='carnode',
        name='carnode',
        output='screen',
        parameters=[{'port': '/dev/dock_32car'}]
    )

    # 5. 雷达驱动 (使用你指定的 usb2_lidar 端口)
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_sllidar, 'launch', 'sllidar_c1_launch.py')
        ),
        launch_arguments={
            'serial_port': '/dev/usb2_lidar', 
            'scan_mode': 'Standard',
            'frame_id': 'laser'
        }.items()
    )

    # 6. IMU 驱动 (使用你指定的 dock_imu 端口)
    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_witmotion, 'launch', 'witmotion.launch.py')
        ),
        launch_arguments={'port': '/dev/dock_imu'}.items()
    )

    # 7. Nav2 导航堆栈 (包含 map_server, amcl 定位, planner, controller)
    # 使用官方 bringup_launch.py 实现一键拉起
    nav2_bringup_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')
    ),
    launch_arguments={
        'map': map_yaml_file,
        'use_sim_time': 'false',
        'params_file': nav2_params_file,
        'autostart': 'true',


        'slam': 'false',
        'localization': 'true'
    }.items()
    )
    # 8. 返回描述，依然建议使用延迟确保硬件稳定
    return LaunchDescription([
        rsp_node,
        car_node,
        lidar_launch,
        imu_launch,
        # 延迟 5 秒启动 Nav2，等待地图加载和传感器同步
        TimerAction(
            period=5.0,
            actions=[nav2_bringup_launch]
        )
    ])