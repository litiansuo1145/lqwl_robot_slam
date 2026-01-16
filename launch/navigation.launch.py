import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 获取路径
    pkg_my_slam = get_package_share_directory('my_slam')
    pkg_driver = get_package_share_directory('car_driver')
    pkg_nav2 = get_package_share_directory('nav2_bringup')
    
    urdf_file = os.path.join(pkg_my_slam, 'urdf', 'robot.urdf')
    map_file = os.path.join(pkg_my_slam, 'maps', 'my_map.yaml')
    nav_params = os.path.join(pkg_my_slam, 'config', 'nav2_params.yaml')

    # 1. 机器人模型
    rsp_node = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[{'robot_description': open(urdf_file, 'r').read()}]
    )

    # 2. 硬件驱动
    car_node = Node(package='car_driver', executable='carnode', parameters=[{'port': '/dev/dock_32car'}])
    
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('sllidar_ros2'), 'launch', 'sllidar_c1_launch.py')),
        launch_arguments={'serial_port': '/dev/usb2_lidar'}.items()
    )
    
    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('witmotion_ros2'), 'launch', 'witmotion.launch.py')),
        launch_arguments={'port': '/dev/dock_imu'}.items()
    )
    # 3. 核心导航 (带地图和定位)
    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_nav2, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'use_sim_time': 'false',
            'map': map_file, 
            'params_file': nav_params,
            'autostart': 'true'
        }.items()
    )

    # 4. 自动归零
    auto_reset = TimerAction(
        period=2.0,
        actions=[ExecuteProcess(cmd=['ros2', 'topic', 'pub', '--once', '/num_cmd', 'std_msgs/msg/Int32', '{data: 23}'])]
    )

    return LaunchDescription([
        rsp_node, car_node, lidar_launch, imu_launch,
        TimerAction(period=5.0, actions=[nav_launch]),
        auto_reset
    ])