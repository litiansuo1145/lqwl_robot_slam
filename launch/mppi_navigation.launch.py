import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_my_slam = get_package_share_directory('my_slam')
    pkg_nav2 = get_package_share_directory('nav2_bringup')
    map_yaml = '/home/ros2/ros2_ws/src/my_slam/maps/my_map.yaml'
    # 加载硬件驱动
    hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_my_slam, 'launch', 'cartographer.launch.py'))
    )

    # 配置 MPPI 导航参数
    nav_params = os.path.join(pkg_my_slam, 'config', 'mppi_nav_params.yaml')
    
    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_nav2, 'launch', 'navigation_launch.py')),
        launch_arguments={'params_file': nav_params, 'use_sim_time': 'false'}.items()
    )
    map_server = Node(
    package='nav2_map_server',
    executable='map_server',
    name='map_server',
    output='screen',
    parameters=[{
        'yaml_filename': map_yaml,
        'use_sim_time': False
    }]
    )
    return LaunchDescription([
        hardware_launch,
        map_server,
        # 延迟 5 秒让 SLAM 稳定后再拉起 MPPI
        TimerAction(period=5.0, actions=[nav_launch])
    ])