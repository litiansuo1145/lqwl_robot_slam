import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 路径定义
    pkg_my_slam = get_package_share_directory('my_slam')
    pkg_driver = get_package_share_directory('car_driver')
    pkg_sllidar = get_package_share_directory('sllidar_ros2')
    pkg_witmotion = get_package_share_directory('witmotion_ros2')
    pkg_nav2 = get_package_share_directory('nav2_bringup')

    # URDF 内容
    urdf_file = os.path.join(pkg_my_slam, 'urdf', 'robot.urdf')
    robot_desc = open(urdf_file, 'r').read()

    # ================= 2. 基础硬件 (立即启动) =================
    rsp_node = Node(package='robot_state_publisher', executable='robot_state_publisher',
                    parameters=[{'robot_description': robot_desc}])
    
    car_node = Node(package='car_driver', executable='carnode', parameters=[{'port': '/dev/usb_port2'}])
    
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_sllidar, 'launch', 'sllidar_c1_launch.py')),
        launch_arguments={'serial_port': '/dev/usb_port3', 'scan_mode': 'Standard'}.items()
    )
    
    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_witmotion, 'launch', 'witmotion.launch.py')),
        launch_arguments={'port': '/dev/usb_imu'}.items()
    )

    # ================= 3. Cartographer SLAM (关键：提供定位和地图) =================
    slam_node = Node(
        package='cartographer_ros', executable='cartographer_node',
        arguments=['-configuration_directory', os.path.join(pkg_my_slam, 'config'),
                   '-configuration_basename', 'lidar_2d.lua'],
        remappings=[('/imu', '/imu/data'), ('/odom', '/odom')]
    )
    
    grid_node = Node(package='cartographer_ros', executable='cartographer_occupancy_grid_node',
                     arguments=['-resolution', '0.05', '-publish_period_sec', '1.0'])

    # ================= 4. Nav2 (只启动规划和控制，不启动AMCL) =================
    nav_params = os.path.join(pkg_my_slam, 'config', 'slam_nav_params.yaml')
    
    # 关键：调用 navigation_launch.py 而不是 bringup_launch.py
    # 因为 bringup 会强制找地图文件并启动 AMCL
    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_nav2, 'launch', 'navigation_launch.py')),
        launch_arguments={'params_file': nav_params, 'use_sim_time': 'false'}.items()
    )

    return LaunchDescription([
        rsp_node,
        car_node,
        lidar_launch,
        imu_launch,
        # 延迟启动 SLAM
        TimerAction(period=3.0, actions=[slam_node, grid_node]),
        # 延迟启动 Nav2
        TimerAction(period=6.0, actions=[nav_launch])
    ])