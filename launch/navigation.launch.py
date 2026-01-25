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
   # 启动 py_pkg 包中的 carnode.py - 创建小车控制节点
    car_node = Node(
        # 节点所属的功能包名
        package='py_pkg',
        # 可执行文件的名称（carnode）
        executable='carnode',
        # 节点的名称
        name='carnode',
        # 输出到屏幕（控制台），便于调试
        output='screen',
        parameters=[{
            'publish_tf': True}]
    )
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('rplidar_ros'), 'launch', 'rplidar_c1_launch.py')),
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


    return LaunchDescription([
        rsp_node, car_node, lidar_launch, imu_launch,
        TimerAction(period=3.0, actions=[nav_launch]),
    ])