import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 路径定义
    pkg_my_slam = get_package_share_directory('my_slam')   
    pkg_driver = get_package_share_directory('car_driver') 
    pkg_sllidar = get_package_share_directory('sllidar_ros2')
    pkg_witmotion = get_package_share_directory('witmotion_ros2')
    pkg_nav2 = get_package_share_directory('nav2_bringup')
    
    # URDF 路径
    urdf_file = os.path.join(pkg_my_slam, 'urdf', 'robot.urdf')

    # 读取 URDF 内容
    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    # ================= 2. 机器人状态发布  =================
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    # ================= 3. 硬件启动 (立即执行) =================
    # 底盘 C++ 驱动
    car_node = Node(
        package='car_driver', executable='carnode', name='carnode',
        parameters=[{'port': '/dev/usb_port2'}]
    )
    # 雷达 
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_sllidar, 'launch', 'sllidar_c1_launch.py')),
        launch_arguments={'serial_port': '/dev/usb_port3', 'scan_mode': 'Standard'}.items()
    )
    # IMU 
    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_witmotion, 'launch', 'witmotion.launch.py')),
        launch_arguments={'port': '/dev/usb_imu'}.items()
    )

    # ================= 4. SLAM 定位 (延迟 3 秒) =================
    slam_node = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='cartographer_ros', executable='cartographer_node', name='cartographer_node',
                arguments=['-configuration_directory', os.path.join(pkg_my_slam, 'config'),
                           '-configuration_basename', 'lidar_2d.lua'],
                remappings=[('/imu', '/imu/data'), ('/odom', '/odom')]
            ),
            Node(package='cartographer_ros', executable='cartographer_occupancy_grid_node',
                 arguments=['-resolution', '0.05', '-publish_period_sec', '1.0'])
        ]
    )

    # ================= 5. Nav2 导航堆栈 (延迟 6 秒) =================
    nav_params = os.path.join(pkg_my_slam, 'config', 'nav2_params.yaml')
    nav_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_nav2, 'launch', 'navigation_launch.py')),
        launch_arguments={'params_file': nav_params, 'use_sim_time': 'false'}.items()
    )
    
    # 包装延迟
    delayed_nav_node = TimerAction(period=6.0, actions=[nav_node])

    # ================= 6. 启动时自动触发归零 =================
    auto_reset = TimerAction(
        period=2.0,
        actions=[ExecuteProcess(cmd=['ros2', 'topic', 'pub', '--once', '/num_cmd', 'std_msgs/msg/Int32', '{data: 23}'], output='screen')]
    )

    # ================= 7. 返回启动列表 =================
    return LaunchDescription([
        rsp_node,    
        car_node, 
        lidar_launch, 
        imu_launch,
        slam_node, 
        delayed_nav_node, 
        auto_reset
    ])