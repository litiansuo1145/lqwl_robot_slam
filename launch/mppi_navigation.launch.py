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
    pkg_driver = get_package_share_directory('py_pkg')
    pkg_sllidar = get_package_share_directory('rplidar_ros')
    pkg_witmotion = get_package_share_directory('witmotion_ros2')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    # 2. 声明地图和参数路径
    map_yaml_file = os.path.join(pkg_my_slam, 'maps', 'my_map_05.yaml')
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

    # 4. C++ 驱动节点
    car_node = Node(
        package='py_pkg',
        executable='carnode',
        name='carnode',
        output='screen',
        parameters=[{'port': '/dev/dock_32car', 'publish_tf': True, }]
    )

    # 5. 雷达驱动 
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_sllidar, 'launch', 'rplidar_c1_launch.py')
        ),
        launch_arguments={
            'serial_port': '/dev/usb2_lidar', 
            'scan_mode': 'Standard',
            'frame_id': 'laser'
        }.items()
    )
    
    # 6. IMU 驱动 
    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_witmotion, 'launch', 'witmotion.launch.py')
        ),
        launch_arguments={'port': '/dev/dock_imu'}.items()
    )

    # 7. Nav2 导航堆栈 
    nav2_bringup_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')
    ),
    launch_arguments={
        'map': map_yaml_file,
        'use_sim_time': 'False',
        'params_file': nav2_params_file,
        'autostart': 'True',
        'slam': 'False',
        'localization': 'True'
    }.items()
    )
    topic_nav_node = Node(
        package='py_pkg',              # 你的 Python 包名
        executable='topic_nav_node',   # setup.py 里写的那个名字
        name='topic_nav_node',         # 节点名
        output='screen',               # 将日志打印到屏幕
        parameters=[{'use_sim_time': False}] # 可选参数
    )
    #小智节点
    xiaozhi_node = Node(
        package='xiaozhi',              # 你的 Python 包名
        executable='xiaozhi_node',   # setup.py 里写的那个名字
        name='xiaozhi_node',         # 节点名
        output='screen',               # 将日志打印到屏幕
        parameters=[{'use_sim_time': False}] # 可选参数
    )
    # 8. 返回描述
    return LaunchDescription([
        rsp_node,
        car_node,
        lidar_launch,
        imu_launch,
        # 延迟 3 秒启动 Nav2
        TimerAction(
            period=3.0,
            actions=[nav2_bringup_launch]
        ),
        topic_nav_node,
        xiaozhi_node
    ])