import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_my_slam = get_package_share_directory('my_slam')
    pkg_driver = get_package_share_directory('car_driver')
    pkg_sllidar = get_package_share_directory('sllidar_ros2')
    pkg_witmotion = get_package_share_directory('witmotion_ros2')

    # URDF 内容
    urdf_file = os.path.join(pkg_my_slam, 'urdf', 'robot.urdf')
    robot_desc = open(urdf_file, 'r').read()

    # 1. 机器人状态发布
    rsp_node = Node(package='robot_state_publisher', executable='robot_state_publisher',
                    parameters=[{'robot_description': robot_desc}])

    # 2. C++ 驱动节点
    car_node = Node(package='car_driver', executable='carnode', parameters=[{'port': '/dev/dock_32car'}])

    # 3. 雷达与 IMU 驱动
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_sllidar, 'launch', 'sllidar_c1_launch.py')),
        launch_arguments={'serial_port': '/dev/usb2_lidar', 'scan_mode': 'Standard'}.items()
    )
    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_witmotion, 'launch', 'witmotion.launch.py')),
        launch_arguments={'port': '/dev/dock_imu'}.items()
    )

    # 4. Slam Toolbox 异步节点
    slam_params = os.path.join(pkg_my_slam, 'config', 'slam_toolbox_params.yaml')
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params]
    )

    return LaunchDescription([
        rsp_node,
        car_node,
        lidar_launch,
        imu_launch,
        # 延迟 3 秒启动 SLAM 确保 TF 树建立
        TimerAction(period=3.0, actions=[slam_node])
    ])