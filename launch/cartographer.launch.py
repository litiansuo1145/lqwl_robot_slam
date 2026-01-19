import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    pkg_my_slam = get_package_share_directory('my_slam')
    pkg_sllidar = get_package_share_directory('sllidar_ros2')
    pkg_witmotion = get_package_share_directory('witmotion_ros2')
    pkg_car_driver = get_package_share_directory('car_driver')
    pkg_my_launch = get_package_share_directory('my_launch_pkg') 
    # ---------- 路径 ----------
    config_dir = os.path.join(pkg_my_slam, 'config')
    carto_config = 'lidar_2d.lua'
    ekf_config = os.path.join(pkg_my_slam, 'config', 'ekf.yaml')
    urdf_file = os.path.join(pkg_my_slam, 'urdf', 'robot.urdf')

    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    # ---------- robot_state_publisher ----------
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    # ---------- 底盘 ----------
    car_node = Node(
        package='car_driver',
        executable='carnode',
        output='screen',
        parameters=[{
            'port': '/dev/dock_32car',
            'baudrate': 115200,
            'publish_tf': False
        }],
    )
    # 手柄驱动节点
    joynode = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'dev': '/dev/input/js0',
            'deadzone': 0.05,
            'autorepeat_rate': 10.0,
        }]
    )
    # 操作控制节点
    teleop_node = Node(
            package='py_pkg',
            executable='teleop_control_node',
            parameters=[{'linear_scale': 0.4, 'angular_scale': 1.0}]
        )
    # ---------- 雷达 ----------
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_sllidar, 'launch', 'sllidar_c1_launch.py')
        ),
        launch_arguments={
            'serial_port': '/dev/usb2_lidar',
            'frame_id': 'laser',
            'scan_mode': 'Standard'
        }.items()
    )

    # ---------- IMU ----------
    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_witmotion, 'launch', 'witmotion.launch.py')
        ),
        launch_arguments={'port': '/dev/dock_imu'}.items()
    )
    # ---------- EKF ----------
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config]
    )

    # ---------- Cartographer ----------
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        output='screen',
        arguments=[
            '-configuration_directory', config_dir,
            '-configuration_basename', carto_config
        ],
        remappings=[
            ('/imu', '/imu/data'),
            ('/odom', '/odometry/filtered')  
        ]
    )

    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        output='screen',
        arguments=['-resolution', '0.05', '-publish_period_sec', '1.0']
    )

    return LaunchDescription([
        rsp_node,
        car_node,
        joynode,
        teleop_node,
        lidar_launch,
        imu_launch,
        TimerAction(
            period=1.5,
            actions=[ekf_node]
        ),
        TimerAction(
            period=3.0,
            actions=[cartographer_node, occupancy_grid_node]
        )
    ])
