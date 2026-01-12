import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 获取各个功能包的路径
    pkg_my_slam = get_package_share_directory('my_slam')
    pkg_sllidar = get_package_share_directory('sllidar_ros2')
    pkg_witmotion = get_package_share_directory('witmotion_ros2')
    pkg_car_driver = get_package_share_directory('car_driver')

    # 2. 路径配置
    configuration_directory = os.path.join(pkg_my_slam, 'config')
    configuration_basename = 'lidar_2d.lua'
    urdf_file = os.path.join(pkg_my_slam, 'urdf', 'robot.urdf')

    # 3. 读取 URDF 文件内容 (供 robot_state_publisher 使用)
    with open(urdf_file, 'r') as f:
        robot_description_config = f.read()

    # ================= 4. 启动 robot_state_publisher (加载 URDF) =================
    # 它会自动发布 URDF 中定义的所有静态坐标 (base_footprint -> base_link -> laser/imu_link)
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_config}]
    )

    # ================= 5. 启动底盘 C++ 驱动节点 (usb_port2) =================
    car_node = Node(
        package='car_driver',
        executable='carnode',
        name='carnode',
        output='screen',
        parameters=[{
            'port': '/dev/usb_port2',
            'baudrate': 115200
        }]
    )

    # ================= 6. 启动雷达驱动 (usb_port3) =================
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_sllidar, 'launch', 'sllidar_c1_launch.py')
        ),
        launch_arguments={
            'serial_port': '/dev/usb_port3',
            'frame_id': 'laser',
            'scan_mode': 'Standard'
        }.items()
    )

    # ================= 7. 启动 IMU 驱动 (usb_imu) =================
    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_witmotion, 'launch', 'witmotion.launch.py')
        ),
        launch_arguments={
            'port': '/dev/usb_imu'
        }.items()
    )

    # ================= 8. 启动 Cartographer SLAM (延迟 3 秒) =================
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
            ('/imu', '/imu/data'),
            ('/odom', '/odom')
        ]
    )

    # 地图栅格化节点
    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': False}],
        arguments=['-resolution', '0.05', '-publish_period_sec', '1.0']
    )

    # ================= 9. 返回启动描述 =================
    return LaunchDescription([
        rsp_node,          # 1. 发布机器人模型坐标
        car_node,          # 2. 起底盘
        lidar_launch,      # 3. 起雷达
        imu_launch,        # 4. 起IMU
        
        # 延迟 3 秒启动算法，确保 TF 树已经由 rsp_node 建立完整
        TimerAction(
            period=3.0,
            actions=[cartographer_node, occupancy_grid_node]
        )
    ])