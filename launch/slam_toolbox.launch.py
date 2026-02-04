import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction,GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.actions import Node, SetRemap # 需要导入 SetRemap


def generate_launch_description():
    pkg_my_slam = get_package_share_directory('my_slam')
    pkg_driver = get_package_share_directory('car_driver')
    pkg_sllidar = get_package_share_directory('rplidar_ros')
    pkg_witmotion = get_package_share_directory('witmotion_ros2')
    config_file = '/home/ros2/ros2_ws/src/my_slam/config/laser_filter_config.yaml'

    # URDF 内容
    urdf_file = os.path.join(pkg_my_slam, 'urdf', 'robot.urdf')
    robot_desc = open(urdf_file, 'r').read()

    # 1. 机器人状态发布
    rsp_node = Node(package='robot_state_publisher', executable='robot_state_publisher',
                    parameters=[{'robot_description': robot_desc}])

    # 2. C++ 驱动节点
     # 启动 py_pkg 包中的 carnode.py - 创建小车控制节点
    usbcar_node = Node(
        # 节点所属的功能包名
        package='py_pkg',
        # 可执行文件的名称（carnode）
        executable='usbcar',
        # 节点的名称
        name='usbcar',
        # 输出到屏幕（控制台），便于调试
        output='screen',
        parameters=[{
            'publish_tf': False}]
    )
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
            parameters=[{'linear_scale': 0.8, 'angular_scale': 2.0}]
        )

    # 3. 雷达与 IMU 驱动
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_sllidar, 'launch', 'rplidar_c1_launch.py')
        ),
        launch_arguments={
            'serial_port': '/dev/usb2_lidar',
            'frame_id': 'laser',
            'scan_mode': 'Standard',
        }.items()
    )
    laser_filter_node = Node(
        package='py_pkg',
        executable='laser_filter',
        name='laser_filter',
        output='screen'
    )
    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_witmotion, 'launch', 'witmotion.launch.py')),
        launch_arguments={'port': '/dev/dock_imu'}.items()
    )
     # EKF 节点
    ekf_config = os.path.join(pkg_my_slam, 'config', 'ekf.yaml')
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config]
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
        usbcar_node,
        lidar_launch,
        laser_filter_node,
        imu_launch,
        ekf_node,
        joynode,
        teleop_node,
        # 延迟 3 秒启动 SLAM 确保 TF 树建立
        TimerAction(period=3.0, actions=[slam_node])
    ])