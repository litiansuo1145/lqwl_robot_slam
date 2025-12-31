import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 定位到我们在 src 下创建的包
    pkg_share = '/home/ros2/ros2_ws/src/my_slam' 
    # 注意：上面这个路径建议写绝对路径，防止 colcon 没编译好找不到
    
    # 配置文件路径
    configuration_directory = os.path.join(pkg_share, 'config')
    configuration_basename = 'lidar_2d.lua'

    return LaunchDescription([
        # 启动 Cartographer 节点
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': False}],
            arguments=[
                '-configuration_directory', configuration_directory,
                '-configuration_basename', configuration_basename]),

        # 启动地图栅格化节点 (把数据变成 Rviz 能看的图)
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            output='screen',
            parameters=[{'use_sim_time': False}],
            arguments=['-resolution', '0.05', '-publish_period_sec', '1.0']),
            
        # 这是一个静态 TF 变换：把雷达跟车体绑在一起
        # 假设雷达就在车中心，没有偏移
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser']
        ),
    ])
