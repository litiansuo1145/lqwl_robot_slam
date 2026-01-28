
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
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
 

    return LaunchDescription([    
        joynode,
        teleop_node,
    ])
