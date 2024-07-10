import os
from pathlib import Path
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable, ExecuteProcess)
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    usb_node = IncludeLaunchDescription(PythonLaunchDescriptionSource(get_package_share_directory('hobot_usb_cam') + '/launch/hobot_usb_cam.launch.py'),
                                        launch_arguments={'usb_image_width': '640', 'usb_image_height': '480',
                                                            'usb_video_device': '/dev/video8'}.items())
    
    base_node = IncludeLaunchDescription(PythonLaunchDescriptionSource(get_package_share_directory('origincar_base') + '/launch/origincar_bringup.launch.py'))

    qrdecode_node = Node(
        package='qrdecode',
        executable='qrdecode',
        output='screen',
        
    )

    line_follow_node = Node(
        package='line_follow',
        executable='line_follow',
        output='screen',
        
    )

    ros2_web_bridge_node = ExecuteProcess(
        cmd=[
            'ros2',
            'run',
            'rosbridge_server',
            'rosbridge_websocket_launch.xml',
        ],
        output='screen',
    )

    return LaunchDescription([
        usb_node,
        base_node,
        qrdecode_node,
        line_follow_node,
        ros2_web_bridge_node
    ])

