#!/usr/bin/env python3
import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg = get_package_share_directory('my_bot')

    # 1) URDF from Xacro
    xacro_file = os.path.join(pkg, 'description', 'robot.urdf.xacro')
    robot_desc = xacro.process_file(xacro_file).toxml()
    robot_params = [{'robot_description': robot_desc, 'use_sim_time': False}]

    # 2) RViz2 config
    rviz_config = os.path.join(pkg, 'config', 'hw_full.rviz')

    return LaunchDescription([
        # —————————————————————————
        # robot_state_publisher (URDF → TF)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=robot_params
        ),

        # joint_state_publisher (publishes joint_states → wheel TFs)
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen'
        ),

        # encoder_to_odom (publishes /joint_states, /odom, TF)
        Node(
            package='my_bot',
            executable='encoder_to_odom.py',
            name='encoder_to_odom',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'serial_port': '/dev/ttyUSB1',
                'baudrate': 57600,
                'ticks_per_rev': 1024,
                'wheel_radius': 0.02,
                'wheel_base': 0.36,
                'poll_rate': 20.0
            }]
        ),

        # USB camera via V4L2
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            output='screen',
            parameters=[
                {'video_device': '/dev/video2'},
                {'image_width': 640},
                {'image_height': 480},
                {'pixel_format': 'yuyv'}
            ]
        ),

        # teleop_twist_keyboard (publish /cmd_vel)
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop_keyboard',
            output='screen',
            prefix='xterm -e'
        ),
         #Lidar
        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            name='rplidar',
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'serial_baudrate': 115200,
                'frame_id': 'laser_frame',
                'angle_compensate': True,
                'scan_mode': 'Standard'
            }]
        ),
        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            emulate_tty=True,
            arguments=['-d', rviz_config]
        ),



    ])
 