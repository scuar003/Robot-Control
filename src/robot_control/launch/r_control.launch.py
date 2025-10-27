import launch
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch Zenoh Daemon
        # ExecuteProcess(
        #     cmd=['ros2', 'run', 'rmw_zenoh_cpp', 'rmw_zenohd'],
        #     output='screen'
        # ),
        
        # Launch robot control node
        ExecuteProcess(
            cmd=['ros2', 'run', 'joy', 'joy_node'],
            output='screen'
        ),
        ExecuteProcess(
            cmd=['ros2', 'run', 'robot_control', 'controls'],
            output='screen'
        ),
        ExecuteProcess(
            cmd=['ros2', 'run', 'robot_control', 'joy_to_cmd' ],
            output='screen'
        )
        
        # Launch USB Camera node
        # ExecuteProcess(
        #     cmd=['ros2', 'run', 'usb_cam', 'usb_cam_node_exe'],
        #     output='screen'
        # ),
        
        # Launch line follower node
        # ExecuteProcess(
        #     cmd=['ros2', 'run', 'robot_control', 'line_follower'],
        #     output='screen'
        # ),
    ])
