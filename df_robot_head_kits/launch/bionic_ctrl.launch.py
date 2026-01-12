from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='df_robot_head_kits',
            executable='bionic_ctrl_node',
            name='bionic_ctrl_node',
            output='screen',
            parameters=[],
	    arguments=['--ros-args', '--log-level', 'info']
        )
    ])
