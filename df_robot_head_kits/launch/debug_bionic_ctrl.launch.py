from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['gdb', '-ex', 'run', '--args',
                 '/home/be/df_robot_ws/install/df_robot_head_kits/lib/df_robot_head_kits/bionic_ctrl_node'],
            output='screen'
        )
    ])
