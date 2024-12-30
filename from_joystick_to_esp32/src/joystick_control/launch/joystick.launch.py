import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    ld = LaunchDescription()

    # Nodo per il joystick
    ld.add_action(Node(
        package='joy',
        executable='joy_node',
        name='game_controller',
        parameters=[{
            'device_id': 1,
            'deadzone': 0.05,
            'autorepeat_rate': 0.0,
            'sticky_buttons': False,
            'coalesce_interval_ms': 1}],
     ))

    # Nodo per il controllo teleop
    ld.add_action(Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy',
        parameters=[{
            'require_enable_button': True,
            'enable_button': 5,
            'enable_turbo_button': -1,
            'axis_linear.x': 1,
            'axis_linear.y': 2,
            'axis_linear.z': 3,
            'scale_linear.x': 0.2,
            'scale_linear.y': 1.0,
            'scale_linear.z': 0.0,
            'axis_angular.yaw': 2,
            'scale_angular.yaw': 1.0,
            'publish_stamped_twist': False}],
     ))

    # Nodo per micro-ROS agent, eseguito con UDP6
    ld.add_action(ExecuteProcess(
        cmd=['bash', '-c', 'source ~/Desktop/agent/install/local_setup.bash && ros2 run micro_ros_agent micro_ros_agent udp6 --port 8888'],
        output='screen',
    ))

    return ld

