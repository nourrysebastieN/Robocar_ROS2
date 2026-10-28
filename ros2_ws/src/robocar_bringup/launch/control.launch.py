"""Brique control : manette, mux, VESC, odométrie et URDF.

Chaîne : robocar_teleop (F710, libevdev) -> /teleop, /drive_enable ┐
                              nœud étudiant -> /drive ──────────────┴> ackermann_mux -> ackermann_cmd
         -> robocar_vesc (wesc, UART) -> /sensors/core -> vesc_to_odom -> /odom
"""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = get_package_share_directory('robocar_bringup') + '/config/'
    urdf = get_package_share_directory('robocar_description') + '/urdf/robocar.urdf.xacro'

    # false : permet de tester la chaîne sans manette ni VESC (ros2 topic pub /teleop ...)
    with_teleop = LaunchConfiguration('with_teleop')
    with_vesc = LaunchConfiguration('with_vesc')

    return LaunchDescription([
        DeclareLaunchArgument('with_teleop', default_value='true'),
        DeclareLaunchArgument('with_vesc', default_value='true'),

        Node(
            package='robocar_teleop',
            executable='robocar_teleop_node',
            name='robocar_teleop',
            parameters=[config + 'teleop.yaml'],
            condition=IfCondition(with_teleop),
        ),
        # Pas de remapping : le mux publie ackermann_cmd, qu'écoute robocar_vesc
        Node(
            package='ackermann_mux',
            executable='ackermann_mux',
            name='ackermann_mux',
            parameters=[config + 'mux.yaml'],
        ),
        Node(
            package='robocar_vesc',
            executable='robocar_vesc_node',
            name='robocar_vesc',
            parameters=[config + 'vesc.yaml'],
            condition=IfCondition(with_vesc),
        ),
        Node(
            package='vesc_ackermann',
            executable='vesc_to_odom_node',
            name='vesc_to_odom_node',
            parameters=[config + 'vesc.yaml'],
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': ParameterValue(Command(['xacro ', urdf]), value_type=str),
            }],
        ),
    ])
