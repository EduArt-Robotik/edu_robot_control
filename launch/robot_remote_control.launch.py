import os

from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    package_path = get_package_share_path('edu_robot_control')
    parameter_file = os.path.join(
      package_path,
      'parameter',
      'remote_control.yaml'
    )

    joy_node = Node(
      package='joy',
      executable='joy_node',
      parameters=[
        {'autorepeat_rate': 20.0},
        {'coalesce_interval_ms' : 50}
      ]
    )

    remote_control_node = Node(
      package='edu_robot_control',
      executable='remote_control',
      parameters= [parameter_file]
    )

    return LaunchDescription([
      joy_node,
      remote_control_node
    ])