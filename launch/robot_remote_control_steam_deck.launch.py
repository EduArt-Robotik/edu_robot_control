import os

from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, EnvironmentVariable, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    package_path = FindPackageShare('edu_robot_control')
    parameter_file = PathJoinSubstitution([
      package_path,
      'parameter',
      'remote_control_steam_deck.yaml'
    ])

    # robot namespace
    edu_robot_namespace = LaunchConfiguration('edu_robot_namespace')
    edu_robot_namespace_arg = DeclareLaunchArgument(
        'edu_robot_namespace', default_value=os.getenv('EDU_ROBOT_NAMESPACE', default='eduard')
    )  

    joy_node = Node(
      package='joy',
      executable='joy_node',
      parameters=[
        {'rate': 20.0}
      ],
      namespace=edu_robot_namespace
    )

    remote_control_node = Node(
      package='edu_robot_control',
      executable='remote_control',
      parameters=[parameter_file],
      namespace=edu_robot_namespace
    )

    return LaunchDescription([
      edu_robot_namespace_arg,
      joy_node,
      remote_control_node
    ])
