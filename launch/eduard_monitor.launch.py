import os

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription

from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, EnvironmentVariable, PathJoinSubstitution

def generate_launch_description():
    # RViz Config
    package_path = get_package_share_path('edu_robot_control')
    rviz_config = os.path.join(package_path, 'parameter', 'eduard.rviz')

    rviz_node = Node(
      package='rviz2',
      executable='rviz2',
      name='rviz2',
      namespace=EnvironmentVariable('EDU_ROBOT_NAMESPACE', default_value="eduard"),
      arguments=['-d', rviz_config]
    )

    # Robot Description for Eduard
    launch_file_path = os.path.join(package_path, 'launch', 'eduard_robot_description.launch.py')
    launch_file = IncludeLaunchDescription(PythonLaunchDescriptionSource(launch_file_path))

    return LaunchDescription([
      rviz_node,
      launch_file
    ])