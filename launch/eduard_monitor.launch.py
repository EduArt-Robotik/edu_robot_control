import os

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription

from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, EnvironmentVariable, PathJoinSubstitution

def generate_launch_description():
    # launch file arguments
    edu_robot_namespace = LaunchConfiguration('edu_robot_namespace')
    edu_robot_namespace_arg = DeclareLaunchArgument(
        'edu_robot_namespace', default_value=os.getenv('EDU_ROBOT_NAMESPACE', default='eduard')
    )
    # RViz Config
    package_path = get_package_share_path('edu_robot_control')
    rviz_config = os.path.join(package_path, 'parameter', 'eduard.rviz')

    rviz_node = Node(
      package='rviz2',
      executable='rviz2',
      name='rviz2',
      namespace=edu_robot_namespace,
      arguments=['-d', rviz_config]
    )

    # Robot Description for Eduard
    launch_file_path = os.path.join(package_path, 'launch', 'eduard_robot_description.launch.py')
    launch_file = IncludeLaunchDescription(PythonLaunchDescriptionSource(launch_file_path))

    return LaunchDescription([
      edu_robot_namespace_arg,
      rviz_node,
      launch_file
    ])
