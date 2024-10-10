import os

from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, TextSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
  # Launch File Arguments
  edu_robot_namespace = LaunchConfiguration('edu_robot_namespace')
  edu_robot_namespace_arg = DeclareLaunchArgument('edu_robot_namespace', default_value=os.getenv('EDU_ROBOT_NAMESPACE', default='eduard'))

  use_sim_time = LaunchConfiguration('use_sim_time')
  use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='False')    

  # RViz Config
  package_path = FindPackageShare('edu_robot_control')
  rviz_config = PathJoinSubstitution([
    package_path,
    'parameter',
    'arthur.rviz'
  ])

  rviz_node = Node(
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    namespace=edu_robot_namespace,
    parameters=[
      {'use_sim_time': use_sim_time}
    ],
    arguments=['-d', rviz_config]
  )

  # Robot Description for Eduard
  launch_file_path = PathJoinSubstitution([
    package_path,
    'launch',
    'arthur_robot_description.launch.py'
  ])
  launch_file = IncludeLaunchDescription(PythonLaunchDescriptionSource(launch_file_path))

  return LaunchDescription([
    edu_robot_namespace_arg,
    use_sim_time_arg,
    rviz_node,
    launch_file
  ])