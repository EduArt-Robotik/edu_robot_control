import os

from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, EnvironmentVariable, TextSubstitution

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

import xacro

def generate_launch_description():
    package_path = get_package_share_path('edu_robot_control')
    model_path = os.path.join(package_path, 'model/urdf')
    urdf_eduard_model_path = os.path.join(model_path, 'eduard.urdf')

    robot_name = os.getenv('EDU_ROBOT_NAMESPACE', default='eduard')
    wheel_type = os.getenv('EDU_ROBOT_WHEEL_TYPE', default='mecanum')

    print("use robot name = ", robot_name)
    print("use wheel type = ", wheel_type)
    robot_description = xacro.process_file(
        urdf_eduard_model_path,
        mappings={
            'robot_name': robot_name,
            'wheel_type': wheel_type
        }
    ).toprettyxml()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'robot_description': robot_description},
        ]
    )

    return LaunchDescription([
        robot_state_publisher_node,
    ])