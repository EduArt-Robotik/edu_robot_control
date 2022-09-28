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
    model_path = os.path.join(package_path, 'model/urdf')
    urdf_eduard_model_path = os.path.join(model_path, 'eduard.urdf')

    robot_description = ParameterValue(Command(['xacro ', urdf_eduard_model_path]), value_type=str)

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
    )

    return LaunchDescription([
        robot_state_publisher_node,
        # joint_state_publisher_gui_node
    ])