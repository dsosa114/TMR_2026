from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch.substitutions import Command

from ament_index_python.packages import get_package_share_path
import os

def generate_launch_description():

    urdf_file_path = os.path.join(get_package_share_path('ackermann_vehicle_description'),
                                  'urdf', 'mobile_base.urdf')
    
    rviz_config_path = os.path.join(get_package_share_path('ackermann_vehicle_description'),
                                  'rviz', 'rviz_config.rviz')
    
    robot_description = ParameterValue(Command(['xacro ', urdf_file_path]), value_type=str)

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}]
    )
    
    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui"
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=['-d', rviz_config_path]
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node
    ])
