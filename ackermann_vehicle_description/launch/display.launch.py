from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
import os
import yaml
from ament_index_python.packages import get_package_share_path


def generate_launch_description():

    package_name = 'ackermann_vehicle_description'
    robot_description_pkg = get_package_share_path(package_name)

    urdf_path = os.path.join(robot_description_pkg,
                             'urdf', 'ackermann_robot.urdf.xacro')
    
    default_params_file_path = os.path.join(robot_description_pkg, 
                               'config', 'params.yaml')
    
    rviz_config_path = os.path.join(robot_description_pkg,
                                    'rviz', 'config.rviz')

    # Declare launch arguments for Xacro parameters
    robot_parameters_file_arg = DeclareLaunchArgument(
        'robot_parameters_file', 
        default_value= default_params_file_path,
        description='Parameter file with all property values of the robot'
    )

    robot_parameters_file = LaunchConfiguration('robot_parameters_file')

    
    robot_description = ParameterValue(Command(
        ['xacro', 
         ' ',
         urdf_path, 
         ' ', 
         'robot_parameters_file:=',
         robot_parameters_file
        ]), value_type=str)

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{'robot_description':robot_description}]
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui"
    )

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=['-d', rviz_config_path]
    )

    return LaunchDescription([
        robot_parameters_file_arg,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz2_node
    ])

# OpaqueFunction defers the execution until launch time,
# so all substitutions are resolved