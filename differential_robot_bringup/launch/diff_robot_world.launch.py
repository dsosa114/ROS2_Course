from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch.substitutions import Command
import os
from ament_index_python.packages import get_package_share_path, get_package_share_directory

def generate_launch_description():

    gazebo_launch_path = os.path.join(get_package_share_directory('gazebo_ros'), 'launch')

    gazebo_world_path = os.path.join(get_package_share_path('differential_robot_bringup'), 
                                     'worlds', 'sample_home.world')

    # display_launch_path = os.path.join(get_package_share_directory('differential_robot_description'), 'launch')

    urdf_path = os.path.join(get_package_share_path('differential_robot_description'), 
                             'urdf', 'differential_robot.urdf.xacro')
    
    # rviz_config_path = os.path.join(get_package_share_path('differential_robot_bringup'), 
    #                          'rviz', 'odom_config.rviz')
    
    robot_description = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{'robot_description':robot_description}]
    )

    # display_robot_rviz = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         display_launch_path,
    #         '/display.launch.py'
    #     ])
    # )

    # rviz2_node = Node(
    #     package="rviz2",
    #     executable="rviz2",
    #     arguments=['-d', rviz_config_path]
    # )

    display_robot_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            gazebo_launch_path,
            '/gazebo.launch.py'
        ]), launch_arguments={'world':gazebo_world_path}.items()
    )

    spawn_entity_node = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=['-topic', '/robot_description',
                   '-entity', 'differential_robot']
    )

    return LaunchDescription([
        robot_state_publisher_node,
        # rviz2_node,
        display_robot_gazebo,
        spawn_entity_node
    ])