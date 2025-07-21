from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch.substitutions import Command
import os
from ament_index_python.packages import get_package_share_directory, get_package_share_path

def generate_launch_description():

    gz_launch_path = os.path.join(get_package_share_directory('ros_gz_sim'), 'launch')

    urdf_path = os.path.join(get_package_share_path('diff_summer_robot_description'),
                             'urdf', 'diff_summer_robot.urdf.xacro')
    
    rviz_config_path = os.path.join(get_package_share_path('diff_summer_robot_description'),
                             'rviz', 'config.rviz')
    
    gz_bridge_config_path = os.path.join(get_package_share_path('diff_summer_robot_bringup'),
                                        'config', 'gz_bridge.yaml')

    gz_world_path = os.path.join(get_package_share_path('diff_summer_robot_bringup'),
                             'worlds', 'my_first_world.sdf')
    
    robot_description = ParameterValue(Command(
        ['xacro',
         ' ',
         urdf_path
        ]), value_type=str
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description':robot_description}]
    )

    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            gz_launch_path,
            "/gz_sim.launch.py"
        ]), launch_arguments={'gz_args': f'{gz_world_path} -r'}.items()
    )

    gz_create_entity_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', '/robot_description']
    )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path]
    )

    gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': gz_bridge_config_path}]
    )



    return LaunchDescription([
        robot_state_publisher_node,
        gz_sim_launch,
        gz_create_entity_node,
        rviz2_node,
        gz_bridge_node
    ])