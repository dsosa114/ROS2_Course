from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch.substitutions import Command
import os
from ament_index_python.packages import get_package_share_path, get_package_share_directory

def generate_launch_description():

    gazebo_launch_path = os.path.join(get_package_share_directory('ros_gz_sim'), 'launch')

    gazebo_world_path = os.path.join(get_package_share_path('differential_robot_bringup'), 
                                     'worlds', 'test_world.sdf')
    # gazebo_world_path = "empty.sdf"

    # display_launch_path = os.path.join(get_package_share_directory('differential_robot_description'), 'launch')

    urdf_path = os.path.join(get_package_share_path('diff_summer_robot_description'), 
                             'urdf/tutor', 'diff_summer_robot_tutor.urdf.xacro')
    
    rviz_config_path = os.path.join(get_package_share_path('diff_summer_robot_description'), 
                             'rviz', 'config.rviz')
    
    gazebo_config_path = os.path.join(get_package_share_path('differential_robot_bringup'), 
                                     'config', 'gz_summer_bridge.yaml')
    
    # robot_description = ParameterValue(Command(
    #     ["xacro",
    #      " ", 
    #      urdf_path,
    #      " ",
    #      "sim_classic:=",
    #      "false",
    #      " ",
    #      "ign:=",
    #      "false"
    #     ]), value_type=str)

    robot_description = ParameterValue(Command(
        ["xacro",
         " ", 
         urdf_path
        ]), value_type=str)

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

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=['-d', rviz_config_path]
    )

    display_robot_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            gazebo_launch_path,
            '/gz_sim.launch.py'
        ]), launch_arguments={'gz_args':f'{gazebo_world_path} -r'}.items()
    )

    spawn_entity_node = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=['-topic', '/robot_description']
    )

    bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{'config_file': gazebo_config_path}]
        # arguments=[
        #     '--ros-args',
        #     '-p',
        #     f'config_file:={gazebo_config_path}'
        # ]
    )

    return LaunchDescription([
        robot_state_publisher_node,
        rviz2_node,
        display_robot_gazebo,
        spawn_entity_node,
        bridge_node
    ])