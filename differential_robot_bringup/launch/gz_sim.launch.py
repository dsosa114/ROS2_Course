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
                                     'worlds', 'sample_home.world')

    # display_launch_path = os.path.join(get_package_share_directory('differential_robot_description'), 'launch')

    urdf_path = os.path.join(get_package_share_path('differential_robot_description'), 
                             'urdf', 'differential_robot.urdf.xacro')
    
    # rviz_config_path = os.path.join(get_package_share_path('differential_robot_bringup'), 
    #                          'rviz', 'odom_config.rviz')
    
    robot_description = ParameterValue(Command(
        ["xacro",
         " ", 
         urdf_path,
         " ",
         "sim_classic:=",
         "false",
         " ",
         "ign:=",
         "false"
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

    # rviz2_node = Node(
    #     package="rviz2",
    #     executable="rviz2",
    #     arguments=['-d', rviz_config_path]
    # )

    display_robot_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            gazebo_launch_path,
            '/gz_sim.launch.py'
        ]), launch_arguments={'gz_args':'empty.sdf'}.items()
    )

    spawn_entity_node = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=['-topic', '/robot_description',
                   '-entity', 'differential_robot']
    )

    ign_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        # parameters=[{'use_sim_time': True}],
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # '/robot_description@std_msgs/msg/String',
            # '/differential_robot/odometry@nav_msgs/msg/Odometry',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'
        ]
    )

    return LaunchDescription([
        robot_state_publisher_node,
        # rviz2_node,
        display_robot_gazebo,
        spawn_entity_node,
        ign_bridge_node
    ])