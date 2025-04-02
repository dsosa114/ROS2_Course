from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def launch_setup(context, *args, **kwargs):

    # General arguments
    # runtime_config_package = LaunchConfiguration("runtime_config_package")
    # controllers_file = LaunchConfiguration("controllers_file")
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    # prefix = LaunchConfiguration("prefix")
    start_joint_controller = LaunchConfiguration("start_joint_controller")
    initial_joint_controller = LaunchConfiguration("initial_joint_controller")
    launch_rviz = LaunchConfiguration("launch_rviz")
    gazebo_gui = LaunchConfiguration("gazebo_gui")

    # initial_joint_controllers = PathJoinSubstitution(
    #     [FindPackageShare(runtime_config_package), "config", controllers_file]
    # )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "rviz", "urdf_config.rviz"]
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "urdf", description_file]
            ),
            " ",
            "sim_gazebo:=",
            "true",
        ]
    )
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"use_sim_time": True}, robot_description],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(launch_rviz),
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        ),
        condition=IfCondition(launch_rviz),
    )

    # There may be other controllers of the joints, but this is the initially-started one
    initial_joint_controller_spawner_started = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[initial_joint_controller, "-c", "/controller_manager"],
        condition=IfCondition(start_joint_controller),
    )
    initial_joint_controller_spawner_stopped = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[initial_joint_controller, "-c", "/controller_manager", "--stopped"],
        condition=UnlessCondition(start_joint_controller),
    )

    # Spawn controllers
    # def controller_spawner(controllers, active=True):
    #     inactive_flags = ["--inactive"] if not active else []
    #     return Node(
    #         package="controller_manager",
    #         executable="spawner",
    #         arguments=[
    #             "--controller-manager",
    #             "/controller_manager",
    #             "--controller-manager-timeout",
    #             "10",
    #         ]
    #         + inactive_flags
    #         + controllers,
    #         condition=IfCondition(start_joint_controller),
    #     )
    
    # controllers_active = [
    #     "robotiq_controller_LKJ",
    #     "robotiq_controller_RKJ",
    #     "robotiq_controller_LIKJ",
    #     "robotiq_controller_RIKJ",
    #     "robotiq_controller_LFTJ",
    #     "robotiq_controller_RFTJ",
    # ]
    # controllers_inactive = []

    # controller_spawners = [controller_spawner(controllers_active)]

    # gripper_lfj_controller_spawner_started = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["robotiq_controller_LKJ", "-c", "/controller_manager"],
    #     condition=IfCondition(start_joint_controller),
    # )
    # gripper_lfj_controller_spawner_stopped = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["robotiq_controller_LKJ", "-c", "/controller_manager", "--stopped"],
    #     condition=UnlessCondition(start_joint_controller),
    # )


    # Gazebo nodes
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("gazebo_ros"), "/launch", "/gazebo.launch.py"]
        ),
        launch_arguments={
            "gui": gazebo_gui,
        }.items(),
    )

    # Spawn robot
    gazebo_spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_mark_one",
        arguments=["-entity", "mark_one_arm", "-topic", "robot_description"],
        output="screen",
    )

    nodes_to_start = [
        robot_state_publisher_node,
        joint_state_broadcaster_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner,
        initial_joint_controller_spawner_stopped,
        initial_joint_controller_spawner_started,
        # gripper_lfj_controller_spawner_stopped,
        # gripper_lfj_controller_spawner_started,
        gazebo,
        gazebo_spawn_robot,
    ]# + controller_spawners

    return nodes_to_start


def generate_launch_description():
    declared_arguments = []
    
    # General arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="mark_one_arm_description",
            description="Description package with robot URDF/XACRO files. Usually the argument \
        is not set, it enables use of a custom description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="mark_one_arm.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "start_joint_controller",
            default_value="true",
            description="Enable headless mode for robot control",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "initial_joint_controller",
            default_value="joint_trajectory_controller",
            description="Robot controller to start.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument("launch_rviz", default_value="true", description="Launch RViz?")
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gazebo_gui", default_value="true", description="Start gazebo with GUI?"
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])