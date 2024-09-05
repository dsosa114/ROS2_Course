import launch
import os
import sys

from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, Command, FindExecutable
from launch_ros.substitutions import FindPackageShare

def get_robot_description():
    default_kinematics_params = PathJoinSubstitution(
        [FindPackageShare("xarm_description"), "config", "kinematics", "default", "lite6_default_kinematics.yaml"]
    )
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("xarm_description"), "urdf", "xarm_device.urdf.xacro"]),
            " ",
            "robot_ip:=",
            " ",
            "robot_type:=",
            "lite",
            " ",
            "dof:=",
            "6",
            " ",
            "add_gripper:=",
            "true",
            " ",
            "kinematics_suffix:=",
            "",
            " ",
        ]
    )


    robot_description = {"robot_description": robot_description_content}
    return robot_description

def get_robot_description_semantic():
    # MoveIt Configuration
    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("xarm_moveit_config"), "srdf", "xarm.srdf.xacro"]),
            " ",
            "dof:=",
            # Also ur_type parameter could be used but then the planning group names in yaml
            # configs has to be updated!
            '6',
            " ",
            "robot_type:=",
            'lite',
            " ",
            "add_gripper:=",
            'true',
            " ",
        ]
    )
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_content
    }
    return robot_description_semantic
    
def generate_launch_description():
    # generate_common_hybrid_launch_description() returns a list of nodes to launch
    robot_description = get_robot_description()
    robot_description_semantic = get_robot_description_semantic()
    demo_node = Node(
        package="mcr_moveit",
        executable="xarm_moveit_basic",
        name="xarm_moveit_basic",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
        ],
    )

    return launch.LaunchDescription([demo_node])