from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    pkg_share = FindPackageShare("robot_description_template").find("robot_description_template")
    urdf_file = os.path.join(pkg_share, "urdf", "robot_description_template.urdf")
    rviz_config = os.path.join(pkg_share, "rviz", "display.rviz")

    return LaunchDescription([
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher_gui",
            output="screen",
            parameters=[{
                'source_list': ['rviz/joint_state'],
                'rate': 60,
            }]
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": open(urdf_file).read()}]
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", rviz_config],
            output="screen"
        )
    ])
