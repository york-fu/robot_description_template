from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    pkg_name = 'robot_description_template'
    pkg_share = FindPackageShare(pkg_name).find(pkg_name)
    # urdf_file = PathJoinSubstitution([pkg_share, "urdf", "robot_description_template.urdf"])
    urdf_file = os.path.join(pkg_share, "urdf", "robot_description_template.urdf")

    gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(
                FindPackageShare("ros_gz_sim").find("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py"
            )]
        ),
        launch_arguments={"gz_args": "-r empty.sdf"}.items()
    )

    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": open(urdf_file).read()}],
        output="screen"
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-name", pkg_name, "-topic", "robot_description", "-z", "1.2"],
        output="screen"
    )

    return LaunchDescription([
        gz_launch,
        robot_state_pub,
        spawn_robot
    ])
