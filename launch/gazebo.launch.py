from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
import re
from ament_index_python.packages import get_package_share_directory


def resolve_package_path(urdf_str: str) -> str:
    def replacer(match):
        pkg = match.group(1)
        rel_path = match.group(2)
        pkg_path = get_package_share_directory(pkg)
        return os.path.join(pkg_path, rel_path)

    return re.sub(r"package://([^/]+)/(.+)", replacer, urdf_str)


def generate_launch_description():
    pkg_name = "robot_description_template"
    pkg_share = get_package_share_directory(pkg_name)
    urdf_file = os.path.join(pkg_share, "urdf", "robot_description_template.urdf")

    with open(urdf_file, "r") as f:
        urdf_content = f.read()
    robot_description = resolve_package_path(urdf_content)

    gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    FindPackageShare("ros_gz_sim").find("ros_gz_sim"),
                    "launch",
                    "gz_sim.launch.py",
                )
            ]
        ),
        # launch_arguments={"gz_args": "-r empty.sdf"}.items(),
        launch_arguments={
            "world": "empty.sdf",
            "paused": "true",
        }.items(),
    )

    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
        output="screen",
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-name", pkg_name, "-topic", "robot_description", "-z", "1.2"],
        output="screen",
    )

    return LaunchDescription(
        [
            gz_launch,
            robot_state_pub,
            spawn_robot,
        ]
    )
