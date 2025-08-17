from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_name = 'robot_description_template'
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_path = get_package_share_directory(pkg_name)

    urdf_file = os.path.join(pkg_path, 'urdf', 'robot_description_template.urdf')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        )
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-file', urdf_file,
            '-entity', pkg_name,
        ],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        spawn_entity,
    ])
