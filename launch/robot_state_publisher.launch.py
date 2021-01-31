import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

import xacro


def generate_launch_description():
    # robot state publisher
    my_rotate_bot_path = os.path.join(
        get_package_share_directory('my_rotate_bot'))

    urdf_file = os.path.join(my_rotate_bot_path,
                              'urdf',
                              'model.urdf')

    doc = xacro.parse(open(urdf_file))
    xacro.process_doc(doc)
    robot_description = {'robot_description': doc.toxml()}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    return LaunchDescription([
        # missing:
        # 1. rviz2
        # 2. joint-state-publisher-gui
        node_robot_state_publisher,
    ])
