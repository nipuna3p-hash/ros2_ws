from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg_desc = get_package_share_directory('my_robot_description')
    
    # 1. PROCESS THE XACRO FILE
    # We use 'Command' to run xacro on the file. 
    # This converts macros into pure URDF.
    path_to_urdf = os.path.join(pkg_desc, 'urdf', 'arm.xacro')
    robot_description_content = Command(['xacro ', path_to_urdf])
    
    # Wrap it in ParameterValue to ensure it's treated as a string
    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    # 2. ROBOT STATE PUBLISHER
    # Publishes the processed URDF to the /robot_description topic
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # 3. GAZEBO
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            )
        )
    )

    # 4. SPAWN ENTITY
    # Spawns the robot using the description from the topic
    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'my_robot',
            '-x', '0', '-y', '0', '-z', '0.0'
        ],
        output='screen',
    )

    return LaunchDescription([
        robot_state_publisher,
        gazebo,
        spawn
    ])