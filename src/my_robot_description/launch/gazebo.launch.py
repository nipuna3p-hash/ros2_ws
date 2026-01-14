import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_desc = get_package_share_directory('my_robot_description')
    pkg_moveit = get_package_share_directory('my_robot_moveit_config')

    # 1. PROCESS XACRO
    path_to_urdf = os.path.join(pkg_desc, 'urdf', 'arm.xacro')
    robot_description_content = Command(['xacro ', path_to_urdf])
    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    # 2. CONTROLLER CONFIGURATION
    # We point to the yaml file you already have in the moveit config package
    controller_config = os.path.join(pkg_moveit, 'config', 'ros2_controllers.yaml')

    # 3. ROBOT STATE PUBLISHER
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # 4. GAZEBO (with controller params)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            )
        ),
        launch_arguments={
            # This is the magic line that feeds the controller config to Gazebo
            'extra_gazebo_args': '--ros-args --params-file ' + controller_config
        }.items()
    )

    # 5. SPAWN ENTITY
    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'my_robot',
            '-x', '0', '-y', '0', '-z', '0.05' # Raised slightly to prevent clipping
        ],
        output='screen',
    )

    # 6. SPAWN CONTROLLERS (Optional but recommended to auto-start them)
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller"],
    )

    return LaunchDescription([
        robot_state_publisher,
        gazebo,
        spawn,
        joint_state_broadcaster,
        arm_controller
    ])