import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # 1. Load MoveIt Configuration
    moveit_config = MoveItConfigsBuilder("my_robot", package_name="my_robot_moveit_config").to_moveit_configs()

    # 2. Launch Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('my_robot_description'), 'launch', 'gazebo.launch.py')
        )
    )

    # 3. Launch Move Group (The Planner)
    # FIX: We pass 'use_sim_time': True as a SEPARATE dictionary in the list.
    # This guarantees it is applied to the node.
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {'use_sim_time': True}, # <--- CRITICAL FIX
            {'trajectory_execution.allowed_execution_duration_scaling': 2.0}, # Allow slower movement
            {'plan_execution.max_replan_attempts': 5}, # Retry if failed
        ],
    )

    # 4. Launch RViz
    rviz_config_file = os.path.join(
        get_package_share_directory("my_robot_moveit_config"), "config", "moveit.rviz"
    )
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            {'use_sim_time': True} # Fix RViz too
        ],
    )

    return LaunchDescription([
        gazebo,
        run_move_group_node,
        rviz_node
    ])