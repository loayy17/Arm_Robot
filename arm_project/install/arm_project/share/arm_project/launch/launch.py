from importlib.resources import Package
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_path
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
import xacro
from launch.event_handlers import OnProcessExit
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.substitutions import Command, LaunchConfiguration
def generate_launch_description():

    # Specify the name of the package and path to xacro file within the package
    pkg_name = 'arm_project'
    file_subpath = 'description/Robot.xacro'
    arm_project_path = get_package_share_path('arm_project')
    default_rviz_config_path = arm_project_path / 'config/rviz.rviz'
    default_gazebo_config_path = '/home/loay/loay_ws/src/arm_project/config/config.yaml'
    rviz_arg = DeclareLaunchArgument(name='rvizconfig', default_value=str(default_rviz_config_path),
                                     description='Absolute path to rviz config file')
    # Use xacro to process the file
    gazebo_arg=DeclareLaunchArgument(name='gazebo', default_value=str(default_gazebo_config_path),
                                     description='Absolute path to gazebo config file')

    # Use xacro to process the file
    xacro_file = os.path.join(get_package_share_directory(pkg_name),file_subpath)
    robot_description_raw = xacro.process_file(xacro_file).toxml()


    # Configure the node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw,
        'use_sim_time': True}] # add other parameters here if required
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )


    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
        )


    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                    arguments=['-topic', 'robot_description',
                                '-entity', 'project'],
                    output='screen')




    # Run the node
    return LaunchDescription([

        rviz_arg,
        gazebo_arg,        
        node_robot_state_publisher,
        spawn_entity,
        
    ])