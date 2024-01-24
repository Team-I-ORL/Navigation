import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    python_commander_dir = get_package_share_directory('nav2_simple_commander')
    pkgShare_dir = launch_ros.substitutions.FindPackageShare(package='simple_nav').find('simple_nav')
    nav_yaml_file = os.path.join(pkgShare_dir, 'config', 'nav2_params.yaml')
    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
        launch_arguments={'slam': 'True','use_sim_time': 'true', 'params_file': nav_yaml_file}.items())
    
    ld = LaunchDescription()
    ld.add_action(bringup_cmd)
    
    return ld