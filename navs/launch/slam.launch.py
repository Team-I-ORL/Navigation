import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription



def generate_launch_description():
    pkg_share = get_package_share_directory('navs')
    # SLAM launching
    slam_config = os.path.join(pkg_share,'config/slam_params.yaml')
    slam = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory("slam_toolbox"),'launch','online_sync_launch.py'
                )]), launch_arguments={'use_sim_time': 'true','params_file': slam_config}.items()
    )

    
    return LaunchDescription([
            slam,
        ])