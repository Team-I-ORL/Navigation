import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    pkgShare_dir = get_package_share_directory('navs')
    # map_file = os.path.join(pkgShare_dir, 'maps', 'map.yaml')
    
    # pkg_tb3_sim = get_package_share_directory('tb3_sim')
    map_file = os.path.join(pkgShare_dir, 'maps', 'map.yaml')

    localizer_choice = LaunchConfiguration('localizer')
    localizer = DeclareLaunchArgument(
        'localizer',
        default_value='"amcl"',
        description='Whether to use amcl or slam toolbox'
    )
    # ros2 launch navs nav.launch.py localizer:="'slam'" launches slam toolbox as localizer
    # ros2 launch navs nav.launch.py localizer:="'amcl'" launches amcl as localizer


    use_sim_time_choice = LaunchConfiguration('use_sim_time')
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation clock if true'
    )

    ld = LaunchDescription()
    ld.add_action(localizer)
    ld.add_action(use_sim_time)
    print("Using Sim Time:", use_sim_time_choice)
    nav_yaml_file = os.path.join(pkgShare_dir, 'config', 'nav2_params_amcl.yaml')
    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
        launch_arguments={
                        'use_sim_time': "true",
                        'params_file': nav_yaml_file,
                        'map' : map_file
                        }.items(),
        condition=IfCondition(PythonExpression([localizer_choice, '==', '"amcl"']))
    )
    ld.add_action(bringup_cmd)
    
    nav_slam_yaml_file = os.path.join(pkgShare_dir, 'config', 'nav2_params_slam.yaml')
    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')),
        launch_arguments={
                        'use_sim_time': "true",
                        'params_file': nav_slam_yaml_file,
                        'map' : map_file,
                        # 'slam' : 'True'
                        }.items(),
        condition=IfCondition(PythonExpression([localizer_choice, '==', '"slam"']))
        )
    
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_dir, 'launch', 'localization_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time_choice,
            'slam_params_file': nav_slam_yaml_file
        }.items(),
        condition=IfCondition(PythonExpression([localizer_choice, '==', '"slam"']))
    )

    ld.add_action(slam_launch)
    ld.add_action(bringup_cmd)

    set_init_amcl_pose_cmd = Node(
      package="navs",
      executable="set_amcl_init_pose",
      name="set_amcl_init_pose",
      parameters=[{
          "x": 8.8,#16.59,
          "y": 4.5,#8.465,
          "theta": 0.0,  
      }]
    )
    ld.add_action(set_init_amcl_pose_cmd)
        
    return ld
