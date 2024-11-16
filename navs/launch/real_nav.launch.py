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
# [INFO] [1709129476.484761733] [rviz2]: Setting goal pose: Frame:map, Position(4.29789, -0.74416, 0), Orientation(0, 0, -0.699018, 0.715104) = Angle: -1.54805
# [INFO] [1709129571.304819925] [rviz2]: Setting goal pose: Frame:map, Position(4.12592, 1.34027, 0), Orientation(0, 0, -0.997467, 0.0711357) = Angle: -2.9992
# [INFO] [1709129665.008079153] [rviz2]: Setting goal pose: Frame:map, Position(-4.28872, -1.90128, 0), Orientation(0, 0, -0.999095, 0.0425278) = Angle: -3.05651


def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    pkgShare_dir = get_package_share_directory('navs')
    
    # pkg_tb3_sim = get_package_share_directory('tb3_sim')
    # map_file = os.path.join(pkg_tb3_sim, 'maps', 'map.yaml')
    map_file = os.path.join(pkgShare_dir, 'maps', 'fall_map.yaml')

    localizer_choice = LaunchConfiguration('localizer')
    localizer = DeclareLaunchArgument(
        'localizer',
        default_value='"amcl"',
        description='Whether to use amcl or slam toolbox'
    )
    # ros2 launch navs real_nav.launch.py localizer:="'slam'" launches slam toolbox as localizer
    # ros2 launch navs real_nav.launch.py localizer:="'amcl'" launches amcl as localizer


    use_sim_time_choice = LaunchConfiguration('use_sim_time')
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use simulation clock if true'
    )

    ld = LaunchDescription()
    ld.add_action(localizer)
    ld.add_action(use_sim_time)
    print("Using Sim Time:", use_sim_time_choice)
    nav_yaml_file = os.path.join(pkgShare_dir, 'config', 'nav2_params_real.yaml')
    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
        launch_arguments={
                        'use_sim_time': use_sim_time_choice,
                        'params_file': nav_yaml_file,
                        'map' : map_file
                        }.items(),
        condition=IfCondition(PythonExpression([localizer_choice, '==', '"amcl"']))
    )
    ld.add_action(bringup_cmd)
    ########################################################################################
    # SLAM mode launching
    nav_slam_yaml_file = os.path.join(pkgShare_dir, 'config', 'nav2_params_real_slam.yaml')
    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')),
        launch_arguments={
                        'use_sim_time': use_sim_time_choice,
                        'params_file': nav_slam_yaml_file,
                        # 'map' : map_file,
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
          "x": -8.05,
          "y": 10.43,
          "theta": -1.098, 
          "use_sim_time": use_sim_time_choice
      }]
    )
    ld.add_action(
        TimerAction(
            period=10.0, 
            actions=[set_init_amcl_pose_cmd],
        )
    )        
    return ld