import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():
    package_name = "path_planner_server"
   
    DeclareLaunchArgument(
        'map_file',
        description='Path to the map file',
        default_value='warehouse_map_sim.yaml'
        )

    map_name = LaunchConfiguration('map_file')
    map_file_path = PathJoinSubstitution([get_package_share_directory(package_name), 'config', map_name])
    amcl_yaml         = os.path.join(get_package_share_directory(package_name), 'config', 'amcl_config.yaml')
    controller_yaml   = os.path.join(get_package_share_directory(package_name), 'config', 'controller.yaml')
    planner_yaml      = os.path.join(get_package_share_directory(package_name), 'config', 'planner_server.yaml')
    recovery_yaml     = os.path.join(get_package_share_directory(package_name), 'config', 'recovery.yaml')
    bt_navigator_yaml = os.path.join(get_package_share_directory(package_name), 'config', 'bt.yaml')
    filters_yaml      = os.path.join(get_package_share_directory(package_name), 'config', 'filters.yaml')
    
    mapserver = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'use_sim_time': False}, 
                    {'yaml_filename': map_file_path}]
    )
    
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[amcl_yaml]
    )

    cont = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[controller_yaml],
        remappings=[('cmd_vel', 'robot/cmd_vel')]
    )

    plan = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[planner_yaml]
    )
        
    rec = Node(
        package='nav2_recoveries',
        executable='recoveries_server',
        name='recoveries_server',
        output='screen',
        parameters=[recovery_yaml]
    )
        

    nav = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[bt_navigator_yaml]
    )

    filter_mask = Node(
        package='nav2_map_server',
        executable='map_server',
        name='filter_mask_server',
        output='screen',
        emulate_tty=True,
        parameters=[filters_yaml]
    )

    filter_costmap = Node(
        package='nav2_map_server',
        executable='costmap_filter_info_server',
        name='costmap_filter_info_server',
        output='screen',
        emulate_tty=True,
        parameters=[filters_yaml]
    )

    life = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        parameters=[{'use_sim_time': False},
                    {'autostart': True},
                    {'node_names': ['controller_server',
                                    'planner_server',
                                    'recoveries_server',
                                    'bt_navigator',
                                    'filter_mask_server',
                                    'costmap_filter_info_server']}]
    )

    # RVIZ config
    rviz_config_dir = os.path.join(get_package_share_directory(package_name), 'rviz', 'rviz2.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        name='rviz_node',
        parameters=[{'use_sim_time': False}],
        arguments=['-d', rviz_config_dir]
        )
    
    return LaunchDescription([
        cont,
        plan,
        rec,
        nav,
        filter_mask,
        filter_costmap,     
        life,
        rviz_node      
    ])