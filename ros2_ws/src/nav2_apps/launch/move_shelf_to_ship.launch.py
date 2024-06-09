from launch import LaunchDescription
from launch_ros.actions import Node



def generate_launch_description():

    approach_service = Node(
        package='nav2_apps',
        executable='approach_service_server_node',
        output='screen',
        name='approach_service_server_node',
        parameters=[{'use_sim_time': True}]
        )

    move_shelf_to_ship_node = Node(
        package='nav2_apps',
        executable='move_shelf_to_ship.py',
        output='screen',
        name='move_shelf_to_ship_node',
        )


    return LaunchDescription(
        [
            move_shelf_to_ship_node,
            approach_service
        ]
    )