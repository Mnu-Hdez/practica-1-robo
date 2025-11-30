import os
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher

def generate_launch_description():
    pkg_share = get_package_share_directory('car_pkg')
    
    
    world_file_path = os.path.join(pkg_share, 'world', 'city_traffic.wbt')

   
    webots = WebotsLauncher(
        world=world_file_path,
        mode='realtime'
    )

    lane_controller_node = Node(
        package='car_pkg',
        executable='lane_controller',
        name='lane_controller_node',
        output='screen'
    )


    sign_detector_node = Node(
        package='car_pkg',
        executable='sign_detector',
        name='sign_detector_node',
        output='screen'
    )

    
    exit_handler = launch.actions.RegisterEventHandler(
        event_handler=launch.event_handlers.OnProcessExit(
            target_action=webots,
            on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
        )
    )

    return LaunchDescription([
        webots,
        lane_controller_node,
        sign_detector_node,
        exit_handler
    ])