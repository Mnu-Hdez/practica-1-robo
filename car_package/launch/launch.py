import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_dir = get_package_share_directory('car_package')
    world_file = os.path.join(package_dir, 'worlds', 'city_traffic.wbt')
    urdf_file = os.path.join(package_dir, 'resource', 'my_robot.urdf')
    
    # Verificar que los archivos existen
    if not os.path.exists(world_file):
        raise FileNotFoundError(f"World file not found: {world_file}")
    if not os.path.exists(urdf_file):
        raise FileNotFoundError(f"URDF file not found: {urdf_file}")
    
    webots = ExecuteProcess(
        cmd=['webots', '--mode=realtime', world_file],
        output='screen',
        shell=True
    )
    
    # Webots ROS2 driver node (ESENCIAL)
    driver_node = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        parameters=[
            {'robot_description': open(urdf_file, 'r').read()},
            {'use_sim_time': True}
        ]
    )
    
    # Tus nodos personalizados
    lane_controller = Node(
        package='car_package',
        executable='lane_controller',
        name='lane_controller',
        output='screen'
    )
    
    sign_detector = Node(
        package='car_package', 
        executable='sign_detector',
        name='sign_detector',
        output='screen'
    )
    
    car_driver = Node(
        package='car_package',
        executable='car_driver',
        name='car_driver',
        output='screen'
    )
    
    return LaunchDescription([
        webots,
        driver_node,
        lane_controller,
        sign_detector,
        car_driver
    ])