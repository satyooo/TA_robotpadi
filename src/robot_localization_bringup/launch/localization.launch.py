import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Mendapatkan path ke direktori share dari paket INI
    pkg_share = get_package_share_directory('robot_localization_bringup')

    # Mendapatkan path lengkap ke file konfigurasi ekf.yaml
    config_file = os.path.join(pkg_share, 'config', 'ekf.yaml')

    # Membuat node untuk robot_localization (EKF)
    start_robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[config_file, {'use_sim_time': True}]    
    )

    return LaunchDescription([
        start_robot_localization_node
    ])