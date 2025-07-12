import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Mendapatkan path ke direktori share paket Anda
    pkg_share = get_package_share_directory('robotpadi')

    # Mendapatkan path lengkap ke file konfigurasi ekf.yaml
    config_file = os.path.join(pkg_share, 'config', 'ekf.yaml')

    # Membuat node untuk robot_localization (EKF)
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[config_file]  # Memuat parameter dari file YAML
    )

    return LaunchDescription([
        ekf_node
    ])