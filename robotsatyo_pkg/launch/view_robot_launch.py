import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package path
    pkg_share = get_package_share_directory('robotsatyo_pkg')

    # Launch configuration
    urdf_file = LaunchConfiguration('urdf_file')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    use_joint_state_pub = LaunchConfiguration('use_joint_state_pub')
    use_rviz = LaunchConfiguration('use_rviz')

    # File paths
    default_urdf = os.path.join(pkg_share, 'urdf', 'robot_padi.urdf')
    default_rviz = os.path.join(pkg_share, 'rviz', 'view.rviz')
    world_path = os.path.join(pkg_share, 'worlds', 'worlds.sdf')

    # Declare arguments
    declare_args = [
        DeclareLaunchArgument('urdf_file', default_value=default_urdf),
        DeclareLaunchArgument('rviz_config_file', default_value=default_rviz),
        DeclareLaunchArgument('use_robot_state_pub', default_value='true'),
        DeclareLaunchArgument('use_joint_state_pub', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
    ]

    # Gazebo sim with GZ_SIM_RESOURCE_PATH set
    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-v', '4', world_path],
        output='screen',
        additional_env={
            'GZ_SIM_RESOURCE_PATH': f"{os.path.expanduser('~/.gazebo/models')}:{os.path.expanduser('~/.gz')}"
        }
    )

    # Joint state publisher
    joint_state_publisher = Node(
        condition=IfCondition(use_joint_state_pub),
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        arguments=[urdf_file]
    )

    # Robot state publisher
    robot_state_publisher = Node(
        condition=IfCondition(use_robot_state_pub),
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': open(default_urdf).read()}]
    )

    # RViz
    rviz = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    # Spawn robot to Gazebo
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_robot',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'bebas',
            '-allow_renaming', 'true',
            '-x', '12.0', '-y', '12.0', '-z', '0.0',
        ]
    )

    # Combine launch description
    return LaunchDescription(
        declare_args + [
            gz_sim,
            joint_state_publisher,
            robot_state_publisher,
            rviz,
            spawn_robot
        ]
    )
