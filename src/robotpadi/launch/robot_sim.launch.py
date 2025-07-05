import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit

def generate_launch_description():

    # --- Konfigurasi Umum ---
    pkg_project_name = 'robotpadi' 
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # --- Path ke File & Direktori ---
    pkg_path = get_package_share_directory(pkg_project_name)
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    
    # Path ke direktori models untuk Gazebo
    model_path = os.path.join(pkg_path, 'models')
    
    # Path ke file world, robot, dan rviz
    world_file = os.path.join(pkg_path, 'worlds', 'worlds.sdf')
    robot_file = os.path.join(pkg_path, 'urdf', 'main.xacro')
    rviz_file = os.path.join(pkg_path, 'rviz', 'view.rviz')

    # --- 1. Set Environment Variable untuk Gazebo ---
    set_env_vars = SetEnvironmentVariable(
    name='GZ_SIM_RESOURCE_PATH',
    value=os.pathsep.join([
        os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
        model_path
        ])
    )

    # --- 2. Launch Gazebo Simulator ---
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        # launch_arguments={'gz_args': f'-r {world_file}'}.items()
        launch_arguments={'gz_args': f'-r {world_file} -v 4'}.items()
    )

    # --- 3. Proses Deskripsi Robot dari XACRO ---
    robot_description = xacro.process_file(robot_file).toxml()

    # --- 4. Launch Robot State Publisher ---
    # Membaca URDF dan mempublikasikan state dan TF dari robot.
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description
        }]
    )

    # --- 5. Spawn Robot ke Gazebo ---
    # Menggunakan service /create dari Gazebo untuk memunculkan robot
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', '/robot_description',
            '-name', 'my_robot',
            '-x', '20.0',
            '-y', '0.0',
            '-z', '5.0'
        ],
        output='screen'
    )
    
    # --- 6. Launch Gazebo-ROS Bridge ---
    # Menghubungkan topic Gazebo ke topic ROS 2
    gz_ros_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[ '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                    '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
		            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
		            '/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
	                '/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
	                '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
	                '/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model',
                    '/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU'],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # --- 7. Launch RViz2 ---
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # --- 8. Controller Manager ---
    # Load the robot controllers configuration
    robot_controllers = os.path.join(pkg_path, 'config', 'diff_drive_controller.yaml')
    # Spawn joint state broadcaster controllers
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )
    
    # Spawn diff drive controller
    diff_drive_base_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'diff_drive_controller',
            '--param-file',
            robot_controllers,
            ],
    )

    joystick = IncludeLaunchDescription(
            PythonLaunchDescriptionSource ([os.path.join(get_package_share_directory('robotpadi'),'launch','teleop_joy.launch.py')
                                        ]), launch_arguments={'use_sim_time': 'true'}.items()
            )

    twist_mux_params = os.path.join(get_package_share_directory('robotpadi'),'config','twist_mux.yaml')
    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params, {'use_sim_time': True}],
            remappings=[('/cmd_vel_out','/diff_drive_controller/cmd_vel_unstamped')],
        )   

    # Node untuk Sliding Mode Controller
    # smc_controller_node = Node(
    #         package='smc_controller',
    #         executable='smc_controller',
    #         name='smc_controller_node',
    #         output='screen',
    #         remappings=[
    #             ('/cmd_vel_smc', '/diff_drive_controller/cmd_vel_unstamped')
    #         ],
    #         emulate_tty=True,
    #     )
    # Node untuk Sliding Mode Controller

    smc_controller_node = Node(
            package='smc_controller',
            executable='smc_controller',
            name='smc_controller_node',
            output='screen',
            remappings=[
            ('/cmd_vel', '/cmd_vel_unstamped')
            ],
            emulate_tty=True,
        )

    return LaunchDescription([  
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        set_env_vars,
        gz_sim,
        spawn_robot,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_robot,
                on_exit=[joint_state_broadcaster_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[diff_drive_base_controller_spawner],
            )
        ),
        robot_state_publisher,
        gz_ros_bridge,
        # rviz_node,
        joystick,
        twist_mux,  
        smc_controller_node
    ])