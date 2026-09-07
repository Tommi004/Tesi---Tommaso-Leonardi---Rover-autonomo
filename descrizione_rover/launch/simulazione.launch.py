from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg_descrizione_rover = get_package_share_directory('descrizione_rover')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    xacro_file = PathJoinSubstitution([
        FindPackageShare('descrizione_rover'), 'urdf', 'uni_rover.urdf.xacro'
    ])

    world_file = PathJoinSubstitution([
        FindPackageShare('descrizione_rover'), 'scenari', 'scenario_base.sdf'
    ])

    robot_description = Command(['xacro ', xacro_file])

    # Pubblica la descrizione del robot (TF statici tra i link)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }],
        output='screen'
    )

    # Avvia Gazebo con un mondo vuoto
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r ', world_file]}.items()
    )

    # Posiziona il rover 
    spawn_rover = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'uni_rover',
            '-topic', 'robot_description',
            '-z', '0.15'
        ],
        output='screen'
    )

    # Ponte topic tra ROS 2 e Gazebo
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/scan_raw@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        ],
        output='screen'
    )

    # Ponte TF: collega il nome del frame usato da Gazebo per il LiDAR a quello atteso dal resto del sistema
    lidar_frame_bridge = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'lidar_link', 'uni_rover/lidar_link/lidar'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    filtro_lidar_params = PathJoinSubstitution([
        FindPackageShare('descrizione_rover'), 'config', 'filtro_lidar.yaml'
    ])

    filtro_lidar = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        parameters=[filtro_lidar_params, {'use_sim_time': True}],
        remappings=[
            ('scan', 'scan_raw'),
            ('scan_filtered', 'scan'),
        ],
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher,
        gazebo,
        spawn_rover,
        bridge,
        lidar_frame_bridge,
        filtro_lidar,
    ])