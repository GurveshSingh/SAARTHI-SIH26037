from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import TimerAction
import os
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import SetParameter
from launch.conditions import IfCondition


def generate_launch_description():

    SLAM_DELAY = 2.0
    NAV2_DELAY = 5.0

    namePackage = 'lirovo'

    pkg_share = get_package_share_directory(namePackage)

    slam_params_path = os.path.join(
        pkg_share,
        'config',
        'slam_params2.yaml'
    )

    nav2_params_path = os.path.join(
        pkg_share,
        'config',
        'nav2_params.yaml'
    )    
    pkg_nav2_dir = get_package_share_directory('nav2_bringup')

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'params_file': nav2_params_path,
            'autostart': 'True',
            'map': 'map',  
            
        }.items() 
    )
    delayed_nav2_launch = TimerAction(
    period=NAV2_DELAY,
    actions=[nav2_launch]
)

    return LaunchDescription([
        SetParameter(name='use_sim_time', value=True),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0.5', '0', '0', '0', 'base_link', 'lidar'],
            name='static_tf_lidar'
        ),
    

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '1.5708', '0', '0', 'base_link', 'base_footprint'],
            name = 'static_tf_basefootprint'
        ),

        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            parameters=[{
                'target_frame': 'lidar',
                'transform_tolerance': 0.5,
                'angle_min': -3.14159,
                'angle_max': +3.14159,
                'angle_increment': 0.00872665,
                'scan_time': 0.1,
                'use_inf': True,
                'inf_epsilon': 1.0,
                'queue_size': 50,
            }],
            remappings=[
                ('cloud_in', '/bf_lidar/point_cloud_out'),
                ('scan', '/scan'),
            ],
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('genz_icp'), 'launch', 'odometry.launch.py') 
            ),
            launch_arguments={
                'topic': '/bf_lidar/point_cloud_out', 
                'publish_odom_tf': 'True',
            }.items()
        ),

        

    
        TimerAction(
        period=SLAM_DELAY,
        actions=[
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='both',
            respawn=True,
            respawn_delay=2.0,
            parameters=[
                slam_params_path, 
            ],
                            ),
    ]
    ),
       delayed_nav2_launch, 

    TimerAction(
    period=6.0,
    actions=[
        Node(
            package='lirovo',
            executable='startup_report',
            output='screen'
        )
    ]
),
    Node(
        package='lirovo',
        executable='stack_watchdog',
        name='stack_watchdog',
        output='screen',

        parameters=[{
            'odom_topic': '/genz/odometry',
            'scan_topic': '/scan',
            'pc_topic': '/bf_lidar/point_cloud_out',

            'odom_timeout': 2.0,
            'scan_timeout': 2.0,
            'pc_timeout': 2.0,

            'check_frequency': 1.0,
            'check_tf': True,
        }],

        respawn=True,
        respawn_delay=2.0,
    ),

    ])
