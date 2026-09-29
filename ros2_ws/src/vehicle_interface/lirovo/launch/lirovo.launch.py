from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import TimerAction
import os
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import SetParameter


def generate_launch_description():
    namePackage = 'lirovo'
    slam_params_path = os.path.join(get_package_share_directory(namePackage),'config','slam_params.yaml')
    nav2_params_path = os.path.join(get_package_share_directory(namePackage),'config','nav2_params.yaml')
    print(f"SLAM params path: {slam_params_path}")

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
    period=3.0,
    actions=[nav2_launch]
)

    return LaunchDescription([
    
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0.5', '0', '0', '0', 'base_link', 'lidar'],
            parameters=[{'use_sim_time': False}],
            name='static_tf_lidar'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '1.5708', '0', '0', 'base_link', 'base_footprint'],
            parameters=[{'use_sim_time': False}],
            name = 'static_tf_basefootprint'
        ),
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            parameters=[{
                'target_frame': 'base_link',
                'transform_tolerance': 0.05,
                'min_height': 0.2 ,
                'max_height': 1.0 ,
                'angle_min': -3.14159,
                'angle_max': +3.14159,
                'angle_increment': 0.00872665,
                'scan_time': 0.3,
                'range_min': 0.1,
                'range_max': 200.0,
                'use_inf': True,
                'inf_epsilon': 1.0,
                'queue_size': 50,
                'use_sim_time':False,
            }],
            remappings=[
                ('cloud_in', '/bf_lidar/point_cloud_out'),
                ('scan', '/scan'),
            ],
        ),
        
        Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(
            get_package_share_directory(namePackage),
            'config', 'localization.yaml')],
        ),
        
        SetParameter(name='use_sim_time', value=False),
  
            TimerAction(
        period=1.0,
        actions=[
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                slam_params_path, 
                {'use_sim_time':False}
            ],
            arguments=['--ros-args', '-p', 'use_sim_time:=False'] 
        ),
    ]
    ),
       delayed_nav2_launch
    ])
