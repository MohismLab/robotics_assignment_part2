import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare launch arguments
    model = LaunchConfiguration('model', default='burger')
    slam_methods = LaunchConfiguration('slam_methods', default='gmapping')
    configuration_basename = LaunchConfiguration('configuration_basename', default='turtlebot3_lds_2d.lua')
    set_base_frame = LaunchConfiguration('set_base_frame', default='base_footprint')
    set_odom_frame = LaunchConfiguration('set_odom_frame', default='odom')
    set_map_frame = LaunchConfiguration('set_map_frame', default='map')
    open_rviz = LaunchConfiguration('open_rviz', default='true')

    # Get package directories
    turtlebot3_bringup_dir = get_package_share_directory('turtlebot3_bringup')
    turtlebot3_slam_dir = get_package_share_directory('turtlebot3_slam')

    return LaunchDescription([
        DeclareLaunchArgument('model', default_value=model, description='model type [burger, waffle, waffle_pi]'),
        DeclareLaunchArgument('slam_methods', default_value=slam_methods, description='slam type [gmapping]'),
        DeclareLaunchArgument('configuration_basename', default_value=configuration_basename),
        DeclareLaunchArgument('set_base_frame', default_value=set_base_frame),
        DeclareLaunchArgument('set_odom_frame', default_value=set_odom_frame),
        DeclareLaunchArgument('set_map_frame', default_value=set_map_frame),
        DeclareLaunchArgument('open_rviz', default_value=open_rviz),

        # Include TurtleBot3 description
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(turtlebot3_bringup_dir, 'launch', 'includes', 'description.launch.py')),
            launch_arguments={'model': model}.items()
        ),

        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'publish_frequency': 50.0, 'tf_prefix': ''}]
        ),

        # Static transform publisher
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_broadcaster',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'occupancy_grid', '100']
        ),

        # SLAM: Gmapping
        Node(
            package='slam_gmapping',
            executable='slam_gmapping',
            name='turtlebot3_slam_gmapping',
            output='screen',
            parameters=[{
                'base_frame': set_base_frame,
                'odom_frame': set_odom_frame,
                'map_frame': set_map_frame,
                'map_update_interval': 2.0,
                'maxUrange': 3.0,
                'sigma': 0.05,
                'kernelSize': 1,
                'lstep': 0.05,
                'astep': 0.05,
                'iterations': 5,
                'lsigma': 0.075,
                'ogain': 3.0,
                'lskip': 0,
                'minimumScore': 50,
                'srr': 0.1,
                'srt': 0.2,
                'str': 0.1,
                'stt': 0.2,
                'linearUpdate': 1.0,
                'angularUpdate': 0.2,
                'temporalUpdate': 0.5,
                'resampleThreshold': 0.5,
                'particles': 100,
                'xmin': -10.0,
                'ymin': -10.0,
                'xmax': 10.0,
                'ymax': 10.0,
                'delta': 0.05,
                'llsamplerange': 0.01,
                'llsamplestep': 0.01,
                'lasamplerange': 0.005,
                'lasamplestep': 0.005
            }]
        ),

        # RViz
        GroupAction([
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz',
                arguments=['-d', os.path.join(turtlebot3_slam_dir, 'rviz', f'turtlebot3_{slam_methods}.rviz')],
                condition=IfCondition(open_rviz)
            )
        ])
    ])