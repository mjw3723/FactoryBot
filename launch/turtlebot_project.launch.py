from launch import LaunchDescription
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.actions import TimerAction
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            XMLLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('rosbridge_server'),
                    'launch',
                    'rosbridge_websocket_launch.xml'
                ])
            ]),
        ),
        TimerAction(
            period=2.0,  
            actions=[
                Node(
                    package='turtlebot_project',           
                    executable='static_tf_bridge',     
                    name='static_tf_bridge',       
                    output='screen'
                ),
                Node(
                    package='turtlebot_project',           
                    executable='cmd_vel_bridge',     
                    name='cmd_vel_bridge',       
                    output='screen'
                ),
                Node(
                    package='turtlebot_project',           
                    executable='realsense_bridge',     
                    name='realsense_bridge',       
                    output='screen'
                ),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        PathJoinSubstitution([
                            FindPackageShare('yolo_bringup'),
                            'launch',
                            'yolov5.launch.py'
                        ])
                    ]),
                    launch_arguments={
                        'input_image_topic': '/camera/camera_color/image_raw'
                    }.items(),
                ),
            ]
        ),
        TimerAction(
            period=8.0,  
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        PathJoinSubstitution([
                            FindPackageShare('turtlebot3_navigation2'),
                            'launch',
                            'navigation2.launch.py'
                        ])
                    ]),
                    launch_arguments={
                        'use_sim_time': 'true',
                        'map': '/home/moon/projects/final_map.yaml'
                    }.items(),
                )
            ]
        )
        

    ])
