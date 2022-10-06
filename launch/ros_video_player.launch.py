import os
import sys
from glob import glob
import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    share_dir = get_package_share_directory('ros_video_player')
    videos_path = glob(os.path.join(share_dir, 'video/*.*'))
    default_video = '/dev/video0'
    if(len(videos_path) > 0):
        default_video = videos_path[0]

    container = ComposableNodeContainer(
                name='ros_video_player_container',
                namespace='',
                package='rclcpp_components',
                executable='component_container',
                composable_node_descriptions=[
                    ComposableNode(
                        package='ros_video_player',
                        plugin='ros_video_player::VideoPlayerNode',
                        name='ros_video_player',
                        parameters=[{
                            'publish_topic_name': launch.substitutions.LaunchConfiguration('publish_topic_name'),
                            'video_path': launch.substitutions.LaunchConfiguration('video_path'),
                            'frame_id': launch.substitutions.LaunchConfiguration('frame_id'),
                            'loop': launch.substitutions.LaunchConfiguration('loop'),
                            'speed': launch.substitutions.LaunchConfiguration('speed'),
                            'image_size': launch.substitutions.LaunchConfiguration('image_size'),
                            'video_buffer_size': launch.substitutions.LaunchConfiguration('video_buffer_size'),
                            'camera_info_url': launch.substitutions.LaunchConfiguration('camera_info_url'),
                            'camera_name': launch.substitutions.LaunchConfiguration('camera_name'),
                        }]),
                ],
                output='screen',
        )

    rqt_image_view = launch_ros.actions.Node(
        package='rqt_image_view', executable='rqt_image_view',
    )

    return launch.LaunchDescription(
        [
            launch.actions.DeclareLaunchArgument('publish_topic_name',
                                                 default_value='image_raw',
                                                 description=''),
            launch.actions.DeclareLaunchArgument('video_path',
                                                 default_value=default_video,
                                                 description=''),
            launch.actions.DeclareLaunchArgument('image_size',
                                                 default_value='[640, 480]',
                                                 description=''),
            launch.actions.DeclareLaunchArgument('frame_id',
                                                 default_value='map',
                                                 description=''),
            launch.actions.DeclareLaunchArgument('loop',
                                                 default_value='False',
                                                 description=''),
            launch.actions.DeclareLaunchArgument('speed',
                                                 default_value='1.0',
                                                 description=''),
            launch.actions.DeclareLaunchArgument('video_buffer_size',
                                                 default_value='1',
                                                 description=''),
            launch.actions.DeclareLaunchArgument('camera_info_url',
                                                 default_value='""',
                                                 description=''),
            launch.actions.DeclareLaunchArgument('camera_name',
                                                 default_value='""',
                                                 description=''),
            container,
            rqt_image_view,
            # Shutdown launch when rqt_image_view exits.
            launch.actions.RegisterEventHandler(
                event_handler=launch.event_handlers.OnProcessExit(
                    target_action=rqt_image_view,
                    on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
                )),
        ]
    )
