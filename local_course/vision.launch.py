"""CPU stereo visual odometry; simulated camera inputs only, no truth or IMU."""
import os
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    profile = os.environ.get('VISION_PROFILE', 'temporal-flow')
    profiles = {
        'baseline-descriptor': {'Odom/Strategy': '0', 'Vis/CorType': '0'},
        # Temporal LK tracks local image patches instead of descriptor rematching.
        'temporal-flow': {'Odom/Strategy': '1', 'Vis/CorType': '1'},
    }
    if profile not in profiles:
        raise ValueError(f'Unknown VISION_PROFILE {profile!r}')
    print(f'VISUAL_ESTIMATOR_PROFILE={profile}', flush=True)
    actions = []
    for side, y in [('left', 0.0375), ('right', -0.0375)]:
        actions.append(Node(
            package='tf2_ros', executable='static_transform_publisher',
            name=f'course_{side}_optical_tf',
            arguments=['--x', '0.1053', '--y', str(y), '--z', '0.0304',
                       '--qx', '-0.5', '--qy', '0.5', '--qz', '-0.5', '--qw', '0.5',
                       '--frame-id', 'base_link', '--child-frame-id', f'oak_{side}_camera_optical_frame'],
            parameters=[{'use_sim_time': True}]))
    actions.append(Node(
        package='rtabmap_odom', executable='stereo_odometry', name='course_stereo_odometry',
        output='screen', parameters=[{
            'use_sim_time': True, 'frame_id': 'base_link', 'odom_frame_id': 'odom',
            'publish_tf': True, 'approx_sync': True, 'approx_sync_max_interval': 0.025,
            'topic_queue_size': 20, 'sync_queue_size': 20, 'qos': 2,
            'wait_for_transform': 0.2,
            # Preserve the calibrated local origin through loss; never auto-reset.
            'Odom/ResetCountdown': '0',
            'Vis/MinInliers': '20',
            'Vis/FeatureType': '8', 'Vis/MaxFeatures': '1000',
            'Stereo/OpticalFlow': 'true',
            **profiles[profile],
        }], remappings=[
            ('left/image_rect', '/oak/left/image_raw'),
            ('right/image_rect', '/oak/right/image_raw'),
            ('left/camera_info', '/oak/left/camera_info'),
            ('right/camera_info', '/oak/right/camera_info'),
            ('odom', '/local_course/vo/odom'),
            ('odom_info', '/local_course/vo/odom_info'),
        ]))
    return LaunchDescription(actions)
