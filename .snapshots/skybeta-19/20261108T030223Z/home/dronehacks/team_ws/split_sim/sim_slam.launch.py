"""Simulation camera estimator. Full-rate measured simulated IMU is opt-in."""
import math
import os
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    inertial = os.environ.get('SPLIT_INERTIAL') == '1'
    if inertial and os.environ.get('ROS_DOMAIN_ID') != '219':
        raise RuntimeError('simulation VIO requires domain219')
    imu_params = dict(calibration_frequency=250.,
                      gyro_noise_density=0.0008726646/math.sqrt(250),
                      accel_noise_density=0.00686/math.sqrt(250),
                      gyro_random_walk=0., accel_random_walk=0.,
                      imu_frame='base_link', imu_jitter_threshold_ms=10.) if inertial else {}
    slam = ComposableNode(
        package='isaac_ros_visual_slam',
        plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
        name='visual_slam_node',
        parameters=[{
            'use_sim_time': True,
            'rectified_images': True,
            'enable_imu_fusion': inertial,
            **imu_params,
            'enable_image_denoising': False,
            'enable_localization_n_mapping': True,
            'enable_slam_visualization': True,
            'enable_landmarks_view': True,
            'enable_observations_view': False,
            'image_jitter_threshold_ms': 250.0,
            'base_frame': 'base_link', 'odom_frame': 'odom', 'map_frame': 'map',
            'camera_optical_frames': [
                'oak_left_camera_optical_frame', 'oak_right_camera_optical_frame'],
            'publish_odom_to_base_tf': True,
            'publish_map_to_odom_tf': True,
            'path_max_size': 1000,
        }],
        remappings=([('visual_slam/imu','/split_camera/body_imu')] if inertial else []) + [
            ('visual_slam/image_0', '/oak/left/image_mono'),
            ('visual_slam/image_1', '/oak/right/image_mono'),
            ('visual_slam/camera_info_0', '/oak/left/camera_info'),
            ('visual_slam/camera_info_1', '/oak/right/camera_info'),
        ])
    # Matches the *generated simulation* cameras, not measured hardware calibration.
    transforms = []
    for side, y in [('left', 0.0375), ('right', -0.0375)]:
        transforms.append(Node(
            package='tf2_ros', executable='static_transform_publisher',
            name=f'sim_{side}_camera_tf',
            arguments=['--x', '0.1053', '--y', str(y), '--z', '0.0304',
                       '--qx', '-0.5', '--qy', '0.5', '--qz', '-0.5', '--qw', '0.5',
                       '--frame-id', 'base_link',
                       '--child-frame-id', f'oak_{side}_camera_optical_frame'],
            parameters=[{'use_sim_time': True}]))
    return LaunchDescription(transforms + [ComposableNodeContainer(
        name='sim_vslam_container', namespace='', package='rclcpp_components',
        executable='component_container_mt', composable_node_descriptions=[slam],
        output='screen')])
