"""Isolated source-stamped stereo+IMU replay; contains no PX4 interfaces."""
import math
import os
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    if os.environ.get("ROS_DOMAIN_ID") != "218":
        raise RuntimeError("isolated replay requires ROS_DOMAIN_ID=218")
    transforms = []
    for eye,lateral in (("left", 0.0375), ("right", -0.0375)):
        transforms.append(Node(package="tf2_ros", executable="static_transform_publisher",
            arguments=["--x","0.1053","--y",str(lateral),"--z","0.0304",
                       "--qx","-0.5","--qy","0.5","--qz","-0.5","--qw","0.5",
                       "--frame-id","base_link","--child-frame-id",f"oak_{eye}_camera_optical_frame"]))
    params = dict(use_sim_time=True, rectified_images=True, enable_imu_fusion=True,
        calibration_frequency=250., gyro_noise_density=0.0008726646/math.sqrt(250),
        accel_noise_density=0.00686/math.sqrt(250), gyro_random_walk=0., accel_random_walk=0.,
        imu_frame="base_link", base_frame="base_link", odom_frame="odom", map_frame="map",
        camera_optical_frames=["oak_left_camera_optical_frame","oak_right_camera_optical_frame"],
        enable_image_denoising=False, enable_localization_n_mapping=True,
        publish_odom_to_base_tf=False, publish_map_to_odom_tf=False,
        image_jitter_threshold_ms=250., imu_jitter_threshold_ms=10.)
    mappings = [("visual_slam/imu","/replay/imu")]
    for i,eye in enumerate(("left","right")):
        mappings.extend([(f"visual_slam/image_{i}",f"/replay/{eye}/image"),
                         (f"visual_slam/camera_info_{i}",f"/replay/{eye}/camera_info")])
    node = ComposableNode(package="isaac_ros_visual_slam",
        plugin="nvidia::isaac_ros::visual_slam::VisualSlamNode",name="visual_slam_node",
        parameters=[params], remappings=mappings)
    return LaunchDescription(transforms + [ComposableNodeContainer(name="replay_tracker",namespace="",
        package="rclcpp_components",executable="component_container_mt",
        composable_node_descriptions=[node],output="screen")])
