# Test package launch file : 
import os

import launch
import launch_ros.actions
import launch.actions
import launch_ros.descriptions
import xacro

from ament_index_python.packages import get_package_share_directory

home = os.path.expanduser('~')

def generate_launch_description():
    
    namespace = '/camera'

     # Specify the name of the package and path to xacro file within the package
    pkg_name = 'tlhx_bot'
    file_subpath = 'description/robot.urdf.xacro'


    # Use xacro to process the file
    xacro_file = os.path.join(get_package_share_directory(pkg_name),file_subpath)
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    # Configure the node
    node_robot_state_publisher = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw}] # add other parameters here if required
    )
    
    mecanum_controller_instant = launch_ros.actions.Node(
        package='mecanum_controller',
        executable='mecanum_controller',
        output='screen',
        parameters=[os.path.join(get_package_share_directory("mecanum_controller"), 'params', 'mecanum.yaml')]
    )

    # Depth camera related. PCL2 and laser_scan
    depth_cam_container = launch_ros.actions.ComposableNodeContainer(
        name = 'container',
        namespace = namespace,
        package = 'rclcpp_components',
        executable = 'component_container',
        composable_node_descriptions=[
            # Camera Driver
            launch_ros.descriptions.ComposableNode(
                package = 'openni2_camera',
                plugin = 'openni2_wrapper::OpenNI2Driver',
                name = 'depth_camera_driver',
                namespace = namespace,
                parameters = [{'depth_registration': False},
                              {'use_device_time': True},
                              {'depth_mode': 'VGA_30Hz'}],
            #    remapping = [('depth/image', 'depth_registered/image_raw')],
            ),

            # Create XYZ point cloud
            launch_ros.descriptions.ComposableNode(
                package='depth_image_proc',
                plugin='depth_image_proc::PointCloudXyzNode',
                name='points_xyz',
                namespace=namespace,
                parameters=[{'queue_size': 10}],
                remappings=[('image_rect', 'depth/image_raw'),
                            ('camera_info', 'depth/camera_info'),
                            ('points', 'depth/points')],
            ),
	   

            # Depthimage to laser scan Converter
            launch_ros.descriptions.ComposableNode(
                package='depthimage_to_laserscan',
                plugin='depthimage_to_laserscan::DepthImageToLaserScanROS',
                name='depthimage_to_laserscan_node',
                remappings=[('depth','/camera/depth/image_raw'),
                            ('depth_camera_info', '/camera/depth/camera_info')],
                parameters=[{'scan_time': 0.033},
                            {'scan_range_min': 0.4},
                            {'scan_range_max': 10.0},
                            {'scan_height': 1},
                            {'output_frame': 'camera_scan_frame'}]
            ),
        ],
        output = 'screen',
    )

    laser_proc_node = launch_ros.actions.Node(
        package='ssl_slam2',
        executable='ssl_slam2_laser_processing_node',
        parameters=[
            {'scan_period': 0.03},
            {'vertical_angle': 2.0},
            {'max_dis': 8.0},
            {'map_resolution': 0.005},
            {'min_dis': 0.3},
            {'skip_frames': 1},
            {'map_path': home},   
        ],
        output='screen',
    )
    
    odom_est_map_node = launch_ros.actions.Node(
        package='ssl_slam2',
        executable='ssl_slam2_odom_estimation_mapping_node',
        parameters=[
            {'scan_period': 0.03},
            {'vertical_angle': 2.0},
            {'max_dis': 8.0},
            {'map_resolution': 0.005},
            {'min_dis': 0.3},
            {'skip_frames': 1},
            {'map_path': home},
            {'publish_odom_tf': False}   
        ],
        output='screen',
    )
    
    map_optz_node = launch_ros.actions.Node(
        package='ssl_slam2',
        executable='ssl_slam2_map_optimization_node',
        parameters=[
            {'scan_period': 0.03},
            {'vertical_angle': 2.0},
            {'max_dis': 8.0},
            {'map_resolution': 0.005},
            {'min_dis': 0.3},
            {'skip_frames': 1},
            {'map_path': home},
            {'min_map_update_distance' : 0.5},
            {'min_map_update_angle': 20.0},
            {'min_map_update_frame': 8.0},
        ],
        output='screen',
    )

    ekf_fusion = launch_ros.actions.Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(get_package_share_directory("tlhx_bot"), 'params', 'ekf.yaml')],

    )

    delayed_slam = launch.actions.TimerAction(period=3.0, actions=[laser_proc_node, odom_est_map_node, map_optz_node, ekf_fusion])

    return launch.LaunchDescription([
        node_robot_state_publisher,
        
        mecanum_controller_instant,
        
        depth_cam_container,
        
        delayed_slam,
    ])
