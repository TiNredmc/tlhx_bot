# Test package launch file : 
import os

import launch
import launch_ros.actions
import launch_ros.descriptions
import xacro

from ament_index_python.packages import get_package_share_directory

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

    # Depth camera related. PCL2 and laser_scan

    container = launch_ros.actions.ComposableNodeContainer(
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
                              {'depth_mode': 'QVGA_30Hz'}],
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
                remappings=[('depth','camera/depth/image_raw'),
                            ('depth_camera_info', 'camera/depth/camera_info')],
                parameters=[{'scan_time': 0.033},
                            {'scan_range_min': 0.4},
                            {'scan_range_max': 5.0},
                            {'scan_height': 1},
                            {'output_frame': 'laser_frame'}]
            ),

             ],
        output = 'screen',
    )

    return launch.LaunchDescription([
       # container,
        node_robot_state_publisher,        
    ])
