import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext, LaunchService
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

from pathlib import Path

import xacro


def evaluate_joystick_config(context: LaunchContext, joy_launch_config):

    # Get joystick package directory
    teleop_twist_dir = get_package_share_directory('teleop_twist_joy')

    # Setup name of configuration file
    config_file_name = context.perform_substitution(joy_launch_config)
    joystick_config_file = os.path.join(get_package_share_directory('volksbot_driver'), 'config/joystick/', config_file_name)
  
    # Return list with joystic node generated from 
    # imported lauch file with joystick parameter file
    return [IncludeLaunchDescription(
                PythonLaunchDescriptionSource(teleop_twist_dir + '/launch/teleop-launch.py'),
                launch_arguments={'config_filepath': joystick_config_file}.items(),
    )]



def generate_launch_description():

    # Declare launch configuratuin parameters
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    num_wheels = LaunchConfiguration('num_wheels', default=4)
    wheel_radius = LaunchConfiguration('wheel_radius', default=0.0985)
    publish_tf = LaunchConfiguration('publish_tf', default='false')
    #tf_prefix = LaunchConfiguration("tf_prefix", '')


    channel_type =  LaunchConfiguration('channel_type', default='serial')
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='256000')
    frame_id = LaunchConfiguration('frame_id', default='laser')
    inverted = LaunchConfiguration('inverted', default='false')
    angle_compensate = LaunchConfiguration('angle_compensate', default='true')
    scan_mode = LaunchConfiguration('scan_mode', default='Sensitivity')

    # Get Volksbot URDF / xacro file and parse into valid URDF 
    # description
    urdf_file_name = 'urdf/volksbot.urdf.xacro'
    urdf = os.path.join(
        get_package_share_directory('volksbot_driver'),
        urdf_file_name)

    doc = xacro.process_file(urdf)
    robot_desc = doc.toprettyxml(indent='  ')
    
    config_dir = os.path.join(get_package_share_directory('volksbot_driver'), 'config/imu')
    phidgets_imu = ComposableNodeContainer(
        name='phidgets_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='phidgets_spatial',
                plugin='phidgets::SpatialRosI',
                name='phidgets_spatial',
                parameters=[os.path.join(config_dir, 'phidgets_imu.yaml')]),
        ],
        output='both',
    )

    # Generate lauch description consisting of launch time 
    # arguments and node configurations. Here we launch a 
    # robot state publisher and volksbot instance
    ld_list = [
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'num_wheels',
            default_value='4',
            description='Number of wheels of robot base'),
        DeclareLaunchArgument(
            'wheel_radius',
            default_value='0.0985',
            description='Volksbot wheel radius'),
        DeclareLaunchArgument(
            'publish_tf',
            default_value='false',
            description='Publish tf data'),
        DeclareLaunchArgument(
            'tf_prefix',
            default_value='',
            description='tf prefix. Attention: currently not used in driver!'),
        DeclareLaunchArgument(
            'joystick_config',
            default_value='8bitdo.config.yaml',
            description='Joystick configuration file found in /config/joystick'
        ),
         DeclareLaunchArgument(
            'channel_type',
            default_value=channel_type,
            description='Specifying channel type of lidar'),
        
        DeclareLaunchArgument(
            'serial_port',
            default_value=serial_port,
            description='Specifying usb port to connected lidar'),

        DeclareLaunchArgument(
            'serial_baudrate',
            default_value=serial_baudrate,
            description='Specifying usb port baudrate to connected lidar'),
        
        DeclareLaunchArgument(
            'frame_id',
            default_value=frame_id,
            description='Specifying frame_id of lidar'),

        DeclareLaunchArgument(
            'inverted',
            default_value=inverted,
            description='Specifying whether or not to invert scan data'),

        DeclareLaunchArgument(
            'angle_compensate',
            default_value=angle_compensate,
            description='Specifying whether or not to enable angle_compensate of scan data'),
        DeclareLaunchArgument(
            'scan_mode',
            default_value=scan_mode,
            description='Specifying scan mode of lidar'),

        OpaqueFunction(
            function=evaluate_joystick_config, 
            args=[LaunchConfiguration('joystick_config')]),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}],
            arguments=[urdf]),
        Node(
            package='volksbot_driver',
            executable='volksbot',
            name='volksbot',
            parameters=[{'num_wheels': num_wheels, 'wheel_radius': wheel_radius,'robot_description': robot_desc}],
            output='screen'),
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[{'channel_type':channel_type,
                         'serial_port': serial_port, 
                         'serial_baudrate': serial_baudrate, 
                         'frame_id': frame_id,
                         'inverted': inverted, 
                         'angle_compensate': angle_compensate}],
            output='screen'),
    ]

    launch_description = LaunchDescription(ld_list)
    launch_description.add_action(phidgets_imu)

    

    return launch_description
