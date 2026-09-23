"""Volksbot base stack: driver, URDF, IMU, EKF, lidar, slam_toolbox, Nav2.

Everything a frontier-exploration solution needs underneath it, and none of the
exploration itself. What comes out of a full run is a robot that maps what it
is driven or sent through, and that accepts navigation goals — from RViz, from
a script, or from whatever you write next.

COMPONENTS
Five, bottom to top, and every one is on by default. Switch off what you do not
want:

    driver          robot_state_publisher + the motor driver
    localization    Phidgets IMU + the EKF that publishes odom -> base_footprint
    lidar           sick, rplidar, or none
    slam            slam_toolbox: /map and map -> odom
    nav2            the six Nav2 servers and their lifecycle manager

    ros2 launch volksbot_bringup bringup.launch.py
        The whole thing. Nothing drives itself — Nav2 goes where it is told and
        nothing here is telling it. Send it somewhere with RViz's "Nav2 Goal"
        tool, or teleop with use_joystick:=true.

    ros2 launch volksbot_bringup bringup.launch.py nav2:=false
        Drivers and a map, nothing above them. Start here: if the map does not
        look right while you drive it by hand, nothing above this will help.

    ros2 launch volksbot_bringup bringup.launch.py \
         driver:=false localization:=false lidar:=none slam:=false
        Nav2 alone, so it can be restarted without restarting the motor driver.
        Needs /scan, /odom, /map and the map->odom->base_footprint->laser TF
        chain from somewhere else — normally a second bringup with nav2:=false.

WHAT IT PUBLISHES, AND WHAT YOUR CODE CAN USE
    /scan                       lidar
    /odom                       wheel odometry from the driver
    map -> odom -> base_footprint   slam_toolbox + EKF
    /map                        slam_toolbox, the map itself
    /global_costmap/costmap     Nav2, and /global_costmap/costmap_updates
    /navigate_to_pose           Nav2 action server: where to drive
    /cmd_vel                    what the driver listens to, in the end

TUNING LIVES IN config/
    nav2_params.yaml            the whole Nav2 stack, one file
    slam_toolbox/online_async.yaml
    ekf.yaml                    what the EKF fuses, and how
    bt/                         behavior trees, forked from the Nav2 defaults
"""

import os

from ament_index_python.packages import get_package_share_directory
from ament_index_python.resources import get_resource
from launch import LaunchDescription, LaunchContext
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction)
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

import xacro


def _is_true(context, cfg):
    return context.perform_substitution(cfg).lower() in ('true', '1', 'yes')


def _tristate(context, cfg, auto):
    """Resolve an auto/true/false argument, where 'auto' means `auto`."""
    value = context.perform_substitution(cfg).lower()
    if value == 'auto':
        return auto
    if value in ('true', '1', 'yes'):
        return True
    if value in ('false', '0', 'no'):
        return False
    raise RuntimeError(f"Expected auto, true or false, got '{value}'")


def _component_class(pkg, plugin):
    """Resolve a composable node's registered class name on THIS machine.

    Nav2 has renamed these across distros — Jazzy registers
    `behavior_server::BehaviorServer`, later distros renamed it to
    `nav2_behaviors::BehaviorServer` — and a name this install does not have
    does not fail the launch. The container logs

        Failed to find class with the requested plugin name '...'

    once, that server never appears, and lifecycle_manager then sits in
    "Waiting for service behavior_server/get_state..." forever while everything
    else looks healthy. Hardcoding either name is a guess; the rclcpp_components
    registry in the ament index is the authority, so ask it and match on the
    class name after the last `::`, which has been stable.

    Falls back to the name passed in, so a package whose registry cannot be read
    behaves exactly as it did before this function existed.
    """
    want = plugin.rsplit('::', 1)[-1]
    try:
        content, _ = get_resource('rclcpp_components', pkg)
    except Exception:
        return plugin
    for line in content.splitlines():
        registered = line.strip().split(';')[0]
        if registered and registered.rsplit('::', 1)[-1] == want:
            return registered
    return plugin


def _config_dir():
    return os.path.join(
        get_package_share_directory('volksbot_bringup'), 'config')


def evaluate_robot_description(context: LaunchContext, driver, use_sim_time,
                               num_wheels, wheel_radius,
                               laser_x_offset, laser_y_offset, laser_z_offset,
                               laser_yaw_offset, imu_x_offset, imu_y_offset, imu_z_offset):
    """robot_state_publisher + the motor driver. Component: driver.

    The URDF is a xacro, and the offsets below are substituted into it. They
    are PLACEHOLDERS: measure them on the robot in front of you before you
    trust anything the costmaps say about where an obstacle is.
    """
    if not _is_true(context, driver):
        return []

    volksbot_driver_dir = get_package_share_directory('volksbot_driver')
    urdf_file = os.path.join(volksbot_driver_dir, 'urdf', 'volksbot.urdf.xacro')

    mappings = {
        'laser_x_offset': context.perform_substitution(laser_x_offset),
        'laser_y_offset': context.perform_substitution(laser_y_offset),
        'laser_z_offset': context.perform_substitution(laser_z_offset),
        'laser_yaw_offset': context.perform_substitution(laser_yaw_offset),
        'imu_x_offset': context.perform_substitution(imu_x_offset),
        'imu_y_offset': context.perform_substitution(imu_y_offset),
        'imu_z_offset': context.perform_substitution(imu_z_offset),
    }

    doc = xacro.process_file(urdf_file, mappings=mappings)
    robot_desc = doc.toprettyxml(indent='  ')

    use_sim_time_value = context.perform_substitution(use_sim_time).lower() == 'true'

    return [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time_value,
                         'robot_description': robot_desc}]),
        Node(
            package='volksbot_driver',
            executable='volksbot',
            name='volksbot',
            parameters=[
                os.path.join(volksbot_driver_dir, 'config', 'volksbot.yaml'),
                {
                    'num_wheels': num_wheels,
                    'wheel_radius': wheel_radius,
                    # The EKF below owns odom -> base_footprint. Two publishers
                    # of one transform is a fight, not a fallback.
                    'publish_tf': True,
                    'robot_description': robot_desc,
                },
            ],
            output='screen'),
    ]


def evaluate_localization(context: LaunchContext, localization, use_sim_time):
    """Phidgets IMU + the EKF that fuses it with wheel odometry. Component:
    localization.

    This is what publishes odom -> base_footprint. Without it there is no
    odometry frame, and both slam_toolbox and Nav2 sit waiting for a transform
    that never arrives.
    """
    if not _is_true(context, localization):
        return []

    volksbot_driver_dir = get_package_share_directory('volksbot_driver')

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
                parameters=[os.path.join(
                    volksbot_driver_dir, 'config/imu', 'phidgets_imu.yaml')]),
        ],
        output='both',
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(_config_dir(), 'ekf.yaml'),
                    {'use_sim_time': use_sim_time}],
    )

    return [phidgets_imu, ekf_node]


def evaluate_lidar_config(context: LaunchContext, lidar_choice, serial_port,
                          serial_baudrate, channel_type):
    """The lidar. Component: lidar.

    This component's switch and its choice of hardware are the same argument:
    name a model, or lidar:=none to turn it off.
    """
    choice = context.perform_substitution(lidar_choice).lower()
    if choice == 'none':
        return []

    if choice == 'sick':
        sick_tim_dir = get_package_share_directory('sick_tim')
        return [Node(
            package='sick_tim',
            executable='sick_tim551_2050001',
            name='sick_driver',
            parameters=[os.path.join(sick_tim_dir, 'cfg', 'sick_usb.yaml')],
            output='screen')]
    elif choice == 'rplidar':
        return [Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[{
                'channel_type': channel_type,
                'serial_port': serial_port,
                'serial_baudrate': serial_baudrate,
                'frame_id': 'laser',
                'inverted': False,
                'angle_compensate': True,
            }],
            output='screen')]
    else:
        raise RuntimeError(
            f"Unknown lidar '{choice}', expected sick, rplidar or none")


def evaluate_joystick_config(context: LaunchContext, use_joystick, joystick_config):
    """teleop_twist_joy, for driving the robot by hand. Not a component of the
    stack: it is a tool, and it is off unless you ask for it."""
    if not _is_true(context, use_joystick):
        return []

    teleop_twist_dir = get_package_share_directory('teleop_twist_joy')
    config_file_name = context.perform_substitution(joystick_config)
    joystick_config_file = os.path.join(
        get_package_share_directory('volksbot_driver'), 'config/joystick/',
        config_file_name)

    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(teleop_twist_dir, 'launch', 'teleop-launch.py')),
        launch_arguments={'config_filepath': joystick_config_file}.items(),
    )]


def evaluate_slam_config(context: LaunchContext, slam, slam_config, use_sim_time):
    """slam_toolbox, from config/slam_toolbox/<slam_config>. Component: slam.

    Publishes /map and the map -> odom correction. slam:=false when something
    else already does — two nodes publishing /map is not a useful state.
    """
    if not _is_true(context, slam):
        return []

    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    slam_config_file = os.path.join(
        _config_dir(), 'slam_toolbox', context.perform_substitution(slam_config))

    # slam_toolbox's own launch file owns the lifecycle transitions, so include
    # it rather than reimplementing them.
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_dir, 'launch', 'localization_launch.py')),
        launch_arguments={
            'slam_params_file': slam_config_file,
            'use_sim_time': context.perform_substitution(use_sim_time),
        }.items(),
    )]


def evaluate_nav2(context: LaunchContext, nav2, use_sim_time, use_composition):
    """The Nav2 servers, on config/nav2_params.yaml. Component: nav2.

    Six lifecycle nodes and the manager that brings them up. What they need
    below them is /scan, /odom, /map and TF; what they offer above them is the
    /navigate_to_pose action and the global costmap.
    """
    if not _is_true(context, nav2):
        return []

    config = _config_dir()
    sim_time = _is_true(context, use_sim_time)
    with_composition = _is_true(context, use_composition)

    common = [os.path.join(config, 'nav2_params.yaml'), {'use_sim_time': sim_time}]

    # The behavior trees are forks of the Nav2 defaults with BackUp swapped for
    # a Spin: the 270 deg SICK cannot see behind the robot, so recovery must
    # never reverse. Reversing blind into what the costmap has already
    # forgotten is how you hit the one chair leg nobody scanned.
    bt_dir = os.path.join(config, 'bt')
    bt_params = [{
        'default_nav_to_pose_bt_xml': os.path.join(
            bt_dir, 'navigate_to_pose_no_reverse.xml'),
        'default_nav_through_poses_bt_xml': os.path.join(
            bt_dir, 'navigate_through_poses_no_reverse.xml')}]

    # The velocity chain, in order:
    #   controller/behavior -> cmd_vel_nav -> velocity_smoother -> cmd_vel
    # and /cmd_vel is the only topic the motor driver listens to. Break a link
    # in this chain and Nav2 plans a perfectly good path that nothing drives.
    #
    # One description per server, so the composed and non-composed builds
    # cannot drift apart. `plugin` is only read in the composed case.
    #   (package, executable, plugin, name, parameters, remappings)
    servers = [
        ('nav2_controller', 'controller_server',
         'nav2_controller::ControllerServer', 'controller_server',
         common, [('cmd_vel', 'cmd_vel_nav')]),
        ('nav2_smoother', 'smoother_server',
         'nav2_smoother::SmootherServer', 'smoother_server', common, []),
        ('nav2_planner', 'planner_server',
         'nav2_planner::PlannerServer', 'planner_server', common, []),
        ('nav2_behaviors', 'behavior_server',
         'nav2_behaviors::BehaviorServer', 'behavior_server',
         common, [('cmd_vel', 'cmd_vel_nav')]),
        ('nav2_bt_navigator', 'bt_navigator',
         'nav2_bt_navigator::BtNavigator', 'bt_navigator',
         common + bt_params, []),
        ('nav2_velocity_smoother', 'velocity_smoother',
         'nav2_velocity_smoother::VelocitySmoother', 'velocity_smoother',
         common, [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')]),
    ]

    lifecycle_params = [{
        'use_sim_time': sim_time,
        'autostart': True,
        'node_names': ['controller_server', 'smoother_server', 'planner_server',
                       'behavior_server', 'velocity_smoother', 'bt_navigator']}]

    if not with_composition:
        return [
            Node(package=pkg, executable=exe, name=node_name,
                 output='screen', parameters=params, remappings=remaps)
            for pkg, exe, _plugin, node_name, params, remaps in servers
        ] + [
            Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                 name='lifecycle_manager_navigation', output='screen',
                 parameters=lifecycle_params)]

    return [ComposableNodeContainer(
        name='nav2_container',
        namespace='',
        package='rclcpp_components',
        # _isolated gives each component its own callback thread. The
        # single-threaded container would serialise the controller loop behind
        # the planner, which is exactly the stall composition is meant to
        # remove.
        executable='component_container_isolated',
        # THE CONTAINER NEEDS THE PARAMS FILE ITSELF, and this is the single
        # least obvious line in this file.
        #
        # A ComposableNode's `parameters` are delivered through the container's
        # LoadNode service as overrides for that one node, and launch_ros
        # resolves a params *file* against that node's fully qualified name
        # before sending it. So planner_server gets the `planner_server:`
        # section and nothing else.
        #
        # But the costmaps are not composable nodes. Costmap2DROS is a second
        # rclcpp node that planner_server and controller_server each construct
        # internally, named global_costmap/global_costmap and
        # local_costmap/local_costmap. Nothing in the LoadNode request reaches
        # them, so the `global_costmap:` and `local_costmap:` sections of
        # nav2_params.yaml are simply dropped and both costmaps come up on
        # Nav2's built-in defaults.
        #
        # Passing the file here puts it on the container process's command line
        # as --params-file, which every node built inside it reads as global
        # arguments — the child costmaps included. This is what nav2_bringup's
        # own composed launch does.
        #
        # Without it the stack activates and looks healthy, and then:
        #   - obstacle_layer logs "Subscribed to Topics: " with nothing after
        #     it, because observation_sources defaults to empty, so neither
        #     costmap ever sees /scan
        #   - local_costmap is 5x5 m and non-rolling, giving "Robot is out of
        #     bounds of the costmap"
        #   - always_send_full_costmap defaults to False, so anything
        #     subscribing to /global_costmap/costmap waits forever
        parameters=common,
        composable_node_descriptions=[
            ComposableNode(package=pkg, plugin=_component_class(pkg, plugin),
                           name=node_name, parameters=params, remappings=remaps)
            for pkg, _exe, plugin, node_name, params, remaps in servers
        ] + [
            ComposableNode(
                package='nav2_lifecycle_manager',
                plugin=_component_class(
                    'nav2_lifecycle_manager',
                    'nav2_lifecycle_manager::LifecycleManager'),
                name='lifecycle_manager_navigation',
                parameters=lifecycle_params),
        ],
        output='screen')]


def evaluate_rviz(context: LaunchContext, rviz, use_sim_time):
    """RViz on the project config. Not a component — it observes, it does not
    run.

    Off by default because the usual place to run it is the laptop, not the
    robot: it is a plain ROS 2 node, so it sees everything above over DDS as
    long as ROS_DOMAIN_ID matches.
    """
    if not _is_true(context, rviz):
        return []

    return [Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', os.path.join(
            get_package_share_directory('volksbot_bringup'), 'rviz',
            'volksbot.rviz')],
        parameters=[{'use_sim_time': _is_true(context, use_sim_time)}],
        output='screen')]


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    driver = LaunchConfiguration('driver')
    localization = LaunchConfiguration('localization')
    lidar = LaunchConfiguration('lidar')
    rviz = LaunchConfiguration('rviz')

    num_wheels = LaunchConfiguration('num_wheels')
    wheel_radius = LaunchConfiguration('wheel_radius')
    slam_config = LaunchConfiguration('slam_config')

    laser_x_offset = LaunchConfiguration('laser_x_offset')
    laser_y_offset = LaunchConfiguration('laser_y_offset')
    laser_z_offset = LaunchConfiguration('laser_z_offset')
    laser_yaw_offset = LaunchConfiguration('laser_yaw_offset')
    imu_x_offset = LaunchConfiguration('imu_x_offset')
    imu_y_offset = LaunchConfiguration('imu_y_offset')
    imu_z_offset = LaunchConfiguration('imu_z_offset')

    channel_type = LaunchConfiguration('channel_type')
    serial_port = LaunchConfiguration('serial_port')
    serial_baudrate = LaunchConfiguration('serial_baudrate')

    return LaunchDescription([
        # the components. All on by default; switch off what you do not want.
        DeclareLaunchArgument(
            'driver', default_value='true', choices=['true', 'false'],
            description='robot_state_publisher + the motor driver. false '
                        'leaves the motors alone while you restart what is '
                        'above them'),
        DeclareLaunchArgument(
            'localization', default_value='true', choices=['true', 'false'],
            description='Phidgets IMU + the EKF that publishes '
                        'odom -> base_footprint'),
        DeclareLaunchArgument(
            'lidar', default_value='sick', choices=['sick', 'rplidar', 'none'],
            description='Which lidar to bring up, or none to run without one'),
        DeclareLaunchArgument(
            'slam', default_value='true', choices=['true', 'false'],
            description='slam_toolbox: /map and map -> odom'),
        DeclareLaunchArgument(
            'nav2', default_value='true', choices=['true', 'false'],
            description='The Nav2 servers plus their lifecycle manager, on '
                        'config/nav2_params.yaml'),
        DeclareLaunchArgument(
            'rviz', default_value='false',
            description='Start RViz on rviz/volksbot.rviz'),

        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        # hardware
        DeclareLaunchArgument(
            'num_wheels', default_value='4',
            description='Number of wheels of the robot base'),
        DeclareLaunchArgument(
            'wheel_radius', default_value='0.13',
            description='Wheel radius in metres. Wrong here means odometry '
                        'that is wrong by a constant factor'),
        DeclareLaunchArgument(
            'channel_type', default_value='serial',
            description='rplidar only: channel type of lidar'),
        DeclareLaunchArgument(
            'serial_port', default_value='/dev/ttyUSB0',
            description='rplidar only: usb port of connected lidar'),
        DeclareLaunchArgument(
            'serial_baudrate', default_value='256000',
            description='rplidar only: usb port baudrate of connected lidar'),
        DeclareLaunchArgument(
            'use_joystick', default_value='false',
            description='Bring up teleop_twist_joy for driving by hand'),
        DeclareLaunchArgument(
            'joystick_config', default_value='8bitdo.config.yaml',
            description='Joystick config file in volksbot_driver/config/joystick'),

        # PLACEHOLDERS: these go into the URDF. Measure them on the robot.
        DeclareLaunchArgument(
            'laser_x_offset', default_value='0.0',
            description='PLACEHOLDER: base_link -> laser x offset, measure it'),
        DeclareLaunchArgument(
            'laser_y_offset', default_value='0.0',
            description='PLACEHOLDER: base_link -> laser y offset, measure it'),
        DeclareLaunchArgument(
            'laser_z_offset', default_value='0.20',
            description='PLACEHOLDER: base_link -> laser z offset, measure it'),
        DeclareLaunchArgument(
            'laser_yaw_offset', default_value='0.0',
            description='PLACEHOLDER: base_link -> laser yaw offset, measure it'),
        DeclareLaunchArgument(
            'imu_x_offset', default_value='0.0',
            description='PLACEHOLDER: base_link -> imu x offset, measure it'),
        DeclareLaunchArgument(
            'imu_y_offset', default_value='0.0',
            description='PLACEHOLDER: base_link -> imu y offset, measure it'),
        DeclareLaunchArgument(
            'imu_z_offset', default_value='0.05',
            description='PLACEHOLDER: base_link -> imu z offset, measure it'),

        # mapping and navigation
        DeclareLaunchArgument(
            'slam_config', default_value='online_async.yaml',
            description='slam_toolbox config file in config/slam_toolbox'),
        DeclareLaunchArgument(
            'use_composition', default_value='false',
            description='Run the Nav2 servers as components in one process '
                        'with intra-process comms instead of six processes '
                        'over DDS. Cheaper, but one crash takes the whole '
                        'stack with it and the logs interleave, so false is '
                        'the one to debug with'),

        OpaqueFunction(
            function=evaluate_robot_description,
            args=[driver, use_sim_time, num_wheels, wheel_radius,
                  laser_x_offset, laser_y_offset, laser_z_offset, laser_yaw_offset,
                  imu_x_offset, imu_y_offset, imu_z_offset]),

        OpaqueFunction(
            function=evaluate_localization,
            args=[localization, use_sim_time]),

        OpaqueFunction(
            function=evaluate_lidar_config,
            args=[lidar, serial_port, serial_baudrate, channel_type]),

        OpaqueFunction(
            function=evaluate_joystick_config,
            args=[LaunchConfiguration('use_joystick'),
                  LaunchConfiguration('joystick_config')]),

        OpaqueFunction(
            function=evaluate_slam_config,
            args=[LaunchConfiguration('slam'), slam_config, use_sim_time]),

        OpaqueFunction(
            function=evaluate_nav2,
            args=[LaunchConfiguration('nav2'), use_sim_time,
                  LaunchConfiguration('use_composition')]),

        OpaqueFunction(
            function=evaluate_rviz,
            args=[rviz, use_sim_time]),
    ])
