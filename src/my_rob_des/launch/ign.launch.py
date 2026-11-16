import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler
from launch.actions import SetEnvironmentVariable
from launch.event_handlers import OnProcessExit


def generate_launch_description():
    package_share = get_package_share_directory('my_rob_des')

    world = LaunchConfiguration('world')
    world_name = LaunchConfiguration('world_name')
    robot_name = LaunchConfiguration('robot_name')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    yaw = LaunchConfiguration('yaw')
    use_sim_time = LaunchConfiguration('use_sim_time')
    headless = LaunchConfiguration('headless')

    world_file = os.path.join(package_share, 'worlds', 'warehouse_world.sdf')
    robot_file = os.path.join(package_share, 'urdf', 'mobile_robotcopy.urdf.xacro')
    robot_desc = xacro.process_file(robot_file).toxml()


    gazebo_gui = ExecuteProcess(
        cmd=['ign', 'gazebo', world, '-r'],
        output='screen',
        condition=UnlessCondition(headless),
    )
    gazebo_headless = ExecuteProcess(
        cmd=['ign', 'gazebo', '-s', world, '-r'],
        output='screen',
        condition=IfCondition(headless),
    )

    spawn_robot = Node(
        package='ros_ign_gazebo',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', robot_name,
            '-world', world_name,
            '-x', x,
            '-y', y,
            '-z', z,
            '-Y', yaw,
        ],
        output='screen',
    )

    jsb = Node(
        package='controller_manager', 
        executable='spawner',
        arguments=['joint_state_broadcaster'])
    vel = Node(package='controller_manager', 
               executable='spawner',
            arguments=['diff_drive_controller'])

    after_spawn = RegisterEventHandler(OnProcessExit(target_action=spawn_robot, on_exit=[jsb]))
    after_jsb = RegisterEventHandler(OnProcessExit(target_action=jsb, on_exit=[vel]))

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': use_sim_time,
        }],
    )

    bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/lidar@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
            '/camera/image@sensor_msgs/msg/Image[ignition.msgs.Image',
        ],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=world_file,
            description='SDF world file to load in Ignition Gazebo Fortress.',
        ),
        DeclareLaunchArgument(
            'world_name',
            default_value='my_world',
            description='World name used by the robot spawn service.',
        ),
        DeclareLaunchArgument(
            'robot_name',
            default_value='mobile_robot',
            description='Name assigned to the robot model in simulation.',
        ),
        DeclareLaunchArgument('x', default_value='0.0', description='Robot x position.'),
        DeclareLaunchArgument('y', default_value='0.0', description='Robot y position.'),
        DeclareLaunchArgument(
            'z',
            default_value='0.4',
            description='Robot z position; raises the wheels clear of the ground.',
        ),
        DeclareLaunchArgument('yaw', default_value='0.0', description='Robot yaw in radians.'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use the Ignition Gazebo simulation clock.',
        ),
        DeclareLaunchArgument(
            'headless',
            default_value='false',
            description='Run Ignition Gazebo without its graphical client.',
        ),
        SetEnvironmentVariable('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', '/opt/ros/humble/lib'),
        gazebo_gui,
        gazebo_headless,
        robot_state_publisher,
        bridge,
        after_spawn,     
        after_jsb,  
        TimerAction(period=3.0, actions=[spawn_robot]),
    ])
