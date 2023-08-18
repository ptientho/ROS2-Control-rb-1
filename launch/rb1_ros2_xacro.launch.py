import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_prefix
from launch_ros.descriptions import ParameterValue
from launch.actions import RegisterEventHandler, TimerAction, GroupAction, ExecuteProcess
from launch.event_handlers import OnProcessExit


def generate_launch_description():

    delay_time_seconds = 20.0
    description_package_name = "rb1_ros2_description"
    install_dir = get_package_prefix(description_package_name)

    world_file_path = "worlds/ros2_online_workshop_v2.world"
    world_file_pkg = "warehouse_world"

    # This is to find the models inside the models folder in rb1_ros2_description package
    gazebo_models_path = os.path.join(description_package_name, 'models')
    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] = os.environ['GAZEBO_MODEL_PATH'] + \
            ':' + install_dir + '/share' + ':' + gazebo_models_path
    else:
        os.environ['GAZEBO_MODEL_PATH'] = install_dir + \
            "/share" + ':' + gazebo_models_path

    if 'GAZEBO_PLUGIN_PATH' in os.environ:
        os.environ['GAZEBO_PLUGIN_PATH'] = os.environ['GAZEBO_PLUGIN_PATH'] + \
            ':' + install_dir + '/lib'
    else:
        os.environ['GAZEBO_PLUGIN_PATH'] = install_dir + '/lib'

    print("GAZEBO MODELS PATH=="+str(os.environ["GAZEBO_MODEL_PATH"]))
    print("GAZEBO PLUGINS PATH=="+str(os.environ["GAZEBO_PLUGIN_PATH"]))

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Declare a new launch argument for the world file
    world_file_arg = DeclareLaunchArgument(
        'world',
        default_value=[get_package_share_directory(
            world_file_pkg), world_file_path],
        description='Path to the Gazebo world file'
    )

    # Define the launch arguments for the Gazebo launch file
    gazebo_launch_args = {
        'verbose': 'false',
        'pause': 'false',
        'world': LaunchConfiguration('world')
    }

   # Include the Gazebo launch file with the modified launch arguments
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
        launch_arguments=gazebo_launch_args.items(),
    )
    
    # Define the robot model files to be used
    robot_desc_file = "rb1_ros2_base.urdf.xacro"
    robot_desc_path = os.path.join(get_package_share_directory(
        "rb1_ros2_description"), "xacro", robot_desc_file)

    robot_name_1 = "rb1_robot"

    rsp_robot1 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'frame_prefix': robot_name_1+'/', 'use_sim_time': use_sim_time,
                     'robot_description': ParameterValue(Command(['xacro ', robot_desc_path]), value_type=str)}],
        output="screen"
    )
    
    spawn_robot1 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', robot_name_1, '-x', '0.0', '-y', '0.0', '-z', '0.0',
                   '-topic', '/robot_description']
    )
    
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    robot_diffdrive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=["diffbot_base_controller", "-c", "/controller_manager"],
    )

    robot_lift_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=["forward_effort_controller", "-c", "/controller_manager"],
    )

    joint = TimerAction(
        period=40.0,
        actions=[joint_state_broadcaster_spawner],
    )

    return LaunchDescription([
        world_file_arg,
        gazebo,
        rsp_robot1,
        spawn_robot1,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_robot1,
                on_exit=[joint],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[robot_diffdrive_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=robot_diffdrive_spawner,
                on_exit=[robot_lift_spawner],
            )
        ),
    ])
