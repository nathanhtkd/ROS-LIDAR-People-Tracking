from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, EmitEvent, RegisterEventHandler
from launch_ros.actions import Node
from launch.event_handlers import *
from launch.events import *
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

def generate_launch_description():
    # Create arguments
    bag_in_arg = DeclareLaunchArgument(
        'bag_in',
        default_value='example1',
        description='Name of the bag file to play'
    )

    # bag_out_arg = DeclareLaunchArgument(
    #   'bag_out',
    #   description='Name of the bag file to output messages to'
    # )

    # Take in arguements
    bag_to_play = LaunchConfiguration('bag_in')
    # bag_to_output = LaunchConfiguration('bag_out')

    # Execute handler for bag playing
    bag_path = PathJoinSubstitution(['bags', bag_to_play])
    play_bag = ExecuteProcess(
      cmd=['ros2', 'bag', 'play', bag_path],
      name='play_bag'
    )


    # Create nodes to start up for lidar scan processing
    lidar_processor_node = Node(
        package='lidar_listener',
        executable='lidar_processor',
        name='lidar_processor_node'
    )

    cluster_node = Node(
        package='lidar_listener',
        executable='point_cluster',
        name='point_cluster_node'
    )

    person_count_node = Node(
        package='lidar_listener',
        executable='person_tracker',
        name='person_tracker_node'
    )

    event_handler = OnProcessExit(
      target_action=play_bag,
      on_exit=[EmitEvent(event=Shutdown())]
    )

    terminate_at_end = RegisterEventHandler(event_handler)

    ld = LaunchDescription([
        bag_in_arg,
        lidar_processor_node,
        cluster_node,
        person_count_node,
        play_bag,
        terminate_at_end,
    ])

    return ld
