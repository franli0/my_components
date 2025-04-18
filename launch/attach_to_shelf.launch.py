from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer

def generate_launch_description():
    # Define paths to RViz config files
    pkg_share = FindPackageShare('my_components')
    rviz_config_path = PathJoinSubstitution([pkg_share, 'rviz', 'attach_to_shelf.rviz'])
    
    # Create a container with both PreApproach and AttachServer components
    container = ComposableNodeContainer(
        name='my_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            # PreApproach component - loaded via launch file
            ComposableNode(
                package='my_components',
                plugin='my_components::PreApproach',
                name='pre_approach',
                # Disable intraprocess communication to avoid QoS issues
                extra_arguments=[{'use_intra_process_comms': False}],
                parameters=[{'standalone_mode': False}],  # Explicitly set to not standalone
            ),
            # AttachServer component - loaded via launch file (manual composition)
            ComposableNode(
                package='my_components',
                plugin='my_components::AttachServer',
                name='attach_server',
                # Disable intraprocess communication to avoid QoS issues
                extra_arguments=[{'use_intra_process_comms': False}],
            )
        ],
        output='screen'
    )
    
    # Launch RViz with appropriate configuration
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path]
    )
    
    # Return the launch description
    return LaunchDescription([
        container,
        rviz_node
    ])