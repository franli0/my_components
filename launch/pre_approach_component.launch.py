from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    """Generate launch description for pre approach."""
    
    # Define paths to RViz config files
    pkg_share = FindPackageShare('my_components')
    rviz_config_path = PathJoinSubstitution([pkg_share, 'rviz', 'attach_to_shelf.rviz'])
    
    container = ComposableNodeContainer(
        name='component_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='my_components',
                plugin='my_components::PreApproach',
                name='pre_approach'),
        ],
        output='screen',
    )

    # Launch RViz with appropriate configuration
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )
    
    return LaunchDescription([
        container,
        rviz_node
    ])