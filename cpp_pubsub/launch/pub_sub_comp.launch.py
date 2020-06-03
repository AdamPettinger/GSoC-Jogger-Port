import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Generate launch description with multiple components."""
    container = ComposableNodeContainer(
            node_name='my_container',
            node_namespace='',
            package='rclcpp_components',
            node_executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='cpp_pubsub',
                    node_plugin='MinimalPublisher',
                    node_name='talker'),
                ComposableNode(
                    package='cpp_pubsub',
                    node_plugin='MinimalSubscriber',
                    node_name='listener')
            ],
            output='screen',
    )

    return launch.LaunchDescription([container])