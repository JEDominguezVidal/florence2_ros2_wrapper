from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/camera/image_raw',
        description='The ROS2 topic to subscribe to for incoming images.'
    )

    example_node = Node(
        package='florence2_ros2',
        executable='florence2_service_call_example',
        name='florence2_service_example',
        output='screen',
        parameters=[{
            'image_topic': LaunchConfiguration('image_topic'),
        }]
    )

    return LaunchDescription([
        image_topic_arg,
        example_node
    ])
