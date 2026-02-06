from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    model_name_arg = DeclareLaunchArgument(
        'model_name',
        default_value='microsoft/Florence-2-large-ft',
        description='The Florence-2 model variant to load (e.g. microsoft/Florence-2-base-ft)'
    )
    
    continuous_task_arg = DeclareLaunchArgument(
        'continuous_task',
        default_value='',
        description='A task to run continuously on every frame, e.g. <OD> for Object Detection. Leave empty for on-demand execution only.'
    )
    
    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/camera/image_raw',
        description='The ROS2 topic to subscribe to for incoming images.'
    )

    florence2_node = Node(
        package='florence2_ros2',
        executable='florence2_node',
        name='florence2_node',
        output='screen',
        parameters=[{
            'model_name': LaunchConfiguration('model_name'),
            'continuous_task': LaunchConfiguration('continuous_task'),
            'image_topic': LaunchConfiguration('image_topic'),
        }]
    )

    return LaunchDescription([
        model_name_arg,
        continuous_task_arg,
        image_topic_arg,
        florence2_node
    ])
