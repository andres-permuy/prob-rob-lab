from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    node_name_arg = DeclareLaunchArgument(
        'node_name',
        default_value='video_processor',
        description='Video processor node name'
    )
    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/camera/image_raw',
        description='Topic to which the video processor subscribes'
    )
    goodfeature_image_topic_arg = DeclareLaunchArgument(
        'goodfeature_image_topic',
        default_value='/goodfeature/image_raw',
        description='Visualization topic for goodfeature detector'
    )
    gray_image_topic_arg = DeclareLaunchArgument(
        'gray_image_topic',
        default_value='/gray/image_raw',
        description='Visualization topic for grayscale interim image'
    )
    goodfeature_corners_topic_arg = DeclareLaunchArgument(
        'goodfeature_corners_topic',
        default_value='/goodfeature/corners',
        description='Output topic for goodfeature detector'
    )
    max_features_arg = DeclareLaunchArgument(
        'max_features',
        default_value='50',
        description='Maximum number of features to extract'
    )
    run_color_filter_arg = DeclareLaunchArgument(
        'run_color_filter',
        default_value='false',
        description='Set to true to run the color filter first'
    )
    min_h_arg = DeclareLaunchArgument(
        'min_h',
        default_value='0',
        description='Minimum hue value for HSV filter'
    )
    max_h_arg = DeclareLaunchArgument(
        'max_h',
        default_value='360',
        description='Maximum hue value for HSV filter'
    )
    min_s_arg = DeclareLaunchArgument(
        'min_s',
        default_value='0',
        description='Minimum saturation value for HSV filter'
    )
    max_s_arg = DeclareLaunchArgument(
        'max_s',
        default_value='255',
        description='Maximum saturation value for HSV filter'
    )
    min_v_arg = DeclareLaunchArgument(
        'min_v',
        default_value='0',
        description='Minimum value (brightness) for HSV filter'
    )
    max_v_arg = DeclareLaunchArgument(
        'max_v',
        default_value='255',
        description='Maximum value (brightness) for HSV filter'
    )

    return LaunchDescription([
        node_name_arg,
        image_topic_arg,
        goodfeature_image_topic_arg,
        gray_image_topic_arg,
        goodfeature_corners_topic_arg,
        max_features_arg,
        run_color_filter_arg,
        min_h_arg,
        max_h_arg,
        min_s_arg,
        max_s_arg,
        min_v_arg,
        max_v_arg,
        # Node configuration with the image_topic parameter
        Node(
            package='prob_rob_vision',
            executable='video_processor',
            name=LaunchConfiguration('node_name'),
            output='screen',
            parameters=[
                {'image_topic': LaunchConfiguration('image_topic')},
                {'goodfeature_image_topic': LaunchConfiguration('goodfeature_image_topic')},
                {'gray_image_topic': LaunchConfiguration('gray_image_topic')},
                {'goodfeature_corners_topic': LaunchConfiguration('goodfeature_corners_topic')},
                {'max_features': LaunchConfiguration('max_features')},
                {'run_color_filter': LaunchConfiguration('run_color_filter')},
                {'min_h': LaunchConfiguration('min_h')},
                {'max_h': LaunchConfiguration('max_h')},
                {'min_s': LaunchConfiguration('min_s')},
                {'max_s': LaunchConfiguration('max_s')},
                {'min_v': LaunchConfiguration('min_v')},
                {'max_v': LaunchConfiguration('max_v')}
            ]
        )
    ])
