import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get the launch directory
    pkg_dir = get_package_share_directory('stanley_controller')

    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(pkg_dir, 'config', 'stanley_params.yaml'),
        description='Path to the config file'
    )

    # Create the stanley controller node
    stanley_controller_node = Node(
        package='stanley_controller',
        executable='stanley_controller_node',
        name='stanley_controller',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[
            # Add any topic remappings here if needed
            # ('/input_topic', '/remapped_input_topic'),
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        config_file_arg,
        stanley_controller_node
    ])
