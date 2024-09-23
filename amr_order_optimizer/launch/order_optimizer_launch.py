from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument  # Declares a launch argument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare a launch argument for the directory path
    directory_arg = DeclareLaunchArgument(
        'directory',
        default_value='/home/jagruth/Documents/amr_example_ROS/applicants_amr_example_1',  # You can provide a default path
        description='Absolute path to the directory containing orders and configuration subdirectories'
    )

    # Define the Node that will be launched
    order_optimizer_node = Node(
        package='amr_order_optimizer',  # Replace with your package name
        executable='OrderOptimizer',  # Replace with your C++ executable name
        name='OrderOptimizer',
        output='screen',
        parameters=[{
            'directory': LaunchConfiguration('directory')  # Pass directory parameter
        }]
    )

    # Return the LaunchDescription with the argument and the node to be launched
    return LaunchDescription([
        directory_arg,
        order_optimizer_node
    ])
