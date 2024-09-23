from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare a launch argument for the directory path
    directory_arg = DeclareLaunchArgument(
        'directory',
        default_value='/home/jagruth/Documents/amr_example_ROS/applicants_amr_example_1',  #default path
        description='Absolute path to the directory containing orders and configuration subdirectories'
    )

    # Define the Node that will be launched
    order_optimizer_node = Node(
        package='amr_order_optimizer',  # Package name
        executable='OrderOptimizer',  # C++ executable name
        name='OrderOptimizer',
        output='screen',
        parameters=[{
            'directory': LaunchConfiguration('directory')  # Passing directory parameter
        }]
    )

    # Returns the LaunchDescription with the argument and the node
    return LaunchDescription([
        directory_arg,
        order_optimizer_node
    ])
