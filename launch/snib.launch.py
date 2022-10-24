import launch
import launch.actions
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
import os
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # declare the launch args to read for this file

    config = os.path.join(
        get_package_share_directory('riptide_SNIB'),
        'config',
        'config.yaml'
        )

    return launch.LaunchDescription([
        DeclareLaunchArgument(
            "log_level", 
            default_value="INFO",
            description="log level to use",
        ),

        DeclareLaunchArgument(
            "launch_simulink",
            default_value="False",
            description="wether or not to launch simulink"
            ),

        # create the nodes    
        launch_ros.actions.Node(
            package='riptide_SNIB',
            executable='SNIB',
            name='riptide_SNIB',
            respawn=True,
            output='screen',
            
            # use the parameters on the node
            parameters = [
                config
            ]
        )
    ])