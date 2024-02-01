from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

    
def get_share_file(package_name, file_name):
    return os.path.join(get_package_share_directory(package_name), file_name)

def generate_launch_description():

    telemetry_bs_param_file = get_share_file(
        package_name='uva_telemetry', file_name='config/racecar.yaml'
    )
    telemetry_bs_param = DeclareLaunchArgument(
        'telemetry_bs_param_file',
        default_value=telemetry_bs_param_file
    )

    return LaunchDescription([
        telemetry_bs_param,
        Node(
            package='uva_telemetry',
            executable='telemetry_bs',
            name = 'telemetry_bs',
            parameters=[LaunchConfiguration('telemetry_bs_param_file')],
        ),

    ])
