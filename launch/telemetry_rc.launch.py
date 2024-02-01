from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

    
def get_share_file(package_name, file_name):
    return os.path.join(get_package_share_directory(package_name), file_name)

def generate_launch_description():

    telemetry_rc_param_file = get_share_file(
        package_name='uva_telemetry', file_name='config/racecar.yaml'
    )
    telemetry_rc_param = DeclareLaunchArgument(
        'telemetry_rc_param_file',
        default_value=telemetry_rc_param_file,
        description='Path to config file for long_control'
    )

    use_sim_time=DeclareLaunchArgument("use_sim_time", default_value="false", description="Use the fake clock published on /clock for synchronization. Only needed in LGSVL.")
    namespace=DeclareLaunchArgument("ns", default_value="", description="What namespace to put stuff in")
    return LaunchDescription([
        telemetry_rc_param,
        use_sim_time,
        namespace,

        Node(
            package='uva_telemetry',
            executable='telemetry_rc',
            name = 'telemetry_rc',
            namespace=LaunchConfiguration(namespace.name),
            parameters=[LaunchConfiguration('telemetry_rc_param_file')],
            remappings=[
                ('vehicle_odom', 'vehicle/uva_odometry'),
                ('lateral_error', 'lateral_control/lateral_error'),
                ('desired_velocity_readout', 'long_control/set_desired_velocity'),
                ('accelerator_cmd', 'raptor_dbw_interface/accelerator_pedal_cmd'),
                ('brake_cmd', 'raptor_dbw_interface/brake_cmd'),
                ('gear_cmd', 'raptor_dbw_interface/gear_cmd'),
                ('steering_cmd', 'raptor_dbw_interface/steering_cmd'),
                ('acceleration', 'vehicle/uva_acceleration'),
                ('opp_trajectory', 'planner/ghost_pred'),
            ]
        ),

    ])
