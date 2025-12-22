from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
import os

def generate_launch_description():

    omni_share = FindPackageShare('omni3_control').find('omni3_control')
    omni_path = os.path.join(omni_share, 'launch', 'bb300.launch.py')

    pin_rec_share = FindPackageShare('pin_recognition').find('pin_recognition')
    pin_rec_path = os.path.join(pin_rec_share, 'launch', 'pin_follow.launch.py')


    linear_motor_controller_node = Node(
            package='linear_motor_controller',
            name='linear_motor_controller_node',
            executable='linear_motor_controller_node',
            parameters=[PathJoinSubstitution([FindPackageShare('linear_motor_controller'), 'config', 'linear_motor_params.yaml'])],
            output='screen',
    )

    return LaunchDescription([
        linear_motor_controller_node,
        IncludeLaunchDescription(
        PythonLaunchDescriptionSource(omni_path)
        ),
        IncludeLaunchDescription(
        PythonLaunchDescriptionSource(pin_rec_path)
        ),
    ])