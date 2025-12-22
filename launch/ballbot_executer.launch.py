from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    ballbot_client_node = Node(
            package='ballbot_executer',
            name='ballbot_client_node',
            executable='ballbot_client_node',
            parameters=[PathJoinSubstitution([FindPackageShare('ballbot_executer'), 'config', 'ballbot_executer_params.yaml'])],
            output='screen',
    )

    return LaunchDescription([
        ballbot_client_node,
    ])