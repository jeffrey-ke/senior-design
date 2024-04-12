from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='hardware_bridge',
            executable='hbnode',
            name='hardware_bridge_node'
        ),
        Node(
            package='state_machine',
            executable='state_machine',
            name='StateMachineNode'
        ),
        Node(
            package='velocity_commander',
            executable='vlcmdnode',
            name='VelocityCommanderNode'
        ),
        Node(
            package='logger',
            executable='logger',
            name='LoggerNode'
        )
    ])