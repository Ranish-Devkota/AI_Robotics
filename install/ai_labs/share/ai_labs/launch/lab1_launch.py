from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='ai_labs',
            executable='square',
            name='square'
        ),
        ## Uncomment the following lines for the extra credit portion of this lab
        Node (
           package='ai_labs',
           executable='subscribe_pose',
           name='subscribe_pose',
           output='screen'
        ),
        # Node (
        #    package='ai_labs',
        #    executable='initials',
        #    name='initials',
        #    output='screen'
        # ),
    ])
