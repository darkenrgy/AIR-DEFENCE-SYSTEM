from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['gz', 'sim', '-v4', 
                 'mayuri/mayuri/gazebo/worlds/empty_world.sdf'],
            output='screen'
        )
    ])
