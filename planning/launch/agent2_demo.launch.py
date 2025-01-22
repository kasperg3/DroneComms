from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    ld = LaunchDescription()
    
    planning_node = Node(
        package='planning',
        executable='planner',
        name='planner',
        output='screen',
        parameters=[
            {'n_agents': 3},
            {'agent_id': 2},
            {'capacity': 3000}
        ]
    )
    
    communication_node = Node(package='communication', executable='driver', name='driver', output='screen', parameters=[{'port': '/dev/ttyACM0'}])
    ld.add_action(planning_node)
    ld.add_action(communication_node)
    return ld