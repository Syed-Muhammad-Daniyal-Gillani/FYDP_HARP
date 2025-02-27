import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os
def generate_launch_description():
    return LaunchDescription([

        Node(
            package='vision_module',
            executable='face_tracker',
            name='Vision',
            output = 'screen'
        ),

        Node(
            package='face_animations',
            executable='face_animations',
            name='Eyes_Display',
            output = 'screen'
        ),
        Node(
            package='neck_module',
            executable='neck_controller',
            name='Neck',
            output = 'screen'
        ),
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='ROSBridge_Local_Server',
            output = 'screen'            
        )
        # # Rosbridge WebSocket (Runs Outside Venv)
        # ExecuteProcess(
        #     cmd=['/usr/bin/python3', '-m', 'rosbridge_server.rosbridge_websocket'],
        #     output='screen',
        #     env={
        #         'PYTHONPATH': '/opt/ros/humble/lib/python3.10/site-packages:' + os.environ.get('PYTHONPATH', ''),
        #         'PATH': '/usr/bin:' + os.environ.get('PATH', ''),
        #     }
        # )

    ])