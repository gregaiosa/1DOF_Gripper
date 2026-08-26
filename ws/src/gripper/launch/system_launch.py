import launch
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. Start the Foxglove Bridge node immediately
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            output='screen',
        ),

        # 2. Start the SO-101 Arm node immediately
        Node(
            package='gripper',
            executable='so101_arm_node',
            name='so101_arm_node',
            output='screen',
        ),

        # 3. Delay the startup of Phosphobot Bridge by 3 seconds.
        # This gives the arm node plenty of time to initialize its CAN 
        # connection and expose the /gripper/home service so that Phosphobot
        # calibration won't fail complaining about the service missing.
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='gripper',
                    executable='phosphobot_bridge',
                    name='phosphobot_bridge',
                    output='screen',
                )
            ]
        )
    ])
