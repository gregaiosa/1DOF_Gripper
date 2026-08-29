import launch
from launch import LaunchDescription
from launch.actions import TimerAction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    arm_port_arg = DeclareLaunchArgument(
        'arm_port',
        default_value='/dev/ttyACM0',
        description='Serial port for the Follower SO-101 Arm'
    )

    return LaunchDescription([
        arm_port_arg,
        
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
            parameters=[{'arm_port': LaunchConfiguration('arm_port')}]
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
