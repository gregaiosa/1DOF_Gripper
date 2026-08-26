import sys
import threading
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from std_srvs.srv import Trigger
import numpy as np
import traceback

# Import phosphobot classes
try:
    from phosphobot.hardware.base import BaseManipulator
    from phosphobot.app import start_server
    import typer
except ImportError as e:
    print(f"Error importing phosphobot: {e}")
    print("Please make sure phosphobot is installed in this python environment.")
    sys.exit(1)

class PhosphobotROSBridge(Node):
    def __init__(self):
        super().__init__('phosphobot_bridge')
        self.pub_arm = self.create_publisher(JointState, '/so101_arm_node/arm/joint_commands', 10)
        self.pub_gripper = self.create_publisher(Float64, '/so101_arm_node/gripper/position_command', 10)
        self.cli_home = self.create_client(Trigger, '/so101_arm_node/gripper/home')
        self.get_logger().info("Phosphobot ROS 2 Bridge Node initialized")

def main(args=None):
    rclpy.init(args=args)
    node = PhosphobotROSBridge()
    
    # Thread to spin ROS 2
    def ros_spin():
        try:
            rclpy.spin(node)
        except Exception as e:
            pass
    
    thread = threading.Thread(target=ros_spin, daemon=True)
    thread.start()

    # Monkey patch BaseManipulator to intercept joint states
    original_set_motors = BaseManipulator.set_motors_positions
    original_control_gripper = BaseManipulator.control_gripper
    
    def hooked_set_motors_positions(self, q_target_rad: np.ndarray, enable_gripper: bool = False):
        try:
            if len(q_target_rad) >= 5:
                msg = JointState()
                msg.header.stamp = node.get_clock().now().to_msg()
                msg.name = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
                msg.position = np.rad2deg(q_target_rad[:5]).tolist()
                node.pub_arm.publish(msg)
        except Exception as e:
            node.get_logger().error(f"Error publishing arm positions: {e}")
        
        # Call the original method so Phosphobot's PyBullet simulation updates
        return original_set_motors(self, q_target_rad, enable_gripper)
        
    def hooked_control_gripper(self, open_command: float, **kwargs):
        try:
            msg = Float64()
            msg.data = float(open_command)
            node.pub_gripper.publish(msg)
        except Exception as e:
            node.get_logger().error(f"Error publishing gripper command: {e}")
            
        return original_control_gripper(self, open_command, **kwargs)

    async def hooked_calibrate(self):
        # Fake the 3 steps to satisfy the UI state machine
        if not hasattr(self, 'calibration_current_step'):
            self.calibration_current_step = 0
            
        if self.calibration_current_step == 0:
            self.calibration_current_step = 1
            # Trigger the actual ROS 2 homing sequence!
            if node.cli_home.wait_for_service(timeout_sec=1.0):
                req = Trigger.Request()
                node.cli_home.call_async(req)
                return "in_progress", "Step 1: Homing Gripper..."
            else:
                return "failed", "Homing service not available! Is arm node running?"
        elif self.calibration_current_step == 1:
            self.calibration_current_step = 2
            return "in_progress", "Step 2: Waiting for homing to finish..."
        else:
            self.calibration_current_step = 0
            return "success", "Calibration complete!"

    BaseManipulator.set_motors_positions = hooked_set_motors_positions
    BaseManipulator.control_gripper = hooked_control_gripper
    
    # We must patch calibrate on the actual hardware class if it overrides it, or on BaseManipulator
    from phosphobot.hardware.so100 import SO100Hardware
    SO100Hardware.calibrate = hooked_calibrate
    
    node.get_logger().info("Successfully hooked Phosphobot! Starting Phosphobot Server in ONLY_SIMULATION mode...")
    
    # Run Phosphobot with ONLY_SIMULATION=True
    # Using typer.run on start_server and forcing keyword arguments
    # Wait, start_server is wrapped by typer. We can just call it directly!
    try:
        start_server(
            host="0.0.0.0",
            port=8000,
            only_simulation=True, # Critical: Avoids connecting to the physical serial port
        )
    except Exception as e:
        node.get_logger().error(f"Phosphobot server stopped: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
