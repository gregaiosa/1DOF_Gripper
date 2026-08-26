#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
import can
import struct
import time
import threading
import math
import sys
import os

# ---------------------------------------------------------
# IMPORT LEROBOT (SO-101 ARM)
# ---------------------------------------------------------
# Dynamically add the lerobot src directory so we can import it
lerobot_src = os.path.expanduser("~/Final_Project/LeRobot/lerobot/src")
if lerobot_src not in sys.path:
    sys.path.append(lerobot_src)

try:
    from lerobot.robots.so_follower.so_follower import SOFollower
    from lerobot.robots.so_follower.config_so_follower import SOFollowerRobotConfig
    LEROBOT_AVAILABLE = True
except ImportError as e:
    print(f"WARNING: Could not import lerobot modules: {e}. SO-101 arm functionality will be disabled.")
    LEROBOT_AVAILABLE = False

# ---------------------------------------------------------
# CONSTANTS FOR GL40 II (GRIPPER)
# ---------------------------------------------------------
MODE_PREFIX = {"mit": 0x0, "pv": 0x1, "vel": 0x2}
P_MIN, P_MAX = -12.5, 12.5
V_MIN, V_MAX = -200.0, 200.0
T_MIN, T_MAX = -10.0, 10.0

MIT_P_MIN, MIT_P_MAX = -12.5, 12.5
MIT_V_MIN, MIT_V_MAX = -65.0, 65.0
MIT_KP_MIN, MIT_KP_MAX = 0.0, 500.0
MIT_KD_MIN, MIT_KD_MAX = 0.0, 5.0
MIT_T_MIN, MIT_T_MAX = -10.0, 10.0
KT = 0.10

ERR_NAMES = {
    0: "Disabled", 1: "Enabled", 8: "Over-voltage", 9: "Under-voltage",
    0xA: "Over-current", 0xB: "MOS over-temp", 0xC: "Winding over-temp",
    0xD: "Comm loss", 0xE: "Overload",
}

UNIVERSAL = {
    "enable":  bytes([0xFF]*7 + [0xFC]),
    "disable": bytes([0xFF]*7 + [0xFD]),
    "zero":    bytes([0xFF]*7 + [0xFE]),
    "clear":   bytes([0xFF]*7 + [0xFB]),
}

def arb_id(mode, node):
    return (MODE_PREFIX[mode] << 8) | (node & 0xFF)

def f2u(x, lo, hi, bits):
    x = max(min(x, hi), lo)
    return int((x - lo) * ((1 << bits) - 1) / (hi - lo))

def u2f(x, lo, hi, bits):
    return x * (hi - lo) / ((1 << bits) - 1) + lo

def pack_mit(pos, vel, kp, kd, tff):
    p = f2u(pos, MIT_P_MIN, MIT_P_MAX, 16)
    v = f2u(vel, MIT_V_MIN, MIT_V_MAX, 12)
    kpi = f2u(kp, MIT_KP_MIN, MIT_KP_MAX, 12)
    kdi = f2u(kd, MIT_KD_MIN, MIT_KD_MAX, 12)
    ti = f2u(tff, MIT_T_MIN, MIT_T_MAX, 12)
    return bytes([
        (p >> 8) & 0xFF, p & 0xFF,
        (v >> 4) & 0xFF,
        ((v & 0xF) << 4) | ((kpi >> 8) & 0xF),
        kpi & 0xFF,
        (kdi >> 4) & 0xFF,
        ((kdi & 0xF) << 4) | ((ti >> 8) & 0xF),
        ti & 0xFF,
    ])

def decode(d):
    if len(d) < 8:
        return None
    torque_val = u2f(((d[4] & 0xF) << 8) | d[5], T_MIN, T_MAX, 12)
    return {
        "canid": d[0] & 0xF,
        "err": d[0] >> 4,
        "err_name": ERR_NAMES.get(d[0] >> 4, f"0x{d[0] >> 4:X}"),
        "pos": u2f((d[1] << 8) | d[2], P_MIN, P_MAX, 16),
        "spd": u2f((d[3] << 4) | (d[4] >> 4), V_MIN, V_MAX, 12),
        "torque": torque_val,
        "current": torque_val / KT,
    }

class SO101ArmNode(Node):
    def __init__(self):
        super().__init__('so101_arm_node')

        # --- Parameters ---
        # Arm
        self.declare_parameter('arm_port', '/dev/ttyACM0')
        # Gripper
        self.declare_parameter('node_id', 3)
        self.declare_parameter('leader_id', 1)
        self.declare_parameter('can_channel', 'can0')
        self.declare_parameter('homing_speed', 2.0) # rad/s. Use positive to close jaws now.
        self.declare_parameter('homing_current_threshold', 2.5) # Amps
        self.declare_parameter('open_position', 0.67) # radians
        self.declare_parameter('kp', 0.16)
        self.declare_parameter('kd', 0.006)
        self.declare_parameter('friction_comp', 0.09) # Nm of Coulomb friction to compensate
        self.declare_parameter('haptic_gain', 1.0)
        self.declare_parameter('enable_bilateral', True)
        
        # Extract params
        arm_port = self.get_parameter('arm_port').value
        self.node_id = self.get_parameter('node_id').value
        self.leader_id = self.get_parameter('leader_id').value
        self.channel = self.get_parameter('can_channel').value
        self.kp = self.get_parameter('kp').value
        self.kd = self.get_parameter('kd').value
        self.haptic_gain = self.get_parameter('haptic_gain').value
        self.enable_bilateral = self.get_parameter('enable_bilateral').value
        
        # ---------------------------------------------------------
        # INITIALIZE SO-101 ARM (FEETECH)
        # ---------------------------------------------------------
        self.get_logger().info(f"Connecting to SO-101 arm on {arm_port}...")
        if not LEROBOT_AVAILABLE:
            self.get_logger().error("lerobot dependencies (like draccus) are missing. Running without arm.")
            self.arm = None
        else:
            try:
                config = SOFollowerRobotConfig(port=arm_port)
                # Prevent attempting to calibrate missing gripper by patching cameras (if needed)
                config.cameras = {} 
                self.arm = SOFollower(config)
                
                # The gripper is now the GL40 II on CAN, so remove it from the Feetech bus
                # Since FeetechMotorsBus caches properties like 'ids', we must completely recreate it
                from lerobot.motors.feetech import FeetechMotorsBus
                new_motors = {name: m for name, m in self.arm.bus.motors.items() if name != "gripper"}
                self.arm.bus = FeetechMotorsBus(
                    port=self.arm.bus.port,
                    motors=new_motors,
                    calibration=self.arm.bus.calibration,
                    protocol_version=self.arm.bus.protocol_version
                )
                    
                self.arm.connect()
                self.get_logger().info("SO-101 Arm connected successfully.")
            except Exception as e:
                self.get_logger().error(f"Failed to connect to SO-101 Arm: {e}. Running without arm.")
                self.arm = None

        self.arm_state_lock = threading.Lock()
        self.arm_target_action = None

        # ---------------------------------------------------------
        # INITIALIZE GL40 II (CAN)
        # ---------------------------------------------------------
        try:
            self.bus = can.Bus(channel=self.channel, interface="socketcan")
            self.get_logger().info(f"Successfully opened CAN bus on {self.channel}")
        except Exception as e:
            self.get_logger().error(f"Failed to open CAN bus: {e}")
            raise e

        # Follower (GL40 II) State
        self.current_pos = 0.0
        self.current_vel = 0.0
        self.current_iq = 0.0
        self.current_torque = 0.0
        
        # Leader (GL60 II) State
        self.leader_pos = 0.0
        self.leader_vel = 0.0
        self.leader_iq = 0.0
        self.leader_torque = 0.0
        
        self.homed = False
        
        # State machine for the control loop
        self.state_lock = threading.Lock()
        self.control_mode = "IDLE" # "IDLE", "HOMING", "POSITION"
        self.target_pos = 0.0
        self.target_vel = 0.0
        
        # Enable both motors in MIT mode
        self._send_raw("mit", self.node_id, UNIVERSAL["enable"])
        self._send_raw("mit", self.leader_id, UNIVERSAL["enable"])
        time.sleep(0.1)
        
        # Zero the leader immediately upon startup (assumes no hard stops)
        self._send_raw("mit", self.leader_id, UNIVERSAL["zero"])
        time.sleep(0.1)

        # ---------------------------------------------------------
        # ROS 2 INTERFACES & THREADS
        # ---------------------------------------------------------
        self.running = True
        
        # Services
        self.srv_home = self.create_service(Trigger, '~/gripper/home', self.home_callback)
        self.srv_open = self.create_service(Trigger, '~/gripper/open', self.open_callback)
        self.srv_close = self.create_service(Trigger, '~/gripper/close', self.close_callback)
        
        # Publishers
        self.pub_arm_state = self.create_publisher(JointState, '~/arm/joint_states', 10)
        self.pub_pos = self.create_publisher(Float64, '~/gripper/position', 10)
        self.pub_vel = self.create_publisher(Float64, '~/gripper/velocity', 10)
        self.pub_cur = self.create_publisher(Float64, '~/gripper/current', 10)
        self.pub_trq = self.create_publisher(Float64, '~/gripper/torque', 10)
        
        # Subscribers
        self.sub_arm_cmd = self.create_subscription(JointState, '~/arm/joint_commands', self.arm_cmd_callback, 10)
        self.sub_gripper_cmd = self.create_subscription(Float64, '~/gripper/position_command', self.gripper_cmd_callback, 10)

        # Gripper CAN Threads
        self.read_thread = threading.Thread(target=self._can_read_loop, daemon=True)
        self.read_thread.start()
        self.control_thread = threading.Thread(target=self._can_control_loop, daemon=True)
        self.control_thread.start()
        
        # Arm Serial Thread
        self.arm_thread = threading.Thread(target=self._arm_loop, daemon=True)
        self.arm_thread.start()
        
        # 50Hz telemetry timer for Gripper
        self.telemetry_timer = self.create_timer(0.02, self.publish_telemetry)
        
        self.get_logger().info("SO101 Arm + Gripper node started.")
        
    def publish_telemetry(self):
        msg_pos = Float64(data=self.current_pos)
        msg_vel = Float64(data=self.current_vel)
        msg_cur = Float64(data=self.current_iq)
        msg_trq = Float64(data=self.current_torque)
        
        self.pub_pos.publish(msg_pos)
        self.pub_vel.publish(msg_vel)
        self.pub_cur.publish(msg_cur)
        self.pub_trq.publish(msg_trq)

    # ---------------------------------------------------------
    # ARM LOGIC
    # ---------------------------------------------------------
    def arm_cmd_callback(self, msg: JointState):
        if self.arm is None:
            return
        action = {}
        for name, pos in zip(msg.name, msg.position):
            if name in self.arm.bus.motors:
                action[f"{name}.pos"] = pos
        if action:
            with self.arm_state_lock:
                self.arm_target_action = action

    def _arm_loop(self):
        if self.arm is None:
            return
        loop_rate = 30.0 # ~30Hz is standard for serial Feetech reads
        dt = 1.0 / loop_rate
        while self.running:
            t_start = time.time()
            try:
                # Read from SO-101
                obs = self.arm.get_observation()
                
                # Publish JointState
                msg = JointState()
                msg.header.stamp = self.get_clock().now().to_msg()
                for motor_name in self.arm.bus.motors.keys():
                    pos_key = f"{motor_name}.pos"
                    if pos_key in obs:
                        msg.name.append(motor_name)
                        msg.position.append(obs[pos_key])
                self.pub_arm_state.publish(msg)

                # Send commands to SO-101
                target_action = None
                with self.arm_state_lock:
                    target_action = self.arm_target_action
                
                if target_action:
                    self.arm.send_action(target_action)

            except Exception as e:
                self.get_logger().error(f"Arm loop error: {e}", throttle_duration_sec=2.0)

            elapsed = time.time() - t_start
            if elapsed < dt:
                time.sleep(dt - elapsed)

    # ---------------------------------------------------------
    # GRIPPER CAN LOGIC
    # ---------------------------------------------------------
    def gripper_cmd_callback(self, msg: Float64):
        if not self.homed:
            self.get_logger().warn("Gripper not homed. Ignoring command.")
            return
        
        # Calculate the physical open position (same logic as open_callback)
        open_val = self.get_parameter('open_position').value
        homing_speed = self.get_parameter('homing_speed').value
        if homing_speed < 0:
            open_target = abs(open_val)
        else:
            open_target = -abs(open_val)
            
        # Map 0.0 to 1.0 (from Phosphobot) into the physical workspace
        # 0.0 -> 0.0 (Closed), 1.0 -> open_target (Open)
        cmd_clamped = max(0.0, min(1.0, float(msg.data)))
        mapped_pos = cmd_clamped * open_target
        
        with self.state_lock:
            self.target_pos = mapped_pos
            self.control_mode = "POSITION"

    def _send_raw(self, mode, node_id, data):
        msg = can.Message(arbitration_id=arb_id(mode, node_id),
                          data=data, is_extended_id=False)
        try:
            self.bus.send(msg)
        except Exception:
            pass

    def _can_read_loop(self):
        while self.running:
            try:
                msg = self.bus.recv(timeout=0.1)
                if msg:
                    fb = decode(msg.data)
                    if fb:
                        if fb["canid"] == self.node_id:
                            self.current_pos = fb["pos"]
                            self.current_vel = fb["spd"]
                            self.current_iq = fb["current"]
                            self.current_torque = fb["torque"]
                            if fb["err"] > 1:
                                self.get_logger().error(f"Follower Motor Fault: {fb['err_name']}", throttle_duration_sec=1.0)
                        elif fb["canid"] == self.leader_id:
                            self.leader_pos = fb["pos"]
                            self.leader_vel = fb["spd"]
                            self.leader_iq = fb["current"]
                            self.leader_torque = fb["torque"]
                            if fb["err"] > 1:
                                self.get_logger().error(f"Leader Motor Fault: {fb['err_name']}", throttle_duration_sec=1.0)
            except Exception:
                pass

    def _can_control_loop(self):
        # Run at ~1000Hz for MIT mode
        loop_rate = 1000.0
        dt = 1.0 / loop_rate
        
        while self.running:
            t_start = time.time()
            
            with self.state_lock:
                mode = self.control_mode
                t_pos = self.target_pos
                t_vel = self.target_vel
            
            if mode == "HOMING":
                # For homing, generate a trajectory by moving target_pos at homing_speed
                t_pos += self.target_vel * dt
                self.target_pos = t_pos # Update shared state
                
                # Apply friction compensation in the direction of homing velocity
                comp_torque = math.copysign(self.get_parameter('friction_comp').value, self.target_vel)
                
                # Use the user-defined kp and kd for homing as requested.
                cmd = pack_mit(pos=t_pos, vel=0.0, kp=self.kp, kd=self.kd, tff=comp_torque)
                self._send_raw("mit", self.node_id, cmd)
                
                # Leader holds zero torque during follower homing
                leader_cmd = pack_mit(pos=0.0, vel=0.0, kp=0.0, kd=0.0, tff=0.0)
                self._send_raw("mit", self.leader_id, leader_cmd)
                
            elif mode == "POSITION":
                if self.enable_bilateral:
                    # Bilateral mode: Follower tracks Leader (1:1), Leader feels Follower's torque
                    t_pos = self.leader_pos
                    
                    # Follower Command
                    error = t_pos - self.current_pos
                    friction_comp = self.get_parameter('friction_comp').value
                    taper_scale = min(1.0, abs(error) / 0.1)
                    comp_torque = math.copysign(friction_comp * taper_scale, error)
                    
                    cmd = pack_mit(pos=t_pos, vel=0.0, kp=self.kp, kd=self.kd, tff=comp_torque)
                    self._send_raw("mit", self.node_id, cmd)
                    
                    # Leader Command (Force Feedback)
                    # We negate the follower's torque so it pushes back against the user
                    force_feedback = -self.current_torque * self.haptic_gain
                    
                    # Add a tiny bit of kd damping (0.01) to the leader to make it feel smooth
                    leader_cmd = pack_mit(pos=0.0, vel=0.0, kp=0.0, kd=0.01, tff=force_feedback)
                    self._send_raw("mit", self.leader_id, leader_cmd)
                else:
                    # Normal position control (e.g. from Phosphobot topic)
                    error = t_pos - self.current_pos
                    comp_torque = 0.0
                    
                    # Apply Coulomb friction compensation, but taper it linearly as it gets very close to the target 
                    # (within 0.1 rad) to prevent violent bang-bang limit cycle oscillations
                    friction_comp = self.get_parameter('friction_comp').value
                    taper_scale = min(1.0, abs(error) / 0.1)
                    comp_torque = math.copysign(friction_comp * taper_scale, error)
                        
                    cmd = pack_mit(pos=t_pos, vel=0.0, kp=self.kp, kd=self.kd, tff=comp_torque)
                    self._send_raw("mit", self.node_id, cmd)
                    
                    # Leader holds zero torque
                    leader_cmd = pack_mit(pos=0.0, vel=0.0, kp=0.0, kd=0.0, tff=0.0)
                    self._send_raw("mit", self.leader_id, leader_cmd)
                    
            elif mode == "IDLE":
                # Send 0 torque / 0 gains just to keep connection alive if needed
                cmd = pack_mit(pos=0.0, vel=0.0, kp=0.0, kd=0.0, tff=0.0)
                self._send_raw("mit", self.node_id, cmd)
                self._send_raw("mit", self.leader_id, cmd)

            elapsed = time.time() - t_start
            if elapsed < dt:
                time.sleep(dt - elapsed)
        
    def home_callback(self, request, response):
        self.get_logger().info("Homing gripper...")
        homing_speed = self.get_parameter('homing_speed').value
        current_thresh = self.get_parameter('homing_current_threshold').value

        num_attempts = 3
        recorded_positions = []
        
        for attempt in range(num_attempts):
            with self.state_lock:
                self.target_pos = self.current_pos # Start ramping from current position
                self.control_mode = "HOMING"
                self.target_vel = homing_speed
            
            # Wait for hard stop (current spike)
            spike_count = 0
            timeout = time.time() + 10.0 # 10 seconds max timeout
            last_log_time = time.time()
            hit_stop = False
            
            while time.time() < timeout:
                current = abs(self.current_iq)
                if time.time() - last_log_time > 1.0:
                    self.get_logger().info(f"Homing attempt {attempt+1}... Current IQ: {current:.2f} A, Target Pos: {self.target_pos:.2f} rad")
                    last_log_time = time.time()
                    
                if current > current_thresh:
                    spike_count += 1
                else:
                    spike_count = 0
                    
                if spike_count >= 3:
                    hit_stop = True
                    break
                time.sleep(0.01)
                
            if not hit_stop:
                with self.state_lock:
                    self.control_mode = "IDLE"
                response.success = False
                response.message = f"Homing failed on attempt {attempt+1}: Hard stop not detected."
                self.get_logger().error(response.message)
                return response
                
            # Hard stop hit
            hard_stop_pos = self.current_pos
            recorded_positions.append(hard_stop_pos)
            self.get_logger().info(f"Attempt {attempt+1} hard stop hit at current: {self.current_iq:.2f} A, Pos: {hard_stop_pos:.3f} rad")
            
            if attempt < num_attempts - 1:
                # Back off slightly before the next attempt
                backoff_dist = 0.3 # rad
                direction = 1 if homing_speed < 0 else -1
                
                with self.state_lock:
                    self.target_pos = self.current_pos + (direction * backoff_dist)
                    self.control_mode = "POSITION"
                
                time.sleep(0.5)
        
        # Calculate the average position
        avg_pos = sum(recorded_positions) / num_attempts
        self.get_logger().info(f"Homing complete. Average hard stop pos: {avg_pos:.3f} rad. Moving to average pos to zero.")
        
        # Move to the exact average position
        with self.state_lock:
            self.target_pos = avg_pos
            self.control_mode = "POSITION"
        
        time.sleep(0.5)
        
        with self.state_lock:
            self.control_mode = "IDLE"
        
        time.sleep(0.1)
        self._send_raw("mit", self.node_id, UNIVERSAL["zero"])
        time.sleep(0.1)
        
        with self.state_lock:
            self.target_pos = 0.0
            self.control_mode = "POSITION"
        
        self.homed = True
        response.success = True
        response.message = "Gripper homed successfully with averaged positions."
        return response

    def open_callback(self, request, response):
        if not self.homed:
            response.success = False
            response.message = "Gripper must be homed before opening."
            self.get_logger().warn(response.message)
            return response
            
        target_pos = self.get_parameter('open_position').value
        homing_speed = self.get_parameter('homing_speed').value
        if homing_speed < 0:
            target_pos = abs(target_pos)
        else:
            target_pos = -abs(target_pos)

        self.get_logger().info(f"Opening gripper to {target_pos:.2f} rad")
        
        with self.state_lock:
            self.target_pos = target_pos
            self.control_mode = "POSITION"
        
        response.success = True
        response.message = "Gripper opened."
        return response

    def close_callback(self, request, response):
        if not self.homed:
            response.success = False
            response.message = "Gripper must be homed before closing."
            self.get_logger().warn(response.message)
            return response
            
        self.get_logger().info("Closing gripper...")
        
        with self.state_lock:
            self.target_pos = 0.0
            self.control_mode = "POSITION"
        
        response.success = True
        response.message = "Gripper closed."
        return response
        
    def destroy_node(self):
        self.get_logger().info("Shutting down... disabling motors.")
        self.running = False
        
        # Stop threads
        try:
            self.read_thread.join(timeout=1.0)
            self.control_thread.join(timeout=1.0)
            self.arm_thread.join(timeout=1.0)
        except Exception:
            pass

        # Disable gripper and leader (send 0 torque explicitly first, then disable)
        try:
            cmd = pack_mit(pos=0.0, vel=0.0, kp=0.0, kd=0.0, tff=0.0)
            for _ in range(3):
                self._send_raw("mit", self.node_id, cmd)
                self._send_raw("mit", self.leader_id, cmd)
                time.sleep(0.01)
            for _ in range(3):
                self._send_raw("mit", self.node_id, UNIVERSAL["disable"])
                self._send_raw("mit", self.leader_id, UNIVERSAL["disable"])
                time.sleep(0.01)
            self.bus.shutdown()
        except Exception:
            pass

        # Disable arm
        try:
            if self.arm is not None:
                self.arm.disconnect()
        except Exception:
            pass
            
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SO101ArmNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
