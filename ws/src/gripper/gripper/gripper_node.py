#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import Float64
import can
import struct
import time
import threading
import math

# --- Constants from gl_control_mit_mode.py ---
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
        "pos": u2f((d[1] << 8) | d[2], P_MIN, P_MAX, 16),
        "spd": u2f((d[3] << 4) | (d[4] >> 4), V_MIN, V_MAX, 12),
        "torque": torque_val,
        "current": torque_val / KT,
    }

class GripperNode(Node):
    def __init__(self):
        super().__init__('gripper_node')

        # Declare parameters
        self.declare_parameter('node_id', 1)
        self.declare_parameter('can_channel', 'can0')
        self.declare_parameter('homing_speed', -2.0) # rad/s. Use negative to close jaws.
        self.declare_parameter('homing_current_threshold', 1.0) # Amps
        self.declare_parameter('open_position', math.pi) # 0.5 revs = pi radians
        self.declare_parameter('kp', 0.19)
        self.declare_parameter('kd', 0.01)
        
        self.node_id = self.get_parameter('node_id').value
        self.channel = self.get_parameter('can_channel').value
        self.kp = self.get_parameter('kp').value
        self.kd = self.get_parameter('kd').value
        
        try:
            self.bus = can.Bus(channel=self.channel, interface="socketcan")
            self.get_logger().info(f"Successfully opened CAN bus on {self.channel}")
        except Exception as e:
            self.get_logger().error(f"Failed to open CAN bus: {e}")
            raise e

        self.current_pos = 0.0
        self.current_vel = 0.0
        self.current_iq = 0.0
        self.current_torque = 0.0
        self.homed = False
        
        # State machine for the control loop
        self.state_lock = threading.Lock()
        self.control_mode = "IDLE" # "IDLE", "HOMING", "POSITION"
        self.target_pos = 0.0
        self.target_vel = 0.0
        
        # Enable the motor in MIT mode
        self._send_raw("mit", UNIVERSAL["enable"])
        time.sleep(0.1)

        # Start background threads
        self.running = True
        self.read_thread = threading.Thread(target=self._can_read_loop, daemon=True)
        self.read_thread.start()
        
        self.control_thread = threading.Thread(target=self._control_loop, daemon=True)
        self.control_thread.start()
        
        self.srv_home = self.create_service(Trigger, 'home_gripper', self.home_callback)
        self.srv_open = self.create_service(Trigger, 'open_gripper', self.open_callback)
        self.srv_close = self.create_service(Trigger, 'close_gripper', self.close_callback)
        
        # Telemetry publishers
        self.pub_pos = self.create_publisher(Float64, '~/position', 10)
        self.pub_vel = self.create_publisher(Float64, '~/velocity', 10)
        self.pub_cur = self.create_publisher(Float64, '~/current', 10)
        self.pub_trq = self.create_publisher(Float64, '~/torque', 10)
        
        # 50Hz telemetry timer
        self.telemetry_timer = self.create_timer(0.02, self.publish_telemetry)
        
        self.get_logger().info("Gripper node started.")
        
    def publish_telemetry(self):
        msg_pos = Float64(data=self.current_pos)
        msg_vel = Float64(data=self.current_vel)
        msg_cur = Float64(data=self.current_iq)
        msg_trq = Float64(data=self.current_torque)
        
        self.pub_pos.publish(msg_pos)
        self.pub_vel.publish(msg_vel)
        self.pub_cur.publish(msg_cur)
        self.pub_trq.publish(msg_trq)

    def _send_raw(self, mode, data):
        msg = can.Message(arbitration_id=arb_id(mode, self.node_id),
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
                    if fb and fb["canid"] == self.node_id:
                        self.current_pos = fb["pos"]
                        self.current_vel = fb["spd"]
                        self.current_iq = fb["current"]
                        self.current_torque = fb["torque"]
            except Exception:
                pass

    def _control_loop(self):
        # Run at ~500Hz for MIT mode
        loop_rate = 500.0
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
                
                cmd = pack_mit(pos=t_pos, vel=self.target_vel, kp=self.kp, kd=self.kd, tff=0.0)
                self._send_raw("mit", cmd)
            elif mode == "POSITION":
                # For position control, use full PD
                # Torque = kp * (t_pos - pos) + kd * (0 - vel)
                cmd = pack_mit(pos=t_pos, vel=0.0, kp=self.kp, kd=self.kd, tff=0.0)
                self._send_raw("mit", cmd)
            elif mode == "IDLE":
                # Send 0 torque / 0 gains just to keep connection alive if needed
                cmd = pack_mit(pos=0.0, vel=0.0, kp=0.0, kd=0.0, tff=0.0)
                self._send_raw("mit", cmd)

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
                # homing_speed is negative if closing jaws. To back off, move in opposite direction
                direction = 1 if homing_speed < 0 else -1
                
                with self.state_lock:
                    self.target_pos = self.current_pos + (direction * backoff_dist)
                    self.control_mode = "POSITION"
                
                # Give it a short time to complete the backoff movement
                time.sleep(0.5)
        
        # Calculate the average position
        avg_pos = sum(recorded_positions) / num_attempts
        self.get_logger().info(f"Homing complete. Average hard stop pos: {avg_pos:.3f} rad. Moving to average pos to zero.")
        
        # Move to the exact average position
        with self.state_lock:
            self.target_pos = avg_pos
            self.control_mode = "POSITION"
        
        # Wait a moment for it to settle exactly at the average position
        time.sleep(0.5)
        
        with self.state_lock:
            self.control_mode = "IDLE"
        
        time.sleep(0.1)
        # Zero the position on the motor controller at this average spot
        self._send_raw("mit", UNIVERSAL["zero"])
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
        self.running = False
        self._send_raw("mit", UNIVERSAL["disable"])
        self.bus.shutdown()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = GripperNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
