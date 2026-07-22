#!/usr/bin/env python3
"""
gl_control_tune.py  --  CubeMars GL40 II KV82.5 control + feedback over SocketCAN.
Supports multiple motors on one bus (daisy-chained). Verified MIT/PV/Vel packing
and §5.4 feedback decode.

Includes post-run matplotlib graphing and step-response metrics for tuning gains.
"""

import argparse
import struct
import sys
import time

import can
import numpy as np
import matplotlib.pyplot as plt

CHANNEL = "can0"
DEFAULT_NODES = [1, 3]

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

LOOP_HZ = 500   

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

def pack_pv(pos, vel):
    return struct.pack("<ff", float(pos), float(vel))

def pack_vel(vel):
    return struct.pack("<f", float(vel))

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
        "err": d[0] >> 4,
        "err_name": ERR_NAMES.get(d[0] >> 4, f"0x{d[0] >> 4:X}"),
        "canid": d[0] & 0xF,
        "pos": u2f((d[1] << 8) | d[2], P_MIN, P_MAX, 16),
        "spd": u2f((d[3] << 4) | (d[4] >> 4), V_MIN, V_MAX, 12),
        "torque": torque_val,
        "current": torque_val / KT,
        "t_drive": d[6] if d[6] < 128 else d[6] - 256,
        "t_motor": d[7] if d[7] < 128 else d[7] - 256,
    }

def send(bus, mode, node, data, label=""):
    bus.send(can.Message(arbitration_id=arb_id(mode, node),
                         data=data, is_extended_id=False))
    if label:
        print(f"[tx] node {node} {label}: {arb_id(mode, node):03X}#{data.hex().upper()}")

def stream(bus, seconds, nodes):
    t_end = time.time() + seconds
    last = {}
    while time.time() < t_end:
        msg = bus.recv(timeout=0.1)
        if msg is None:
            continue
        fb = decode(msg.data)
        if fb is None or fb["canid"] not in nodes:
            continue
        n = fb["canid"]
        now = time.time()
        if now - last.get(n, 0) > 0.15: 
            print(f"  [M{n}] pos={fb['pos']:+6.3f} rad  spd={fb['spd']:+7.2f} r/s  "
                  f"torque={fb['torque']:+6.3f} Nm  current~={fb['current']:+6.2f} A  "
                  f"Tdrv={fb['t_drive']}C Tmot={fb['t_motor']}C  [{fb['err_name']}]")
            last[n] = now

def build_cmd(mode, pos, vel, kp, kd, tff):
    if mode == "pv":
        return pack_pv(pos, vel)
    if mode == "vel":
        return pack_vel(vel)
    return pack_mit(pos, vel, kp, kd, tff)

def _run_mit_loop(bus, nodes, pos_targets, vel, kp, kd, tff,
                  hold, ramp, current_positions, history):
    dt       = 1.0 / LOOP_HZ
    t_start  = time.time()
    t_end    = t_start + hold
    last_print: dict = {}

    while time.time() < t_end:
        t_loop  = time.time()
        elapsed = t_loop - t_start
        alpha   = min(elapsed / ramp, 1.0) if ramp > 0 else 1.0

        for n in nodes:
            p0    = current_positions.get(n, 0.0)
            p_cmd = p0 + alpha * (pos_targets.get(n, p0) - p0)
            bus.send(can.Message(
                arbitration_id=arb_id("mit", n),
                data=pack_mit(p_cmd, vel, kp, kd, tff),
                is_extended_id=False,
            ))

        msg = bus.recv(timeout=0)
        if msg:
            fb = decode(msg.data)
            if fb and fb["canid"] in nodes:
                n   = fb["canid"]
                now = time.time()
                elapsed_fb = now - t_start
                
                # --- RECORD TELEMETRY DATA ---
                history[n]['t'].append(elapsed_fb)
                history[n]['pos'].append(fb['pos'])
                history[n]['spd'].append(fb['spd'])
                history[n]['torque'].append(fb['torque'])
                
                p0 = current_positions.get(n, 0.0)
                alpha_now = min(elapsed_fb / ramp, 1.0) if ramp > 0 else 1.0
                history[n]['cmd'].append(p0 + alpha_now * (pos_targets.get(n, p0) - p0))

                if now - last_print.get(n, 0) > 0.2:
                    alpha_pct = min(elapsed / ramp * 100, 100) if ramp > 0 else 100
                    print(f"  [M{n}] pos={fb['pos']:+6.3f} rad  spd={fb['spd']:+7.2f} r/s  "
                          f"torque={fb['torque']:+6.3f} Nm  current~={fb['current']:+6.2f} A  "
                          f"ramp={alpha_pct:.0f}%  [{fb['err_name']}]")
                    last_print[n] = now

        remaining = dt - (time.time() - t_loop)
        if remaining > 0:
            time.sleep(remaining)

def calculate_metrics(t_list, y_list, y0, y_ref):
    """Calculate step response characteristics."""
    if len(t_list) < 2: return {}
    t = np.array(t_list)
    y = np.array(y_list)
    
    step_size = y_ref - y0
    if abs(step_size) < 1e-3:
        return {"Note": "Step size too small for reliable metrics."}
        
    metrics = {}
    
    # Overshoot/Undershoot
    if step_size > 0:
        peak = np.max(y)
        overshoot = max(0.0, (peak - y_ref) / step_size * 100.0)
    else:
        peak = np.min(y)
        overshoot = max(0.0, (peak - y_ref) / step_size * 100.0)
    metrics["Overshoot (%)"] = overshoot
    
    # Rise time (10% to 90%)
    y_10 = y0 + 0.10 * step_size
    y_90 = y0 + 0.90 * step_size
    try:
        if step_size > 0:
            t_10 = t[np.where(y >= y_10)[0][0]]
            t_90 = t[np.where(y >= y_90)[0][0]]
        else:
            t_10 = t[np.where(y <= y_10)[0][0]]
            t_90 = t[np.where(y <= y_90)[0][0]]
        metrics["Rise Time (s)"] = t_90 - t_10
    except IndexError:
        metrics["Rise Time (s)"] = None # Never reached 90%
        
    # Settling time (stays within 2% of final target)
    margin = 0.02 * abs(step_size)
    upper_bound = y_ref + margin
    lower_bound = y_ref - margin
    out_of_bounds = np.where((y > upper_bound) | (y < lower_bound))[0]
    if len(out_of_bounds) > 0:
        settle_idx = out_of_bounds[-1]
        if settle_idx == len(y) - 1:
            metrics["Settling Time (s)"] = None # Didn't settle before timeout
        else:
            metrics["Settling Time (s)"] = t[settle_idx]
    else:
        metrics["Settling Time (s)"] = 0.0

    # Steady State Error (Mean of last 10% of samples)
    n_last = max(1, len(y) // 10)
    sse = np.mean(y[-n_last:]) - y_ref
    metrics["Steady-State Error (rad)"] = sse

    return metrics

def plot_responses(history, pos_targets, current_positions, ramp):
    """Plot the stored history data and print metrics after the run."""
    print("\n--- Tuning Metrics ---")
    if ramp > 0:
        print(f"WARNING: Metrics are skewed because ramp ({ramp}s) is > 0. For true step response tuning, use --ramp 0")
        
    fig, axes = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
    fig.canvas.manager.set_window_title('Motor Response Tuning')

    for n, data in history.items():
        if not data['t']:
            continue
            
        t = data['t']
        y_ref = pos_targets.get(n, 0.0)
        y0 = current_positions.get(n, 0.0)
        
        # Calculate and print metrics
        metrics = calculate_metrics(t, data['pos'], y0, y_ref)
        print(f"\nMotor {n} -> Target: {y_ref:.3f} rad, Initial: {y0:.3f} rad")
        for k, v in metrics.items():
            if v is None:
                print(f"  {k}: Did not reach")
            elif isinstance(v, float):
                print(f"  {k}: {v:.4f}")
            else:
                print(f"  {k}: {v}")

        # Position Plot
        axes[0].plot(t, data['pos'], label=f"M{n} Actual")
        axes[0].plot(t, data['cmd'], '--', label=f"M{n} Command")
        
        # Velocity Plot
        axes[1].plot(t, data['spd'], label=f"M{n} Velocity")

    axes[0].set_title('Position Tracking')
    axes[0].set_ylabel('Position (rad)')
    axes[0].grid(True)
    axes[0].legend()

    axes[1].set_title('Velocity Feedback')
    axes[1].set_xlabel('Time (s)')
    axes[1].set_ylabel('Velocity (rad/s)')
    axes[1].grid(True)
    axes[1].legend()

    plt.tight_layout()
    print("\nDisplaying plot... (Close window to exit completely)")
    plt.show()

def parse_pos2(s):
    out = {}
    for part in s.split(","):
        k, v = part.split(":")
        out[int(k)] = float(v)
    return out

def main():
    ap = argparse.ArgumentParser(description="GL40 II CAN control + feedback + tuning graph")
    ap.add_argument("--mode", choices=["pv", "vel", "mit"], required=True)
    ap.add_argument("--node", type=lambda s: int(s, 0), default=1,
                    help="single motor CAN ID (default 1)")
    ap.add_argument("--both", action="store_true",
                    help="command both motors (default IDs 1 and 3)")
    ap.add_argument("--nodes", type=str, default=None,
                    help="comma list of node ids for --both, e.g. '1,3' (default 1,3)")
    ap.add_argument("--pos", type=float, default=0.0)
    ap.add_argument("--vel", type=float, default=0.0)
    ap.add_argument("--pos2", type=str, default=None,
                    help="per-motor positions for --both, e.g. '1:0.3,3:-0.3'")
    ap.add_argument("--kp", type=float, default=1.0)
    ap.add_argument("--kd", type=float, default=2.0)
    ap.add_argument("--ramp", type=float, default=1.0,
                    help="MIT: seconds to ramp from current pos to target (default 1.0). Set to 0 for step response.")
    ap.add_argument("--tff", type=float, default=0.0)
    ap.add_argument("--hold", type=float, default=3.0)
    ap.add_argument("--enable-only", action="store_true")
    ap.add_argument("--zero", action="store_true")
    args = ap.parse_args()

    if args.both:
        nodes = [int(x) for x in args.nodes.split(",")] if args.nodes else list(DEFAULT_NODES)
    else:
        nodes = [args.node]

    pos2 = parse_pos2(args.pos2) if args.pos2 else None
    
    # Store trajectory data for analysis
    history = {n: {'t': [], 'pos': [], 'cmd': [], 'spd': [], 'torque': []} for n in nodes}
    pos_targets = {}
    current_positions = {}

    try:
        bus = can.Bus(channel=CHANNEL, interface="socketcan")
    except Exception as e:
        print(f"Could not open {CHANNEL}: {e}")
        print("Bring it up:  sudo ip link set can0 up type can bitrate 1000000")
        sys.exit(1)

    try:
        for n in nodes:
            send(bus, args.mode, n, UNIVERSAL["enable"], "enable")
            time.sleep(0.03)
            deadline = time.time() + 0.3
            while time.time() < deadline:
                msg = bus.recv(timeout=0.05)
                if msg:
                    fb = decode(msg.data)
                    if fb and fb["canid"] == n:
                        current_positions[n] = fb["pos"]
                        print(f"  [M{n}] current pos = {fb['pos']:+.3f} rad  [{fb['err_name']}]")
                        break
            if n not in current_positions:
                current_positions[n] = 0.0
                print(f"  [M{n}] WARNING: no enable feedback, assuming pos=0.0")

        if args.zero:
            for n in nodes:
                send(bus, args.mode, n, UNIVERSAL["zero"], "set-zero")
                time.sleep(0.05)
            current_positions = {n: 0.0 for n in nodes}
            print("[zero] all positions reset to 0.0")

        if not args.enable_only:
            pos_targets = {
                n: (pos2[n] if (pos2 and n in pos2) else args.pos)
                for n in nodes
            }
            if args.mode == "mit":
                print(f"[MIT loop] {args.hold}s @ {LOOP_HZ}Hz  "
                      f"ramp={args.ramp}s  kp={args.kp}  kd={args.kd}  "
                      f"targets={pos_targets}  (Ctrl-C to stop):")
                _run_mit_loop(bus, nodes, pos_targets, args.vel,
                              args.kp, args.kd, args.tff,
                              args.hold, args.ramp, current_positions, history)
            else:
                for n in nodes:
                    data = build_cmd(args.mode, pos_targets[n], args.vel,
                                     args.kp, args.kd, args.tff)
                    send(bus, args.mode, n, data, f"cmd pos={pos_targets[n]}")
                print(f"[stream] feedback for {args.hold}s (Ctrl-C to stop):")
                stream(bus, args.hold, nodes)
        else:
            print(f"[stream] feedback for {args.hold}s (Ctrl-C to stop):")
            stream(bus, args.hold, nodes)

    except KeyboardInterrupt:
        print("\n[abort] Ctrl-C")
    finally:
        # Crucial: Disable motors BEFORE showing the blocking matplotlib window
        print("[disable] all motors")
        for n in nodes:
            try:
                send(bus, args.mode, n, UNIVERSAL["disable"])
            except Exception:
                pass
            time.sleep(0.02)
        bus.shutdown()
        
        # Display performance graphs safely once motors are disabled
        if args.mode == "mit" and not args.enable_only:
            has_data = any(len(history[n]['t']) > 0 for n in nodes)
            if has_data:
                plot_responses(history, pos_targets, current_positions, args.ramp)

if __name__ == "__main__":
    main()