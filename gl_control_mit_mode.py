#!/usr/bin/env python3
"""
gl_control.py  --  CubeMars GL40 II KV82.5 control + feedback over SocketCAN.
Supports multiple motors on one bus (daisy-chained). Verified MIT/PV/Vel packing
and §5.4 feedback decode.

Your setup: two motors, CAN IDs 1 and 3, both in MIT mode, on can0 @ 1 Mbps.

Setup:
    sudo ip link set can0 up type can bitrate 1000000

Single motor:
    python3 gl_control.py --mode mit --node 1 --pos 0 --vel 0 --kp 5 --kd 1 --tff 0 --hold 3
    python3 gl_control.py --mode mit --node 3 --enable-only --hold 3

Both motors (IDs default 1 and 3):
    # mirrored: same command to both
    python3 gl_control.py --mode mit --both --pos 0.5 --vel 0 --kp 5 --kd 1 --hold 3
    # independent: per-motor positions
    python3 gl_control.py --mode mit --both --pos2 "1:0.3,3:-0.3" --kp 5 --kd 1 --hold 3
    # just enable both and watch feedback
    python3 gl_control.py --mode mit --both --enable-only --hold 3

SAFETY: secure both motors. In MIT mode the motor actively drives to the target and an
unloaded torque command spins to high speed. Start with small pos, low kp/kd. The script
always disables on exit.
"""

import argparse
import struct
import sys
import time

import can

CHANNEL = "can0"
DEFAULT_NODES = [1, 3]            # your two motors

MODE_PREFIX = {"mit": 0x0, "pv": 0x1, "vel": 0x2}

# feedback scaling (section 5.4)
P_MIN, P_MAX = -12.5, 12.5
V_MIN, V_MAX = -200.0, 200.0
T_MIN, T_MAX = -10.0, 10.0

# MIT command packing ranges (verified against section 5.5 examples)
MIT_P_MIN, MIT_P_MAX = -12.5, 12.5
MIT_V_MIN, MIT_V_MAX = -65.0, 65.0
MIT_KP_MIN, MIT_KP_MAX = 0.0, 500.0
MIT_KD_MIN, MIT_KD_MAX = 0.0, 5.0
MIT_T_MIN, MIT_T_MAX = -10.0, 10.0
KT = 0.10  # Approx Torque Constant (Nm/A) for KV82.5 motor

LOOP_HZ = 500   # MIT command rate (Hz) — needs ~1kHz ideally; 500Hz is Python's practical ceiling

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
    """Print decoded feedback for `seconds`, labeled per source CAN ID."""
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
        if now - last.get(n, 0) > 0.15:    # ~6 Hz per motor
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
                  hold, ramp, current_positions):
    """Send MIT commands at LOOP_HZ, ramping from current_pos to target_pos."""
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

        # Non-blocking recv so we never stall the control loop waiting for a frame.
        msg = bus.recv(timeout=0)
        if msg:
            fb = decode(msg.data)
            if fb and fb["canid"] in nodes:
                n   = fb["canid"]
                now = time.time()
                if now - last_print.get(n, 0) > 0.2:
                    alpha_pct = min(elapsed / ramp * 100, 100) if ramp > 0 else 100
                    print(f"  [M{n}] pos={fb['pos']:+6.3f} rad  spd={fb['spd']:+7.2f} r/s  "
                          f"torque={fb['torque']:+6.3f} Nm  current~={fb['current']:+6.2f} A  "
                          f"ramp={alpha_pct:.0f}%  [{fb['err_name']}]")
                    last_print[n] = now

        remaining = dt - (time.time() - t_loop)
        if remaining > 0:
            time.sleep(remaining)


def parse_pos2(s):
    """'1:0.3,3:-0.3' -> {1:0.3, 3:-0.3}"""
    out = {}
    for part in s.split(","):
        k, v = part.split(":")
        out[int(k)] = float(v)
    return out


def main():
    ap = argparse.ArgumentParser(description="GL40 II CAN control + feedback (multi-motor)")
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
                    help="MIT: seconds to ramp from current pos to target (default 1.0)")
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

    try:
        bus = can.Bus(channel=CHANNEL, interface="socketcan")
    except Exception as e:
        print(f"Could not open {CHANNEL}: {e}")
        print("Bring it up:  sudo ip link set can0 up type can bitrate 1000000")
        sys.exit(1)

    try:
        # Enable each motor and capture current position from the enable-response frame.
        current_positions: dict = {}
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
                              args.hold, args.ramp, current_positions)
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
        print("[disable] all motors")
        for n in nodes:
            try:
                send(bus, args.mode, n, UNIVERSAL["disable"])
            except Exception:
                pass
            time.sleep(0.02)
        bus.shutdown()


if __name__ == "__main__":
    main()