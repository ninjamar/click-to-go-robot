"""
Interactive helpers for calibrating GEAR_RATIO, MIN_PWM, PWM_PER_RAD_S, and
MAX_ANGULAR_VEL. Doesn't write to config.hpp - just prints the measured value
for you to enter yourself. MAX_ANGULAR_ACCEL has no calibration path here -
hand-tune it by feel (watch for wheel slip or stall).

Usage (run from motor_control_system/py/):
    python -m scripts.calibrate gear-ratio --port /dev/cu.usbserial-XXXX --wheel-revs 0.25
    python -m scripts.calibrate min-pwm --port /dev/cu.usbserial-XXXX --start 30 --stop 150
    python -m scripts.calibrate velocity-curve --port /dev/cu.usbserial-XXXX
"""

import argparse
import math
import time

import serial
from protocol import OP, RESPONSE_SIZE, build_command_packet, parse_response_packet


def open_conn(port, baud):
    conn = serial.Serial(port, baud, timeout=2)
    time.sleep(2)
    conn.reset_input_buffer()
    return conn


def send(conn, seq_id, op, arg, direction, timeout=2.0):
    conn.timeout = timeout
    conn.write(build_command_packet(seq_id, op, arg, direction))
    data = b""
    while len(data) < RESPONSE_SIZE:
        chunk = conn.read(RESPONSE_SIZE - len(data))
        if not chunk:
            break
        data += chunk
    return parse_response_packet(data)


def cmd_gear_ratio(args):
    conn = open_conn(args.port, args.baud)
    input(
        f"Motor should be unpowered. Ready to hand-turn the WHEEL exactly "
        f"{args.wheel_revs} revolution(s) in one direction. Press Enter to set the baseline..."
    )
    send(conn, 1, OP.POLL_W, 0, 0)  # zeroes the odometer's accumulator
    t0 = time.time()
    input(f"Now turn the wheel {args.wheel_revs} revolution(s), then press Enter...")
    t1 = time.time()
    resp = send(conn, 2, OP.POLL_W, 0, 0)
    dt = t1 - t0
    motor_shaft_revs = abs(resp.w2) * dt / (2 * math.pi)
    gear_ratio = motor_shaft_revs / args.wheel_revs
    print(
        f"w2={resp.w2:.4f} rad/s over dt={dt:.2f}s -> motor shaft turned {motor_shaft_revs:.3f} rev"
    )
    print(f"GEAR_RATIO ~= {gear_ratio:.3f}")
    conn.close()


def cmd_min_pwm(args):
    conn = open_conn(args.port, args.baud)
    print(
        "Make sure the wheel is under normal load (on the ground), not free-spinning."
    )
    seq = 1
    pwm = args.start
    while pwm <= args.stop:
        send(conn, seq, OP.STOP, 0, 0)
        seq += 1
        time.sleep(0.2)  # settle to a dead stop before testing stiction again
        send(conn, seq, OP.DEBUG_SET_PWM, float(pwm), args.direction)
        seq += 1
        time.sleep(args.hold)
        resp = send(conn, seq, OP.POLL_W, 0, 0)
        seq += 1
        print(f"pwm={pwm}: w2={resp.w2:.3f} rad/s")
        moved = (
            input("  Did the wheel actually start turning from a dead stop? (y/n) ")
            .strip()
            .lower()
        )
        send(conn, seq, OP.STOP, 0, 0)
        seq += 1
        if moved == "y":
            print(
                f"MIN_PWM candidate: {pwm} (rerun this sweep a few times - stiction varies trial to trial)"
            )
            conn.close()
            return
        pwm += args.step
    print("Never moved within [--start, --stop] - widen the range.")
    conn.close()


def cmd_velocity_curve(args):
    conn = open_conn(args.port, args.baud)
    print(
        "Make sure the wheel is under normal load (on the ground), not free-spinning."
    )
    seq = 1
    samples = []
    pwm = args.start
    while pwm <= args.stop:
        send(conn, seq, OP.STOP, 0, 0)
        seq += 1
        time.sleep(0.2)  # settle to a dead stop
        send(conn, seq, OP.DEBUG_SET_PWM, float(pwm), args.direction)
        seq += 1
        time.sleep(args.hold)  # reach steady-state speed
        resp = send(conn, seq, OP.POLL_W, 0, 0)
        seq += 1
        print(f"pwm={pwm}: w2={resp.w2:.3f} rad/s")
        samples.append((pwm, abs(resp.w2)))
        pwm += args.step
    send(conn, seq, OP.STOP, 0, 0)
    conn.close()

    # Least-squares slope through the MIN_PWM anchor (already calibrated separately).
    num = sum(w * (p - args.min_pwm) for p, w in samples)
    den = sum(w * w for _, w in samples)
    pwm_per_rad_s = num / den if den > 0 else 0.0

    omega_at_max = max(w for p, w in samples if p == args.stop)
    max_angular_vel = 0.9 * omega_at_max  # margin below what MAX_PWM can sustain

    print(f"PWM_PER_RAD_S ~= {pwm_per_rad_s:.5f}")
    print(
        f"MAX_ANGULAR_VEL ~= {max_angular_vel:.2f} rad/s (90% of {omega_at_max:.2f} measured at pwm={args.stop})"
    )


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    common = argparse.ArgumentParser(add_help=False)
    common.add_argument("--port", required=True, help="Serial port path")
    common.add_argument("--baud", type=int, default=57600, help="Baud rate")
    sub = parser.add_subparsers(dest="command", required=True)

    p_gear = sub.add_parser("gear-ratio", help="Calibrate GEAR_RATIO", parents=[common])
    p_gear.add_argument(
        "--wheel-revs",
        type=float,
        default=0.25,
        help="Wheel revolutions you'll hand-turn",
    )
    p_gear.set_defaults(func=cmd_gear_ratio)

    p_min = sub.add_parser("min-pwm", help="Find MIN_PWM", parents=[common])
    p_min.add_argument("--start", type=int, default=20)
    p_min.add_argument("--stop", type=int, default=150)
    p_min.add_argument("--step", type=int, default=5)
    p_min.add_argument(
        "--hold", type=float, default=1.0, help="Seconds to hold each PWM level"
    )
    p_min.add_argument("--direction", type=int, choices=[-1, 1], default=1)
    p_min.set_defaults(func=cmd_min_pwm)

    p_vel = sub.add_parser(
        "velocity-curve", help="Fit PWM_PER_RAD_S / MAX_ANGULAR_VEL", parents=[common]
    )
    p_vel.add_argument("--start", type=int, default=20)
    p_vel.add_argument("--stop", type=int, default=160)
    p_vel.add_argument("--step", type=int, default=10)
    p_vel.add_argument(
        "--hold",
        type=float,
        default=1.5,
        help="Seconds to reach steady-state speed at each level",
    )
    p_vel.add_argument("--direction", type=int, choices=[-1, 1], default=1)
    p_vel.add_argument(
        "--min-pwm",
        type=int,
        default=20,
        help="Current MIN_PWM value, used as the fit anchor",
    )
    p_vel.set_defaults(func=cmd_velocity_curve)

    args = parser.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
