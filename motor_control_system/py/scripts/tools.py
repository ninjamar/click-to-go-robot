"""
Misc one-off dev tools.

Usage (run from motor_control_system/py/):
    python -m scripts.tools poll-w --port /dev/cu.usbserial-XXXX
"""

import argparse
import time

from protocol import OP, SerialClient


def cmd_poll_w(args):
    """Continuously poll OP_POLL_W - hand-turn the wheels and watch live w1/w2."""
    client = SerialClient(args.port, args.baud)
    try:
        while True:
            resp = client.call(OP.POLL_W)
            print(f"w1={resp.w1:.4f} rad/s, w2={resp.w2:.4f} rad/s")
            time.sleep(args.interval)
    except KeyboardInterrupt:
        pass
    finally:
        client.close()


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    common = argparse.ArgumentParser(add_help=False)
    common.add_argument("--port", required=True, help="Serial port path")
    common.add_argument("--baud", type=int, default=57600, help="Baud rate")
    sub = parser.add_subparsers(dest="command", required=True)

    p_poll = sub.add_parser(
        "poll-w", help="Continuously poll angular velocity", parents=[common]
    )
    p_poll.add_argument(
        "--interval", type=float, default=0.2, help="Seconds between polls"
    )
    p_poll.set_defaults(func=cmd_poll_w)

    args = parser.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
