"""Read board identity and clock status after the API v2 handshake.

Run ``python Initialization.py COM3`` or add ``--uptime`` when the host
cannot provide valid Unix time. This example does not initialize any motors.
"""

import argparse

from pykinisi import KinisiController


def main():
    """Connect, display identity and time status, and close the connection."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("port", help="Serial port, for example COM3 or /dev/ttyACM0")
    parser.add_argument(
        "--uptime", action="store_true", help="Use controller uptime instead of Unix time"
    )
    args = parser.parse_args()

    controller = KinisiController(wall_clock=not args.uptime)
    if not controller.connect(args.port):
        raise SystemExit(f"Can't connect to {args.port}: {controller.last_error}")

    try:
        board = controller.board_info
        print(f"Board model: {board.board_model}")
        print(f"Board version: {board.board_major}.{board.board_minor}.{board.board_patch}")
        print(
            f"Protocol version: "
            f"{board.protocol_major}.{board.protocol_minor}.{board.protocol_patch}"
        )
        build = f"{board.firmware_build_high:08x}{board.firmware_build_low:08x}"
        print(f"Firmware Git prefix: {build if int(build, 16) else 'unavailable'}")

        status = controller.get_time_status()
        mode = "Unix time" if status.clock_mode == 1 else "controller uptime"
        quality = {0: "unready", 1: "valid", 2: "stale"}.get(
            status.clock_quality, f"unknown ({status.clock_quality})"
        )
        print(f"Ready: {controller.ready}; clock: {mode}; quality: {quality}")
        print(f"Sync interval: {status.interval_ms} ms")
        print(f"Last sync age: {status.last_sync_age_us} us")
    finally:
        controller.disconnect()


if __name__ == "__main__":
    main()
