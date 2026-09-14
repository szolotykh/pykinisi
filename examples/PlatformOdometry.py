"""Capture timestamped platform odometry for ten seconds using messages API v2."""

import time

from pykinisi import ControllerError, ErrorCode
from Core import InitTest


def main():
    """Initialize a platform, poll its measured pose, then stop and disconnect."""
    controller = InitTest()
    platform_initialized = False
    odometry_started = False

    try:
        platform_type = "omni"
        # Example timer ticks per wheel revolution: adjust for your encoder/gearing.
        encoder_resolution = 2048

        if platform_type == "omni":
            controller.initialize_omni_platform(
                is_reversed_0=False,
                is_reversed_1=False,
                is_reversed_2=False,
                is_encoder_reversed_0=False,
                is_encoder_reversed_1=False,
                is_encoder_reversed_2=False,
                wheels_diameter=0.1,  # 10 cm
                robot_radius=0.15,  # 15 cm
                encoder_resolution=encoder_resolution,
            )
        elif platform_type == "mecanum":
            controller.initialize_mecanum_platform(
                is_reversed_0=False,
                is_reversed_1=False,
                is_reversed_2=False,
                is_reversed_3=False,
                is_encoder_reversed_0=False,
                is_encoder_reversed_1=False,
                is_encoder_reversed_2=False,
                is_encoder_reversed_3=False,
                length=0.5,  # 50 cm
                width=0.4,  # 40 cm
                wheels_diameter=0.1,  # 10 cm
                encoder_resolution=encoder_resolution,
            )
        else:
            raise ValueError(f"Unknown platform type: {platform_type}")

        platform_initialized = True
        controller.start_platform_odometry()
        odometry_started = True
        controller.set_platform_velocity(40, 0, 0)

        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            try:
                sample = controller.get_platform_odometry()
            except ControllerError as error:
                if error.error_code != ErrorCode.SAMPLE_NOT_AVAILABLE:
                    raise
                # Starting/resetting odometry invalidates the previous sample.
                time.sleep(0.05)
                continue

            mode = "unix_us" if sample.clock_mode == 1 else "uptime_us"
            print(
                f"{mode}: {sample.timestamp_us}, quality: {sample.clock_quality}, "
                f"X: {sample.x:.2f} m, Y: {sample.y:.2f} m, Theta: {sample.t:.2f} rad"
            )
            time.sleep(0.5)
    finally:
        # Attempt each cleanup step even if an earlier command fails.
        try:
            if platform_initialized:
                controller.set_platform_velocity(0, 0, 0)
        finally:
            try:
                if odometry_started:
                    controller.stop_platform_odometry()
            finally:
                controller.disconnect()


if __name__ == "__main__":
    main()
