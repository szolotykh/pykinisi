"""Read fitted board sensors: python Sensors.py COM3. Motors are not enabled."""

import argparse
from pykinisi import KinisiController, ControllerError


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("port", help="Serial port, for example COM3 or /dev/ttyACM0")
    args = parser.parse_args()
    with KinisiController() as controller:
        if not controller.connect(args.port):
            raise SystemExit(f"Connection failed: {controller.last_error}")
        profile = controller.board_profile
        if profile is None:
            print("Unknown board profile; update the SDK to discover its sensors.")
            return
        print(f"Board: {profile['board']['revision']}; resources: {dict(profile['resources'])}")
        for kind, sensors in profile['sensors'].items():
            for sensor in sensors:
                index = sensor['id']
                if not controller.supports_sensor(kind, index):
                    print(f"Update firmware to read {kind} sensor {index}.")
                    continue
                try:
                    if kind == 'temperature':
                        print(f"Temperature {index}: {controller.get_temperature(index):.2f} degrees C")
                    elif kind == 'imu':
                        sample = controller.get_imu(index)
                        print(f"IMU {index}: acceleration (m/s^2) = "
                              f"{sample.acceleration_x:.3f}, {sample.acceleration_y:.3f}, {sample.acceleration_z:.3f}")
                        print(f"IMU {index}: angular velocity (rad/s) = "
                              f"{sample.angular_velocity_x:.3f}, {sample.angular_velocity_y:.3f}, {sample.angular_velocity_z:.3f}")
                except ControllerError as error:
                    print(f"{kind} {index}: {error}; retry after the sensor is ready.")


if __name__ == '__main__':
    main()
