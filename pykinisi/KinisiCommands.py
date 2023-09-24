# ----------------------------------------------------------------------------
# Kinisi motr controller commands.
# This file is auto generated by the commands generator from JSON file.
# Do not edit this file manually.
# Timestamp: 2023-09-19 23:00:00
# Version: 1.0.0
# ----------------------------------------------------------------------------

import struct

INITIALIZE_MOTOR = 0x01
SET_MOTOR_SPEED = 0x02
INITIALIZE_MOTOR_CONTROLLER = 0x03
SET_MOTOR_TARGET_VELOCITY = 0x04
DELETE_MOTOR_CONTROLLER = 0x05
INITIALIZE_ENCODER = 0x11
GET_ENCODER_VALUE = 0x12
SET_PIN_LOW = 0x20
SET_PIN_HIGH = 0x21
TOGGLE_PIN = 0x22
SET_STATUS_LED_OFF = 0x23
SET_STATUS_LED_ON = 0x24
TOGGLE_STATUS_LED = 0x25
INITIALIZE_PLATFORM = 0x30
SET_PLATFORM_VELOCITY_INPUT = 0x31
SET_PLATFORM_CONTROLLER = 0x32


class KinisiCommands:
    # Write command to serial interface
    def write(self, msg: bytearray):
        """Abstract method to write the byte message to the serial interface."""
        raise NotImplementedError("This method should be overridden by subclass.")
    
    # Read command from serial interface
    def read(self, length: int) -> bytearray:
        """Abstract method to read a specified number of bytes from the serial interface."""
        raise NotImplementedError("This method should be overridden by subclass.")

        # This command initializes a motor and prepares it for use.
    def initialize_motor(self, motor_index:int):
        msg = INITIALIZE_MOTOR.to_bytes(1, 'little') + motor_index.to_bytes(1, 'little')
        length = len(msg)
        msg = length.to_bytes(1, 'little') + msg
        self.write(msg)

    # This command sets the speed of the specified motor.
    def set_motor_speed(self, motor_index:int, direction:int, speed:int):
        msg = SET_MOTOR_SPEED.to_bytes(1, 'little') + motor_index.to_bytes(1, 'little') + direction.to_bytes(1, 'little') + speed.to_bytes(2, 'little')
        length = len(msg)
        msg = length.to_bytes(1, 'little') + msg
        self.write(msg)

    # This command sets the controller for the specified motor.
    def initialize_motor_controller(self, motor_index:int, kp:float, ki:float, kd:float):
        msg = INITIALIZE_MOTOR_CONTROLLER.to_bytes(1, 'little') + motor_index.to_bytes(1, 'little') + bytearray(struct.pack('d', kp)) + bytearray(struct.pack('d', ki)) + bytearray(struct.pack('d', kd))
        length = len(msg)
        msg = length.to_bytes(1, 'little') + msg
        self.write(msg)

    # This command sets the target velocity for the specified motor.
    def set_motor_target_velocity(self, motor_index:int, direction:int, speed:int):
        msg = SET_MOTOR_TARGET_VELOCITY.to_bytes(1, 'little') + motor_index.to_bytes(1, 'little') + direction.to_bytes(1, 'little') + speed.to_bytes(2, 'little')
        length = len(msg)
        msg = length.to_bytes(1, 'little') + msg
        self.write(msg)

    # This command deletes the controller for the specified motor.
    def delete_motor_controller(self, motor_index:int):
        msg = DELETE_MOTOR_CONTROLLER.to_bytes(1, 'little') + motor_index.to_bytes(1, 'little')
        length = len(msg)
        msg = length.to_bytes(1, 'little') + msg
        self.write(msg)

    # This command initializes an encoder and prepares it for use.
    def initialize_encoder(self, encoder_index:int):
        msg = INITIALIZE_ENCODER.to_bytes(1, 'little') + encoder_index.to_bytes(1, 'little')
        length = len(msg)
        msg = length.to_bytes(1, 'little') + msg
        self.write(msg)

    # This command retrieves the current value of the encoder.
    def get_encoder_value(self, encoder_index:int):
        msg = GET_ENCODER_VALUE.to_bytes(1, 'little') + encoder_index.to_bytes(1, 'little')
        length = len(msg)
        msg = length.to_bytes(1, 'little') + msg
        self.write(msg)
        return self.read(4)

    # This command sets the specified pin to a low state.
    def set_pin_low(self, pin_number:int):
        msg = SET_PIN_LOW.to_bytes(1, 'little') + pin_number.to_bytes(1, 'little')
        length = len(msg)
        msg = length.to_bytes(1, 'little') + msg
        self.write(msg)

    # This command sets the specified pin to a high state.
    def set_pin_high(self, pin_number:int):
        msg = SET_PIN_HIGH.to_bytes(1, 'little') + pin_number.to_bytes(1, 'little')
        length = len(msg)
        msg = length.to_bytes(1, 'little') + msg
        self.write(msg)

    # This command toggles the specified pin.
    def toggle_pin(self, pin_number:int):
        msg = TOGGLE_PIN.to_bytes(1, 'little') + pin_number.to_bytes(1, 'little')
        length = len(msg)
        msg = length.to_bytes(1, 'little') + msg
        self.write(msg)

    # This command turns off the status LED.
    def set_status_led_off(self):
        msg = SET_STATUS_LED_OFF.to_bytes(1, 'little')
        length = len(msg)
        msg = length.to_bytes(1, 'little') + msg
        self.write(msg)

    # This command turns on the status LED.
    def set_status_led_on(self):
        msg = SET_STATUS_LED_ON.to_bytes(1, 'little')
        length = len(msg)
        msg = length.to_bytes(1, 'little') + msg
        self.write(msg)

    # This command toggles the status LED.
    def toggle_status_led(self):
        msg = TOGGLE_STATUS_LED.to_bytes(1, 'little')
        length = len(msg)
        msg = length.to_bytes(1, 'little') + msg
        self.write(msg)

    # This command initializes the platform and prepares it for use.
    def initialize_platform(self):
        msg = INITIALIZE_PLATFORM.to_bytes(1, 'little')
        length = len(msg)
        msg = length.to_bytes(1, 'little') + msg
        self.write(msg)

    # This command sets the velocity input for the platform.
    def set_platform_velocity_input(self, x:int, y:int, t:int):
        msg = SET_PLATFORM_VELOCITY_INPUT.to_bytes(1, 'little') + x.to_bytes(1, 'little', signed=True) + y.to_bytes(1, 'little', signed=True) + t.to_bytes(1, 'little', signed=True)
        length = len(msg)
        msg = length.to_bytes(1, 'little') + msg
        self.write(msg)

    # This command sets the controller for the platform.
    def set_platform_controller(self):
        msg = SET_PLATFORM_CONTROLLER.to_bytes(1, 'little')
        length = len(msg)
        msg = length.to_bytes(1, 'little') + msg
        self.write(msg)

