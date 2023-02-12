import serial
import struct

# Commands
INITIALIZE_MOTOR = 0x01
SET_MOTOR_SPEED = 0x02
MOTOR_SET_CONTROLLER =  0x03
MOTOR_SET_TARGET_VELOCITY = 0x04
MOTOR_DEL_CONTROLLER = 0x05

INITIALIZE_ENCODER = 0x11
GET_ENCODER_VALUE = 0x12

# GPIO
PIN_LOW = 0x20
PIN_HIGH = 0x21
PIN_TOGGLE = 0x22

# Status LED
STATUS_LED_OFF = 0x23
STATUS_LED_ON  = 0x24
STATUS_LED_TOGGLE = 0x25

# Platform
PLATFORM_INITIALIZE = 0x30
PLATFORM_SET_VELOCITY_INPUT = 0x31

# Motor indexes
Motor0 = 0x00
Motor1 = 0x01
Motor2 = 0x02
Motor3 = 0x03

# Encoder indexes
Encoder0 = 0x00
Encoder1 = 0x01
Encoder2 = 0x02
Encoder3 = 0x03



SpeedResolution = 840

class Controller:
    def Connect(self, port):
        try:
            self.serial =  serial.Serial(port, 115200, timeout=1)
            return True
        except serial.SerialException:
            return False
        except:
            return False

    # Motors
    def InitializeMotor(self, index):
        msg = bytes([INITIALIZE_MOTOR, index])
        self.serial.write(msg)

    def InitializeAllMotors(self):
        self.InitializeMotor(Motor0)
        self.InitializeMotor(Motor1)
        self.InitializeMotor(Motor2)
        self.InitializeMotor(Motor3)

    def SetMotorSpeed(self, index, direction, speed):
        # Direction 0 or 1
        if speed < 0:
            speed = 0
        if speed > 100:
            speed = 100
        speed = int(speed/100*SpeedResolution)
        msg = bytes([SET_MOTOR_SPEED, index]) + direction.to_bytes(1, 'little') + speed.to_bytes(2, 'little')
        self.serial.write(msg)

    def StopMotor(self, index):
        self.SetMotorSpeed(index, True, 0)

    def StopAllMotors(self):
        self.StopMotor(Motor0)
        self.StopMotor(Motor1)
        self.StopMotor(Motor2)
        self.StopMotor(Motor3)

    def SetMotorController(self, index, kp, ki, kd):
        kpBytes = bytearray(struct.pack("d", kp))
        kpBytes.reverse()
        kiBytes = bytearray(struct.pack("d", ki))
        kiBytes.reverse()
        kdBytes = bytearray(struct.pack("d", kd))
        kdBytes.reverse()

        msg = bytes([MOTOR_SET_CONTROLLER, index]) + kpBytes + kiBytes  + kdBytes 
        self.serial.write(msg)
    
    def DelMotorController(self, index):
        msg = bytes([MOTOR_DEL_CONTROLLER, index])
        self.serial.write(msg)

    def SetMotorTargetVelocity(self, index, direction, speed):
        if speed < 0:
            speed = 0
        if speed > 100:
            speed = 100
        msg = bytes([MOTOR_SET_TARGET_VELOCITY, index]) + direction.to_bytes(1, 'little') + speed.to_bytes(2, 'little')
        self.serial.write(msg)

    # Encoders
    def InitializeEncoder(self, index):
        msg = bytes([INITIALIZE_ENCODER, index])
        self.serial.write(msg)

    def GetEncoderValue(self, index):
        msg = bytes([GET_ENCODER_VALUE, index])
        self.serial.write(msg)
        return self.serial.read(4)

    def GPIOToggleStatusLED(self):
        msg = bytes([STATUS_LED_TOGGLE])
        return self.serial.write(msg)

    def PlatformInitialize(self):
        msg = bytes([PLATFORM_INITIALIZE])
        self.serial.write(msg)

    def PlatformSetVelocityInput(self, x, y, t):
        msg = bytes([PLATFORM_SET_VELOCITY_INPUT]) + x.to_bytes(1, 'little') + y.to_bytes(1, 'little')+ t.to_bytes(1, 'little')
        self.serial.write(msg)

    def __del__(self):
        if self.serial:
            self.serial.close()