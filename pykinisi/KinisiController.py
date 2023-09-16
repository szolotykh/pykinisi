import serial

from pykinisi.KinisiCommands import MotorIndex
from .KinisiCommands import *

SpeedResolution = 840

class KinisiController(KinisiCommands):
    def connect(self, port):
        try:
            self.serial =  serial.Serial(port, 115200, timeout = 1)
            return True
        except serial.SerialException:
            return False
        except:
            return False

    def write(self, msg: bytearray):
        self.serial.write(msg)

    def read(self, length: int) -> bytearray:
        return self.serial.read(length)
    
    def set_motor_speed(self, motorIndex: MotorIndex, direction: bool, speed: int):
        speed = int(speed * SpeedResolution / 100)
        direction = int(direction)
        return super().set_motor_speed(motorIndex, direction, speed)
    
    def set_motor_target_velocity(self, motorIndex:MotorIndex, direction:int, speed:int):
        speed = int(speed * SpeedResolution / 100)
        direction = int(direction)
        return super().set_motor_target_velocity(motorIndex, direction, speed)

    # Motors
    def initialoze_motor_all(self):
        self.initialize_motor(MotorIndex.Motor0)
        self.initialize_motor(MotorIndex.Motor1)
        self.initialize_motor(MotorIndex.Motor2)
        self.initialize_motor(MotorIndex.Motor3)

    def stop_motor(self, index):
        self.set_motor_speed(index, True, 0)

    def stop_motor_all(self):
        self.stop_motor(MotorIndex.Motor0)
        self.stop_motor(MotorIndex.Motor1)
        self.stop_motor(MotorIndex.Motor2)
        self.stop_motor(MotorIndex.Motor3)

    def __del__(self):
        if self.serial:
            self.serial.close()