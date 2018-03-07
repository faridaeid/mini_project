#!/usr/bin/env python

import serial
import numpy as np

class MotorController(object):

    def __init__(self):
        self.serial = serial.Serial('/dev/ttyS0', 19200, timeout=1)
        self.max_speed = 255

        self.motor1 = np.array([chr(0xCA), chr(0xC9)])
        self.motor2 = np.array([chr(0xC2), chr(0xC1)])

        self.right_motor = self.motor1
        self.left_motor = self.motor2

    def _cmd_motor_forward_reverse(self, command_forward, command_rev, speed):
        motor_command = command_forward

        if speed < 0:
            motor_command = command_rev
            speed *= -1

        speed = min(speed, self.max_speed)

        self.serial.write(motor_command)
        self.serial.write(chr(speed))

    def cmd_left_motor(self, speed):

        self._cmd_motor_forward_reverse(self.left_motor[0],
                                        self.left_motor[1],
                                        speed)

    def cmd_right_motor(self, speed):

        self._cmd_motor_forward_reverse(self.right_motor[0],
                                        self.right_motor[1],
                                        speed)

    def cmd_move_motors(self, rightspeed, leftspeed):
        self.cmd_left_motor(leftspeed)
        self.cmd_right_motor(rightspeed)

    def brake(self):
        self.cmd_move_motors(0, 0)
