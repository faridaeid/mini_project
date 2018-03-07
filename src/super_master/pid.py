#!/usr/bin/env python

import time


class PID:
    def __init__(self, p=0.2, i=0.0, d=0.0):

        self.Kp = p
        self.Ki = i
        self.Kd = d

        self.current_time = time.time()
        self.last_time = self.current_time
        self.set_point = 0
        self.last_error = 0.0

        self.iterm = 0.0

        # Windup Guard
        self.int_error = 0.0
        self.windup_guard = 20.0

    def update(self, feedback_value):

        error = self.set_point - feedback_value

        self.current_time = time.time()
        delta_time = self.current_time - self.last_time
        delta_error = error - self.last_error

        pterm = self.Kp * error
        self.iterm += error * delta_time

        if self.iterm < -self.windup_guard:
            self.iterm = -self.windup_guard
        elif self.iterm > self.windup_guard:
            self.iterm = self.windup_guard

        dterm = 0.0
        if delta_time > 0:
            dterm = delta_error / delta_time

        # Remember last time and last error for next calculation
        self.last_time = self.current_time
        self.last_error = error

        output = pterm + (self.Ki * self.iterm) + (self.Kd * dterm)

        return output

    def set_windup(self, windup):
        self.windup_guard = windup

    def set_setpoint(self, set_point):
        self.set_point = set_point
