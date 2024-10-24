#!/usr/bin/env python3
import math
import numpy as np

class PID:
    def __init__(self, min_val, max_val, kp, ki, kd):
        """A basic PID controller"""
        self.min_val = min_val
        self.max_val = max_val
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0
        self.prev_error = 0
        self.derivative = 0

    def compute(self, setpoint, measured_value):
        """computes the PID value based on constants and error"""
        error = setpoint - measured_value

        #the integral term tended to rise too quickly so I capped it at 1 or -1
        if (abs(self.integral) <= 1):
            self.integral += error
        elif self.integral > 0: self.integral = 1
        else: self.integral = -1

        self.derivative = error - self.prev_error

        if setpoint == 0 and self.prev_error == 0:
            self.integreal = 0
        self.prev_error = error

        pid = self.kp * error + self.ki*self.integral + self.kd * self.derivative
        #print(pid)

        return np.clip(pid, self.min_val, self.max_val)
