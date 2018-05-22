#!/usr/bin/python
# -*- coding: utf-8 -*-
#
# DC Motor control with MD10C R3 - pigpio

import pigpio
import time

PWMPIN = 18
DIRPIN = 23
pi1 = pigpio.pi()
pi1.set_mode(PWMPIN, pigpio.OUTPUT)
pi1.set_mode(DIRPIN, pigpio.OUTPUT)

for DUTY_CYCLE in range(100000, 1000000, 100000):
    pi1.write(DIRPIN, 0)
    pi1.hardware_PWM(PWMPIN, 500, DUTY_CYCLE)
    time.sleep(5)

pi1.set_mode(PWMPIN, pigpio.INPUT)
pi1.stop()
