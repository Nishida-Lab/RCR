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

pi1.write(DIRPIN, 1)
time.sleep(15)
pi1.hardware_PWM(PWMPIN, 500, 300000)
time.sleep(5)

pi1.set_mode(PWMPIN, pigpio.INPUT)
pi1.stop()
