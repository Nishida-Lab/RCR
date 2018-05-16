#!/usr/bin/env python3

import pygame
from pygame.locals import *
import pigpio
import time

def period2freq(period):
    return int(1000 / period)

def width2duty(width, period):
    return int((width / period) * 1000000)

pygame.joystick.init()
try:
    j = pygame.joystick.Joystick(0)
    j.init()
except pygame.error:
    print('Joystick not found')
    exit(-1)

def main():
    pygame.init()

    dcm_pwm_pin = 18
    dir_pin = 23
    svm_pwm_pin = 19

    freq = 100
    CW = 0
    CCW = 1
    DUTY_MIN, DUTY_MAX = 0.0, 999999.0
    JOYAXIS_MIN, JOYAXIS_MAX = 0.0, 1.0
    PERIOD = 20

    pi = pigpio.pi()
    pi.set_mode(dcm_pwm_pin, pigpio.OUTPUT)
    pi.set_mode(dir_pin, pigpio.OUTPUT)

    try:
        x_old, y_old = 0.0, 0.0
        while 1:
            for e in pygame.event.get():
                if e.type == pygame.locals.JOYAXISMOTION:
                    x_new, y_new = j.get_axis(0), j.get_axis(1)
                if True:
                    x_old , y_old = x_new, y_new
                    print('x:{}, y:{}'.format(x_new, y_new))
                    if y_new > 0:
                        pi.write(dir_pin, CW)
                        duty = int(y_new * DUTY_MAX)
                        pi.hardware_PWM(dcm_pwm_pin, freq, duty)
                    elif y_new == 0.0:
                        pi.hardware_PWM(dcm_pwm_pin, freq, 0)
                    else:
                        pi.write(dir_pin, CCW)
                        duty = int(abs(y_new * DUTY_MAX))
                        pi.hardware_PWM(dcm_pwm_pin, freq, duty)
                    # servo motor
                    pi.hardware_PWM(srv_pwm_pin, period2freq(PERIOD), width2duty(1.5 + (x_new * 0.6), PERIOD))
                    pygame.event.clear()

    except KeyboardInterrupt:
        pi.set_mode(dcm_pwm_pin, pigpio.INPUT)
        pi.set_mode(dir_pin, pigpio.INPUT)
        pi.stop()
        exit(0)

if __name__ == '__main__':
    main()
