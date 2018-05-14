#!/usr/bin/env python3

import pygame
from pygame.locals import *
import pigpio
import time

pygame.joystick.init()
try:
    j = pygame.joystick.Joystick(0)
    j.init()
except pygame.error:
    print('Joystick not found')
    exit(-1)

def main():
    pygame.init()

    pwm_pin = 18
    dir_pin = 17
    freq = 100
    CW = 0
    CCW = 1
    DUTY_MIN, DUTY_MAX = 0.0, 999999.0
    JOYAXIS_MIN, JOYAXIS_MAX = 0.0, 1.0

    pi = pigpio.pi()
    pi.set_mode(pwm_pin, pigpio.OUTPUT)
    pi.set_mode(dir_pin, pigpio.OUTPUT)

    try:
        x_old, y_old = 0.0, 0.0
        while 1:
            for e in pygame.event.get():
                if e.type == pygame.locals.JOYAXISMOTION:
                    x_new, y_new = j.get_axis(0), j.get_axis(1)
                # if not (x_new == x_old and y_new == x_old):
                if True:
                    x_old , y_old = x_new, y_new
                    print('x:{}, y:{}'.format(x_new, y_new))
                    if y_new > 0:
                        pi.write(dir_pin, CW)
                        duty = int(y_new * DUTY_MAX)
                        pi.hardware_PWM(pwm_pin, freq, duty)
                    elif y_new == 0.0:
                        pi.hardware_PWM(pwm_pin, freq, 0)
                    else:
                        pi.write(dir_pin, CCW)
                        duty = int(abs(y_new * DUTY_MAX))
                        pi.hardware_PWM(pwm_pin, freq, duty)
                    pygame.event.clear()

    except KeyboardInterrupt:
        pi.set_mode(pwm_pin, pigpio.INPUT)
        pi.set_mode(dir_pin, pigpio.INPUT)
        pi.stop()
        exit(0)

if __name__ == '__main__':
    main()
