#!/usr/bin/env python3
import pigpio
import sys
import time

PWM_PIN = 18

PERIOD = 20
STOP_WIDTH = 1.5

def period2freq(period):
    return int(1000 / period)

def width2duty(width, period):
    return int((width / period) * 1000000)

try:
    pi = pigpio.pi()

    pi.set_mode(PWM_PIN, pigpio.OUTPUT)

    while True:
        # neutral
        sys.stdout.write('\r{}'.format('Neutral'))
        sys.stdout.flush()
        pi.hardware_PWM(PWM_PIN, period2freq(PERIOD), width2duty(STOP_WIDTH, PERIOD))
        time.sleep(3)
        # CW
        sys.stdout.write('\r{}'.format('CW'))
        sys.stdout.flush()
        pi.hardware_PWM(PWM_PIN, period2freq(PERIOD), width2duty(0.9, PERIOD))
        time.sleep(3)
        # CCW
        sys.stdout.write('\r{}'.format('CCW'))
        sys.stdout.flush()
        pi.hardware_PWM(PWM_PIN, period2freq(PERIOD), width2duty(2.1, PERIOD))
        time.sleep(3)

except KeyboardInterrupt:
    pi.set_mode(PWM_PIN, pigpio.INPUT)
    pi.stop()
    exit(0)
