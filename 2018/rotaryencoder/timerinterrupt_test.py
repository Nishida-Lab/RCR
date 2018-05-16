#!/usr/bin/env python3
import signal
import time
import pigpio

SAMPLING_TIME = 0.1
ENCODER_PIN = 23

# Variable for DCM drive
pwm_pin = 18
dir_pin = 17
freq = 100
CW = 0
CCW = 1
DUTY_MIN, DUTY_MAX = 0.0, 999999.0

count = 0
velocity = 0

def calc_vel(arg1, arg2):
    global count
    global velocity

    velocity = (count * (360 / 500)) / SAMPLING_TIME
    print(velocity)
    count = 0

def count_pulse(arg1, arg2, arg3):
    global count
    count += 1

signal.signal(signal.SIGALRM, calc_vel)
signal.setitimer(signal.ITIMER_REAL, SAMPLING_TIME, SAMPLING_TIME)

pi = pigpio.pi()
pi.set_mode(ENCODER_PIN, pigpio.INPUT)
pi.callback(ENCODER_PIN, pigpio.RISING_EDGE, count_pulse)

# DCM drive
pi.set_mode(pwm_pin, pigpio.OUTPUT)
pi.set_mode(dir_pin, pigpio.OUTPUT)
pi.write(dir_pin, CW)
pi.hardware_PWM(pwm_pin, freq, int(DUTY_MAX))

while True:
    try:
        pass
    except:
        pi.set_mode(pwm_pin, pigpio.INPUT)
        pi.set_mode(dir_pin, pigpio.INPUT)
        pi.stop()
        exit(0)
