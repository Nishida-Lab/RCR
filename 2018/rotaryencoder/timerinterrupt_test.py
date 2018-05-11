#!/usr/bin/env python3
import signal
import time
import pigpio

SAMPLING_TIME = 0.1
ENCODER_PIN = 23

count = 0
velocity = 0

def calc_vel(arg1, arg2):
    global count
    global velocity

    velocity = (count * (360 / 500)) / SAMPLING_TIME
    count = 0

def count_pulse(arg1, arg2, arg3):
    global count
    count += 1

signal.signal(signal.SIGALRM, calc_vel)
signal.setitimer(signal.ITIMER_REAL, SAMPLING_TIME, SAMPLING_TIME)

pi = pigpio.pi()
pi.set_mode(ENCODER_PIN, pigpio.INPUT)
pi.callback(ENCODER_PIN, pigpio.RISING_EDGE, count_pulse)

while True:
    time.sleep(0.15)
    print(velocity)
