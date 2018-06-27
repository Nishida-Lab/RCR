from __future__ import division
import pigpio
import time

def calc_ang(i):
    return (-100 * (i - 1.5))


PWM_PIN =  19
frequency = 50
range_list = [0.1 * x for x in range(9, 22)]
input_ang = []
servo_ang = []

pi = pigpio.pi()
pi.set_mode(PWM_PIN, pigpio.OUTPUT)

for i in range_list:
    duty = 1000000 * i / 20
    pi.hardware_PWM(PWM_PIN, frequency, duty)
    print(calc_ang(i))
    servo_ang.append(calc_ang(i))
    
    value = raw_input('input measurement ang')
    input_ang.append(value)

print('servo_ang:input_ang')
for s, i in zip(servo_ang, input_ang):
    print('{}:{}'.format(s, i))

pi.set_mode(PWM_PIN, pigpio.INPUT)
pi.stop()
