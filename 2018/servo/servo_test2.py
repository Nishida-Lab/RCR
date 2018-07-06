import pigpio
import time

PWM_PIN 19
frequency 50

pi = pigpio.pi()
pi.set_mode(PWM_PIN, pigpio.OUTPUT)

for i in range(0.9,2.2,0.1):
    duty = 1000000 * i / 20
    pi.hardware_PWM(PWM_PIN, frequency, duty)
    time.sleep(5)

pi.set_mode(PWM_PIN, pigpio.INPUT)
pi.stop()
