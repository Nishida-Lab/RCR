import rospy
from rcr2018 import SvmCommand
import pigpio
import time

def period2freq(period):
    return int(1000 / period)

def width2duty(width, period):
    return int((width / period) * 1000000)

def callback(message):
    global svm_pwm_pin
    global PERIOD
    global DUTY_MAX
    pi.hardware_PWM(svm_pwm_pin, period2freq(PERIOD), width2duty(1.5 - message.cmd_ang_vel, PERIOD))

svm_pwm_pin = 19

freq = 100
CW = 0
CCW = 1
DUTY_MIN, DUTY_MAX = 0.0, 999999.0
JOYAXIS_MIN, JOYAXIS_MAX = 0.0, 1.0
PERIOD = 20
dcm_powersave_coef = 0.3
svm_powersave_coef = 1
    
pi = pigpio.pi()
pi.set_mode(svm_pwm_pin, pigpio.OUTPUT)

rospy.init_node('dummy_svmdriver')
sub = rospy.Subscriber('svm_command', SvmCommand, callback)

rospy.spin()

pi.set_mode(svm_pwm_pin, pigpio.INPUT)
pi.stop()
