import rospy
from rcr2018.msg import DcmCommand
import pigpio
import time

def period2freq(period):
    return int(1000 / period)

def width2duty(width, period):
    return int((width / period) * 1000000)

def callback(message):
    global dcm_pwm_pin
    global freq
    global DUTY_MAX
    global dcm_powersave_coef
    duty = int(message.cmd_vel * DUTY_MAX * dcm_powersave_coef)
    pi.hardware_PWM(dcm_pwm_pin, freq, duty)

dcm_pwm_pin = 18
dir_pin = 23

freq = 100
CW = 0
CCW = 1
DUTY_MIN, DUTY_MAX = 0.0, 999999.0
JOYAXIS_MIN, JOYAXIS_MAX = 0.0, 1.0
PERIOD = 20
dcm_powersave_coef = 0.2
svm_powersave_coef = 1
    
pi = pigpio.pi()
pi.set_mode(dcm_pwm_pin, pigpio.OUTPUT)
pi.set_mode(dir_pin, pigpio.OUTPUT)
pi.write(dir_pin, CCW)

rospy.init_node('dummy_dcmdriver')
sub = rospy.Subscriber('dcm_command', DcmCommand, callback)

rospy.spin()

pi.set_mode(dcm_pwm_pin, pigpio.INPUT)
pi.set_mode(dir_pin, pigpio.INPUT)
pi.stop()
