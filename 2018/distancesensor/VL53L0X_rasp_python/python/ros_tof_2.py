# coding: utf-8
#!/usr/bin/python

# MIT License
# 
# Copyright (c) 2017 John Bryan Moore
# 
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
# 
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
# 
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import time
import VL53L0X
import RPi.GPIO as GPIO
import rospy
from rcr2018.msg import TofFront, TofSide

rospy.init_node('tofsensor')
pub_front = rospy.Publisher('tof_front', TofFront, queue_size=1)
pub_side = rospy.Publisher('tof_side', TofSide, queue_size=1)

# GPIO for Sensor 1 shutdown pin
sensor1_shutdown = 27
# GPIO for Sensor 2 shutdown pin
sensor2_shutdown = 17
# GPIO for sensor 3 shutdown pin
sensor3_shutdown = 22
GPIO.setwarnings(False)

# Setup GPIO for shutdown pins on each VL53L0X
GPIO.setmode(GPIO.BCM)
GPIO.setup(sensor1_shutdown, GPIO.OUT)
GPIO.setup(sensor2_shutdown, GPIO.OUT)
GPIO.setup(sensor3_shutdown, GPIO.OUT)

# Set all shutdown pins low to turn off each VL53L0X
GPIO.output(sensor1_shutdown, GPIO.LOW)
GPIO.output(sensor2_shutdown, GPIO.LOW)
GPIO.output(sensor3_shutdown, GPIO.LOW)

# Keep all low for 500 ms or so to make sure they reset
time.sleep(0.50)

# Create one object per VL53L0X passing the address to give to
# each.
tof = VL53L0X.VL53L0X(address=0x2B)
tof1 = VL53L0X.VL53L0X(address=0x2C)
tof2 = VL53L0X.VL53L0X(address=0x2D)

# Set shutdown pin high for the first VL53L0X then 
# call to start ranging 
GPIO.output(sensor1_shutdown, GPIO.HIGH)
time.sleep(0.50)
tof.start_ranging(VL53L0X.VL53L0X_HIGH_SPEED_MODE)

# Set shutdown pin high for the second VL53L0X then 
# call to start ranging 
GPIO.output(sensor2_shutdown, GPIO.HIGH)
time.sleep(0.50)
tof1.start_ranging(VL53L0X.VL53L0X_HIGH_SPEED_MODE)

GPIO.output(sensor3_shutdown, GPIO.HIGH)
time.sleep(0.50)
tof2.start_ranging(VL53L0X.VL53L0X_HIGH_SPEED_MODE)

timing = tof.get_timing()
if (timing < 20000):
    timing = 20000
print ("Timing %d ms" % (timing/1000))

while not rospy.is_shutdown():
    front_msg = TofFront()
    side_msg = TofSide()
    front_msg.front = tof.normalized()
    rospy.loginfo("sensor 1 normalized- %f" % (front_msg.front))
    side_msg.left = tof1.normalized()
    rospy.loginfo("sensor 2 normalized- %f" % (side_msg.left))
    side_msg.right = tof2.normalized()
    rospy.loginfo("sensor 3 normalized- %f" % (side_msg.right))
    
    pub_front.publish(front_msg)
    pub_side.publish(side_msg)

tof2.stop_ranging()
GPIO.output(sensor3_shutdown, GPIO.LOW)
tof1.stop_ranging()
GPIO.output(sensor2_shutdown, GPIO.LOW)
tof.stop_ranging()
GPIO.output(sensor1_shutdown, GPIO.LOW)
