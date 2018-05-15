#!/usr/bin/python

import RPi.GPIO as GPIO
import time

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

usleep = lambda x: time.sleep(x/(1000*1000.0))


while True:
    
    try:
        
        count = 0

        print('a')
        
        GPIO.setup(4, GPIO.OUT)

        print('b')

        GPIO.output(4, 1)

        print('c')
    
        usleep(10)

        print('d')

        # GPIO.output(4, 0)
        
        print('e')

        GPIO.setup(4, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        print('f')

        
        while GPIO.input(4) == 1:
            count += 1

            print('h')
            
        print(count)

        while GPIO.input(4) == 0:

            print("count:{}".format(count))
        
            
    except KeyboardInterrupt:
        GPIO.cleanup()
        exit(0)
        


