
import RPi.GPIO as GPIO

PIN = 4

GPIO.setmode(GPIO.BCM) 



try: 
    while True: 
        value = GPIO.input(PIN) 
        fp.write( str(value) + '\n' ) 
except(KeyboardInterrupt): 
    fp.close() 
    GPIO.cleanup() 
    print('end\n')
