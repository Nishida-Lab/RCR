#!/usr/bin/env python3
def main():
    pwm_pin = 18
    dir_pin = 17
    pi = pigpio.pi()
    pi.set_mode(pwm_pin, pigpio.OUTPUT)
    pi.set_mode(dir_pin, pigpio.OUTPUT)
    pi.write(dir_pin, 0)
    pi.hardware_PWM(pwm_pin, 100, 300000)

    try:
        while True:
            pass
    except KeyboardInterrupt:
        pi.set_mode(pwm_pin, pigpio.INPUT)
        pi.set_mode(dir_pin, pigpio.INPUT)
        pi.stop()
        exit(0)

if __name__ == '__main__':
    import pigpio

    main()
 