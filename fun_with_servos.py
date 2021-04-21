#!/usr/bin/env python3

import time
import RPi.GPIO as GPIO

def set_duty_cycle(angle):
    return 2.5 + (angle/18)

def move_to_angle(angle, pwm):
    duty = set_duty_cycle(angle)
    pwm.ChangeDutyCycle(duty)
    #bigger angle => move down
    #smaller angle => move up

# sort-of constants
SERVO_PIN = 21
SERVO_FREQ = 50 # PWM frequency
SERVO_INITIAL_DUTY = set_duty_cycle(90)

# GPIO initialisation
GPIO.setmode(GPIO.BCM) # BCM pin notation
GPIO.setup(SERVO_PIN, GPIO.OUT)
p = GPIO.PWM(SERVO_PIN, SERVO_FREQ) # PWM object
p.start(SERVO_INITIAL_DUTY)

    
print('setup complete')

time.sleep(5)

try:
    print('main loop starting')
    time.sleep(1)
    while True:
        print('loop started')
        
        angle = int(input('Enter an angle between 70 and 110: '))
        move_to_angle(angle, p)
        print("angle changed.")
        
        
        '''
        move_to_angle(90, p)
        print('angle is 90')
        time.sleep(1)
        
        move_to_angle(100, p)
        print('angle is 100')
        time.sleep(1)
        
        move_to_angle(80, p)
        print('angle is 80')
        time.sleep(1)
        
        move_to_angle(90, p)
        print('angle is 90. you have 20s to change position of the horn')
        time.sleep(20)
        
        
        
        move_to_angle(135, p)
        print('angle is 135')
        time.sleep(1)
        
        move_to_angle(180, p)
        print('angle is 180')
        time.sleep(5)
        '''
        
except KeyboardInterrupt:
    pass
finally:
    p.stop()
    GPIO.cleanup()