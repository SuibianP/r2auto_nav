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
        pitch_angle = int(input('please enter a pitching angle between 100 and 70: '))
        
        move_to_angle(pitch_angle, p)
        time.sleep(1)
    
        
except KeyboardInterrupt:
    pass

finally:
    p.stop()
    GPIO.cleanup()