#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Apr 13 02:02:20 2021

@author: jfoo
"""

import RPi.GPIO as GPIO
from time import sleep

def main():
    SERVO_INITIAL_DUTY = 2.5
    SERVO_PIN_SERVO = 20 #continous servo
    SERVO_FREQ = 50 # PWM frequency
    
    GPIO.setmode(GPIO.BCM) # BCM pin notation
    GPIO.setup(SERVO_PIN_SERVO, GPIO.OUT)
    p_servo = GPIO.PWM(SERVO_PIN_SERVO, SERVO_FREQ) # PWM object for plunger servo
    
    
    try:
        while True:
            p_servo.start(SERVO_INITIAL_DUTY)
            sleep(5)
            p_servo.stop()
            sleep(3)
    except Exception as e:
        print(e)
    finally:
        p_servo.stop()
        GPIO.cleanup()
    
main()