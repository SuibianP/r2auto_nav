#!/usr/bin/env python3

"""
Created on Mon Apr 12 15:39:33 2021

@author: jfoo
"""

import RPi.GPIO as GPIO
from time import sleep

def main():
    PIN_DC = 16 #DC motor 

    GPIO.setmode(GPIO.BCM) # BCM pin notation
    GPIO.setup(PIN_DC, GPIO.OUT)
    #p_dc = GPIO.PWM (PIN_DC, 50) ####### Apparently can still run non-pwm with pwm code
    print('setup complete')
    try:
        while True:
            GPIO.output(PIN_DC, GPIO.HIGH)
            #p_dc.start(100)
            print("Motor on")
            sleep(5)
            GPIO.output(PIN_DC, GPIO.LOW)
            #p_dc.stop()
            print('Motor off')
            sleep(5)
    except Exception as e:
        print(e)
    finally:
        GPIO.cleanup()
    
main()
