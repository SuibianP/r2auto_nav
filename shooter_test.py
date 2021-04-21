#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Apr 12 13:28:19 2021

@author: jfoo
"""

import RPi.GPIO as GPIO
import time


def set_duty_cycle_2(angle):
    return 2.5 + (angle/18)

def move_to_angle_2(pwm, angle):
  duty = 2.5 + (angle/18)
  pwm.ChangeDutyCycle(duty)
  #bigger angle => move down
  #smaller angle => move up
  
def firing_mech(p_tilt, angle, tilt, p_servo ,SERVO_INITIAL_DUTY):
    p_tilt.start(set_duty_cycle_2(90))
    while (tilt != 6):       
      if (tilt == 0):
        move_to_angle_2(p_tilt, 90)
        tim = time.time()
        tilt = 1
        
      elif (tilt == 1):
        time1 = time.time() - tim
        if ((time1 >= 2.0) and (time1 < 3.6)):
            #2 second delay, let's predict that 1 revolution is 0.8 second
            print('time between 2 and 3.6s')
            p_servo.start(SERVO_INITIAL_DUTY)
        elif ((time1 >= 3.6) and (time1 < 3.8)):
            print('time between 3.6 and 4.6s')#servo stops
            p_servo.stop()
        elif ((time1 >= 4.0) and (time1 < 5.0)):
            print('time between 4.6 adn 5s')
            tilt = 3
          
      elif (tilt == 3):
        print('hi2')
        move_to_angle_2(p_tilt, 95)
        time1 = time.time() - tim
        if ((time1 >= 5.0) and (time1 < 6.6)): #3 second delay to tilt the barrel to the next angle
            p_servo.start(SERVO_INITIAL_DUTY)
        if ((time1 >= 6.6) and (time1 < 7.6)):
            p_servo.stop()
        if ((time1 >= 10.8) and (time1 < 11.0)):
            tilt = 5
          
      elif (tilt == 5):
        print('hi3')
        time1 = time.time() - tim
        move_to_angle_2(p_tilt, 100)
        if ((time1 >= 11.0) and (time1 < 12.6)): #3 second delay to tilt the barrel to the next angle
            p_servo.start(SERVO_INITIAL_DUTY)
        if ((time1 >= 12.6) and (time1 < 12.8)):
            p_servo.stop()
        if (time1 >= 12.8):
            tilt = 6
    return 0 #no longer any need for this once all 3 shots have been fired


def main():
    SERVO_INITIAL_DUTY = 2.5
    SERVO_PIN_TILT = 21 #tilt servo
    SERVO_PIN_SERVO = 20 #continous servo
    SERVO_FREQ = 50 # PWM frequency
    
    GPIO.setmode(GPIO.BCM) # BCM pin notation
    GPIO.setup(SERVO_PIN_TILT, GPIO.OUT)
    GPIO.setup(SERVO_PIN_SERVO, GPIO.OUT)
    p_tilt = GPIO.PWM(SERVO_PIN_TILT ,SERVO_FREQ) # PWM object for tilting servo
    p_servo = GPIO.PWM(SERVO_PIN_SERVO, SERVO_FREQ) # PWM object for plunger servo
    
    tilt = 0
    angle = 90
    try:
        while True:
            firing_mech(p_tilt, angle, tilt, p_servo, SERVO_INITIAL_DUTY)
            print('jareth')
    except Exception as e:
        print(e)
    finally:
        p_tilt.stop()
        p_servo.stop()
        GPIO.cleanup()
    
main()