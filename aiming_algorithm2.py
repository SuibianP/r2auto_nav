#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Spyder Editor

This is a temporary script file.
"""

import os  #idk what this does
#import select  #idk what this does
#import sys  #idk what this does
import rclpy  #This library contains all the classes for the nodes that we need to use with ros i think
import time  #this is for the sleep function which allows real time delays to be used
import RPi.GPIO as GPIO  #This imports the GPIO class which allows us to create GPIO objects to control the 2 servos
#from rclpy.node import Node  #This imports the Node class from the rclpy library. I'm not sure if I need both this and the import rclpy line above. 
#from rclpy.qos import qos_profile_sensor_data  #idk what this does

from geometry_msgs.msg import Twist  #This imports the Twist class. The Twist objects are what we publish to the 
from rclpy.qos import QoSProfile
from std_msgs.msg import Bool

from time import sleep

import busio
import board
import adafruit_amg88xx
import numpy as np

print('libraries imported')

'''
if os.name == 'nt': #I don't know what this does
    import msvcrt
else:
    import termios
    import tty
'''

BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = 2.84

WAFFLE_MAX_LIN_VEL = 0.26
WAFFLE_MAX_ANG_VEL = 1.82

LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.1

TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL'] 

print('velocity limits and os set')

# a couple of functions for dealing with the servo
def set_duty_cycle(angle):
    return 2.5 + (angle/18)

def move_to_angle(angle, pwm):
    duty = set_duty_cycle(angle)
    pwm.ChangeDutyCycle(duty)
    #bigger angle => move down
    #smaller angle => move up


SERVO_PIN = 12
SERVO_FREQ = 50 # PWM frequency. Don't change. Required for operation of turtlebot
SERVO_INITIAL_ANGLE = 90
SERVO_INITIAL_DUTY = set_duty_cycle(SERVO_INITIAL_ANGLE) #Sets the angle at 0 degrees initially
#SERVO_DUTY_DELTA = 2.5 #Equivalent to 45 degrees

print('servo setup complete')


def set_twist(target_linear_velocity, target_angular_velocity):        
    '''
    publisher is to be pub. I.e. the pub = node.create_publisher(Twist, 'cmd_vel', qos) nonsense
    '''
    
    #control_linear_velocity = 0.0
    #control_angular_velocity = 0.0
    
    twist = Twist()

    #control_linear_velocity = make_simple_profile(
     #   control_linear_velocity,
      #  target_linear_velocity,
       # (LIN_VEL_STEP_SIZE / 2.0))

    #twist.linear.x = float(control_linear_velocity)
    twist.linear.x = float(target_linear_velocity)    
    twist.linear.y = 0.0
    twist.linear.z = 0.0

    #control_angular_velocity = make_simple_profile(
     #   control_angular_velocity,
      #  target_angular_velocity,
       # (ANG_VEL_STEP_SIZE / 2.0))

    twist.angular.x = 0.0
    twist.angular.y = 0.0
    twist.angular.z = float(target_angular_velocity) 
    
    return twist




'''
def side_adjust(data, publisher):
    
    temps = set(data)
    cols = [data[i:63:8] for i in range(0, 8)]
    
    left = set(cols[0]+cols[1]+cols[2])
    mid = set(cols[3]+cols[4])
    right = set(cols[5]+cols[6]+cols[7])
    
    midmax = max(mid)
    leftmax = max(left)
    rightmax = max(right)
    
    bigmax = max(temps)
    
    control_linear_velocity = 0
    
    if midmax != bigmax:
        if leftmax == bigmax:
            control_angular_velocity = 1
        elif rightmax == bigmax:
            control_angular_velocity = -1
        twist = set_twist(control_linear_velocity, control_angular_velocity)
        publisher.publish(twist)        
    else:
        control_angular_velocity = 0
        twist = set_twist(control_linear_velocity, control_angular_velocity)
        publisher.publish(twist)
'''
    

def is_centered(thermal_cam):
    raw = thermal_cam.pixels
    data = np.rot90(raw, 3)
    temps = set()
    for row in data:
        temps |= set(row)
    cols = np.transpose(data)
    
    mid = set(cols[3]) | set(cols[4])
    
    midmax = max(mid)
    bigmax = max(temps)
    
    return midmax == bigmax


def side_adjust(thermal_cam, publisher):
    print('side adjust called')
    raw = thermal_cam.pixels
    data = np.rot90(raw, 3)
    temps = set()
    for row in data:
        temps |= set(row)
    cols = np.transpose(data)
    
    left = set(cols[0]) | set(cols[1]) | set(cols[2])
    #mid = set(cols[3]+cols[4])
    right = set(cols[5]) | set(cols[6]) | set(cols[7])
    
    
    #midmax = max(mid)
    leftmax = max(left)
    rightmax = max(right)
    
    bigmax = max(temps)
    
    control_linear_velocity = 0.0
    control_angular_velocity = 0.0
    target_linear_velocity = 0.0
    target_angular_velocity = 0.0
    
    while not is_centered(thermal_cam):
        
        print(left)
        print(right)
        print(temps)
        
        
        print(f'left max = {leftmax}')
        print(f'right max = {rightmax}')
        print(f'bigmax = {bigmax}')
        
        if leftmax == bigmax:
            print('left high. rotating clockwise')
            target_angular_velocity = 0.1
        elif rightmax == bigmax:
            target_angular_velocity = -0.1
            print('right high. rotating anti-clockwise')
        twist = set_twist(control_linear_velocity, target_angular_velocity)
        publisher.publish(twist)   
        print('rotation published')
        
        raw = thermal_cam.pixels
        data  = np.rot90(raw, 3)
        temps = set()
        for row in data:
            print(row)
            temps |= set(row)
        print('/n')
        cols = np.transpose(data)
        
        left = set(cols[0]) | set(cols[1]) | set(cols[2])
        #mid = set(cols[3]+cols[4])
        right = set(cols[5]) | set(cols[6]) | set(cols[7])
        
        #midmax = max(mid)
        leftmax = max(left)
        rightmax = max(right)
        
        bigmax = max(temps)
        
    stop_bot(publisher)
    print('bot stopped')


def stop_bot(publisher):
    control_linear_velocity = 0.0
    control_angular_velocity = 0.0
    twist = set_twist(control_linear_velocity, control_angular_velocity)
    publisher.publish(twist)


def distance_adust(thermal_cam, publisher, threshhold_temp):
    #data = thermal_cam.readPixels()
    
    #c4 = set(data[27:29]+data[35:37])
    #c16 = set(data[18:22]+data[26:30]+data[34:38]+data[42:46])
    
    #while min(c16) <= threshhold_temp:
    while not near_enough(thermal_cam, threshhold_temp):
        lin_vel = 0.04
        twist = set_twist(lin_vel, 0.0)
        publisher.publish(twist)
        
        #data = thermal_cam.readPixels()
        #c4 = set(data[27:29]+data[35:37])
        #c16 = set(data[18:22]+data[26:30]+data[34:38]+data[42:46])
    
    twist = set_twist(0.0, 0.0)
    
def near_enough(thermal_cam, threshold_temp):
    raw = thermal_cam.pixels
    data = np.rot90(raw, 3)
    #c16 = set()
    #c4 = set()
    #for row in data[3:5]:
     #   c4 |= set(row[3:5])
        #c16 |= set(row[2:6])
    #return min(c16)>threshold_temp
    #return max(c4)>threshold_temp
    dataset = set()
    for row in data:
        dataset |= set(row)
    return max(dataset)>threshold_temp
    

def thermal_data(thermal): #function to convert thermal data into something useful - thermal is the array that we get from the IR sensor
    data = thermal.pixels
    dataset = set()
    for row in data:
        dataset |= set(row)
    print(dataset)
    if max(dataset) >= 27:
        return 1 #return 1 when IR object is found
    return 0

def stop(thermal, tilt): #to tell ttb to stop from navigation
    print(thermal_data(thermal))
    if ((thermal_data(thermal) == 1)):#tilt is 1 will record the time at which tilt became 1 from 0 (will see later on in the firing code)
      return 1 #ttb will stop when stop() is 1
    elif (tilt == 6):
      return 0 #ttb can continue moving as long as != 1
    else:
      return 0 #ttb can continue moving as long as != 1
  
    '''
    stop function to publish stop instruction to nav-shoot topic
    nav-shoot topic to be named "shoot"
    prefrerably boolean
    1/True => nav to stop
    0/False => nav to start again
    '''


def angle (thermal, tilt):
    angle = 90
    if stop(thermal, tilt) == 1:
      if ((tilt == 0)):
        tilt = 1
      elif ((tilt == 1) or (tilt == 0)): #default angle is 90
        angle = 85 
      elif (tilt == 2):
        tilt = 3 #additional incrementations are made here to delay the time as much as possible between each consecutive shot 
      elif (tilt == 3):
        angle = 95
      elif (tilt == 4):
        tilt = 5
      elif (tilt == 5):
        angle = 105 
      elif (tilt ==6):
        angle = 90
    return angle

def set_duty_cycle_2(thermal,tilt):
    return 2.5 + (angle(thermal,tilt)/18)

def move_to_angle_2(pwm, thermal, tilt):
  duty = set_duty_cycle_2(thermal, tilt)
  pwm.ChangeDutyCycle(duty)
  #bigger angle => move down
  #smaller angle => move up
 
    
def firing_mech(p_tilt, angle, tilt, thermal, p_servo ,SERVO_INITIAL_DUTY):
    print('Shooting')
    p_tilt.start(set_duty_cycle_2(thermal, tilt))
    print('Starting angle set')
    
    while (tilt != 6):
      if (tilt == 0):
        move_to_angle_2(p_tilt, thermal, tilt)
        tim = time.time()
        tilt = 1
        print('Tilt set to 1')
        
      elif (tilt == 1):
          move_to_angle_2(p_tilt, thermal, tilt)
          p_servo.start(SERVO_INITIAL_DUTY)
          p_servo.stop()
          time1 = time.time() - tim
          print(f'Tilt stage: {tilt}')
          if ((time1 >= 2.0) and (time1 < 3.4)):
            #2 second delay, let's predict that 1 revolution is 1.6 second
            p_servo.start(SERVO_INITIAL_DUTY)
            print('Servo moving')
          elif ((time1 >= 3.4) and (time1 < 5.5)): #servo stops
            p_servo.stop()
            print('servo stopped')
          elif ((time1 >= 5.5) and (time1 < 6.0)):
            print('time between 4.6 and 5s')
            tilt = 3
          
      elif (tilt == 3):
        print(f'Tilt stage: {tilt}')
        move_to_angle_2(p_tilt, thermal, tilt)
        print('Angle set to 95 degrees')
        time1 = time.time() - tim
        if ((time1 >= 8.0) and (time1 < 9.4)): #3 second delay to tilt the barrel to the next angle
            p_servo.start(SERVO_INITIAL_DUTY)
            print('Servo moving')
        if ((time1 >= 9.4) and (time1 < 10.0)):
            p_servo.stop()
            print('Servo stopped')
        if ((time1 >= 10.0) and (time1 < 10.5)):
            tilt = 5
          
      elif (tilt == 5):
        print(f'Tilt stage: {tilt}')
        time1 = time.time() - tim
        move_to_angle_2(p_tilt, thermal, tilt)
        print('Angle set to 100 degrees')
        if ((time1 >= 14.0) and (time1 < 15.4)): #3 second delay to tilt the barrel to the next angle
            p_servo.start(SERVO_INITIAL_DUTY)
            print('Servo moving')
        if ((time1 >= 15.4) and (time1 < 16.0)):
            p_servo.stop()
            print('Servo stopped')
        if (time1 >= 16.0):
            tilt = 6
    return 1 #no longer any need for this once all 3 shots have been fired
   
print('functions defined')


def main():
    print("main function starting")
    rclpy.init()

    qos = QoSProfile(depth=10)
    aim_node = rclpy.create_node('aiming_algorithm') #I think that I also need to create a listener/subscriber for the 
    vel_pub = aim_node.create_publisher(Twist, 'cmd_vel', qos)
    to_nav = aim_node.create_publisher(Bool, 'shoot', qos)
    
    i2c = busio.I2C(board.SCL, board.SDA)
    cam = adafruit_amg88xx.AMG88XX(i2c)
    scanning_threshold = 27
    distancing_threshold = 32
    
    control_linear_velocity = 0.0
    control_angular_velocity = 0.0
    
    #status = 0
    #target_linear_velocity = 0.0
    #target_angular_velocity = 0.0
    #control_linear_velocity = 0.0
    #control_angular_velocity = 0.0
    
    SERVO_PIN_TILT = 21 #tilt servo
    SERVO_PIN_SERVO = 20 #continous servo
    PIN_DC = 16 #DC motor 
    SERVO_FREQ = 50 # PWM frequency
    thermal = cam.pixels
    SERVO_INITIAL_DUTY = set_duty_cycle_2(cam, 0)
    
    # GPIO initialisation
    GPIO.setmode(GPIO.BCM) # BCM pin notation
    GPIO.setup(SERVO_PIN_TILT, GPIO.OUT)
    GPIO.setup(SERVO_PIN_SERVO, GPIO.OUT)
    GPIO.setup(PIN_DC, GPIO.OUT)
    p_tilt = GPIO.PWM(SERVO_PIN_TILT ,SERVO_FREQ) # PWM object for tilting servo
    p_servo = GPIO.PWM(SERVO_PIN_SERVO, SERVO_FREQ) # PWM object for plunger servo
    p_dc = GPIO.PWM (PIN_DC, SERVO_FREQ) ####### Apparently can still run non-pwm with pwm code
    
    tilt = 0
    print('setup complete')
    
    try:
        while(1):
            
            print('main loop starting')
            
            thermal_data(cam)
            if stop(cam, tilt):
                to_nav.publish(True)
                sleep(3) #how long will jialun's portion take to suspend?
                print('have yet to stop')
                while not is_centered(cam):
                    print('adjusting to centre')
                    side_adjust(cam, vel_pub)
                print('bot stopping')
                stop_bot(vel_pub)
                print('centered')
                sleep(3)
                
                while not near_enough(cam, distancing_threshold):
                    print('spacing')
                    distance_adust(cam, vel_pub, distancing_threshold) #remember to leave the 
                stop_bot(vel_pub)
                print('spaced')
                
                while not is_centered(cam):
                    print('adjusting to centre 2')
                    side_adjust(cam, vel_pub)
                    print('centered 2')
                print('bot stopping')
                stop_bot(vel_pub)

            
                print('Bot stopped')
                #stop(thermal, tilt)
                angle(cam,tilt)
                #move_to_angle_2(p_tilt,thermal,tilt)
                shooting_completed = firing_mech(p_tilt, angle, tilt, cam, p_servo, SERVO_INITIAL_DUTY)
                if shooting_completed:
                    to_nav.publish(False)
                    break



    except Exception as e:
        print(e)

    finally:
        #at the end of the code, the robot should stop moving
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0

        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0

        vel_pub.publish(twist)



if __name__ == '__main__':
    main()

