#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Spyder Editor

This is a temporary script file.
"""

import os
import rclpy               #Imports the classes for the nodes required to work with ROS 
import time                #Imports the time() function which is used to determine when to angle and shoot the balls
import RPi.GPIO as GPIO              #This imports the GPIO class which allows us to create GPIO objects to control the 2 servos

from geometry_msgs.msg import Twist  #This imports the Twist class which is published to the 'cmd_vel' topic to move the turtlebot 
from rclpy.qos import QoSProfile 
from std_msgs.msg import Bool        #This bool object is used to communicate with the navigation algorithm to determine when navigation should release controls to aiming and vice versa

from time import sleep 

import busio                #busio and board libraries are used for i2c communication on the rpi
import board 
import adafruit_amg88xx     #This is the required library for operating the AMG8833 thermal camera
import numpy as np          #A few functions from numpy were used to make computation easier

print('libraries imported')


BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = 2.84

WAFFLE_MAX_LIN_VEL = 0.26
WAFFLE_MAX_ANG_VEL = 1.82

LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.1

TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL'] 

print('Velocity limits and os set')


def set_duty_cycle(angle):
    '''Sets the required duty cycle for the tilting servo based on the target angle'''
    return 2.5 + (angle/18)

def move_to_angle(angle, pwm):
    '''
    Moves the tiling servo to the target angle.
    Takes in the desired angle and the pwm object for the tilting servo GPIO pins. 
    bigger angle => move down
    smaller angle => move up
    '''
    duty = set_duty_cycle(angle)
    pwm.ChangeDutyCycle(duty)



def set_twist(target_linear_velocity, target_angular_velocity):        
    '''
    Parameters
    ----------
    target_linear_velocity : INT or FLOAT
        Specifies the forward and backward velocity of the turtlebot    
        
    target_angular_velocity : INT or FLOAT
        Specifies the clockwise or anti-clockwise angular velocity of the turtlebot    

    Returns
    -------
    twist : TWIST OBJECT
        Creates a twist object with the specified linear and angular velocity 
        that will be published to the 'cmd_vel' topic to move the turtlebot
    '''
    
    twist = Twist()

    twist.linear.x = float(target_linear_velocity)    
    twist.linear.y = 0.0
    twist.linear.z = 0.0

    twist.angular.x = 0.0
    twist.angular.y = 0.0
    twist.angular.z = float(target_angular_velocity) 
    
    return twist
    

def is_centered(thermal_cam):
    '''
    Checks if the turtlebot is facing the heat source straight on
    by checking if the highest temperature is detected by the center of the camera
    
    Parameters
    ----------
    thermal_cam : AMG88XX object from adafruit_amg88xx library
        The .pixels (2D array) attribute is accessed from the thermal camera object
        The array is used to determine if the highest temperature in the array is from the center two rows or not

    Returns
    -------
    Booleann
        Returns True if the highest temperature in the array is from the middle two rows
        Otherwise, returns False
    '''
    
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
    '''
    Rotates the turtlebot clockwise and anti-clockwise until it faces the heat source straight on

    Parameters
    ----------
    thermal_cam : AMG88XX object from adafruit_amg88xx library
        The .pixels (2D array) attribute is accessed from the thermal camera object.
        The array is used to determine if the highest temperature in the array is from 
        the left three columns or the right three columns and determine the direction of
        rotation of the turtlebot accordingly.
        
    publisher : Publisher object from the aiming node
        The publisher publishes the Twist object to the 'cmd_vel' topic to rotate the turtlebot

    Returns
    -------
    None.

    '''

    print('side adjust called')
    raw = thermal_cam.pixels
    data = np.rot90(raw, 3)
    temps = set()
    for row in data:
        temps |= set(row)
    cols = np.transpose(data)
    
    left = set(cols[0]) | set(cols[1]) | set(cols[2])
    right = set(cols[5]) | set(cols[6]) | set(cols[7])
    
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
        right = set(cols[5]) | set(cols[6]) | set(cols[7])
        
        leftmax = max(left)
        rightmax = max(right)
        
        bigmax = max(temps)
        
    stop_bot(publisher)
    print('bot stopped')


def stop_bot(publisher):
    '''
    Stops the turtlebot
    
    Parameters
    ----------
    publisher : Publisher object from the aiming node
        The publisher publishes the Twist object to the 'cmd_vel' topic to stop the turtlebot

    Returns
    -------
    None.
    '''
    control_linear_velocity = 0.0
    control_angular_velocity = 0.0
    twist = set_twist(control_linear_velocity, control_angular_velocity)
    publisher.publish(twist)


def distance_adust(thermal_cam, publisher, space_temp, scan_temp):
    '''
    Moves the turtlebot forwards until it is at an appropriate distance from the target
    
    Parameters
    ----------
    thermal_cam : AMG88XX object from adafruit_amg88xx library
        The .pixels (2D array) attribute is accessed from the thermal camera object.
        The array is used to determine if the highest temperature in the array is
        greater than the space_temp or less than the scan_temp. 
        If the highest temp is greater than the space_temp, the bot stops so as not to get too near to the target
        If the highest temp is less than the scan_temp, it likely mean that the target is out of the detection range;
        Assuming that the target is out of the detection range because it is too high to detect, the bot should stop
        because it is likely at a close enough distance to be able to hit the target
        
    publisher : Publisher object from the aiming node
        The publisher publishes the Twist object to the 'cmd_vel' topic to move the turtlebot forwards
        
    space_temp : int or float
        If the highest temperature in the array is greater than this user-specified temperature,
        the bot will stop so that it doesn't get too close to the target
        
    scan_temp : int or float
        This is the user-defined threshhold temperature when background scanning for the target.
        If the highest temperature in the array is less than this temperature, it likely means 
        that the turtlebot is too close to the target to detect it 
        (because the thermal camera is mounted on the front of the turtlebot, 
         which may be below the target if the target is elevated)

    Returns
    -------
    None.
    '''

    while not near_enough(thermal_cam, space_temp, scan_temp):
        lin_vel = 0.04
        twist = set_twist(lin_vel, 0.0)
        publisher.publish(twist)
    
    stop_bot(publisher)
    

def near_enough(thermal_cam, space_temp, scan_temp):
    '''
    Checks if the turtlebot is an appropriate distance from the target to commence shooting
    
    Parameters
    ----------
    thermal_cam : AMG88XX object from adafruit_amg88xx library
        The .pixels (2D array) attribute is accessed from the thermal camera object.
        The array is used to determine if the highest temperature in the array is
        greater than the space_temp or less than the scan_temp.
        
    space_temp : int or float
        If the highest temperature in the array is greater than this user-specified temperature, True will be returned
        
    scan_temp : int or float
        This is the user-defined threshhold temperature when background scanning for the target.
        If the highest temperature in the array is less than this temperature, it likely means 
        that the turtlebot is too close to the target to detect it and True will be returned
        (because the thermal camera is mounted on the front of the turtlebot, 
         which may be below the target if the target is elevated)

    Returns
    -------
    Boolean
        Returns True if the turtlebot is appropriately spaced from the target
        Otherwise, returns False. 
    '''
    
    raw = thermal_cam.pixels
    data = np.rot90(raw, 3)
    dataset = set()
    for row in data:
        dataset |= set(row)
    highest = max(dataset)
    return highest > space_temp or highest < scan_temp



def thermal_data(thermal): 
    '''
    Parameters
    ----------
    thermal : AMG88XX object from adafruit_amg88xx library
        The .pixels (2D array) attribute is accessed from the thermal camera object.
        The array is used to determine if the highest temperature in the array is
        greater than 29 deg cel 
        (29 deg cel is actually the scan_temp, or background scanning temperature. 
         It should have been specified as an input for the function, but because 
         this function is referenced by the other functions further down, 
         there was insufficient time to change it before the final run)

    Returns
    -------
    int (1 or 0)
        Returns 1 if the maximum temperature in the array is greater than 29 deg cel
        Otherwise, returns 0
    '''
    
    data = thermal.pixels
    dataset = set()
    for row in data:
        dataset |= set(row)
    print(dataset)
    if max(dataset) >= 29:
        return 1    #return 1 when IR object is found
    return 0



def stop(thermal, tilt): 
    '''
    Determines when the turtlebot should stop from navigation
    
    Parameters
    ----------
    thermal : AMG88XX object from adafruit_amg88xx library
        Used as an argument for the thermal_data(thermal) function
        
    tilt : int
        tilt is the current phase of shooting that the bot is in.
        It is modified by the angle function. 

    Returns
    -------
    int (1 or 0)
        If the target is detected, 1 will be returned. 
        Otherwise, if the final shooting phase has been reached (phase 6), 0 will be returned. 
        Otherwise, 0 will be returned. 
    '''
    
    if ((thermal_data(thermal) == 1)): 
      return 1 #ttb will stop when stop() is 1
    elif (tilt == 6):
      return 0 #ttb can continue moving as long as != 1
    else:
      return 0 #ttb can continue moving as long as != 1
  


def angle (thermal, tilt):
    '''
    Sets the desired tilt angle based on the current tilt phase
    
    Parameters
    ----------
    thermal : AMG88XX object from adafruit_amg88xx library
        Used as an argument for the stop(thermal, tilt) function
        
    tilt : int
        tilt is the current phase of shooting that the bot is in.

    Returns
    -------
    angle : int
        angle is the desired tilt angle to be returned.
        This is to be passed to the set_duty_cycle_2 function to move the shooter to the desired angle
    '''
    angle = 90
    if stop(thermal, tilt) == 1:
      if ((tilt == 0)):
        tilt = 1
      elif ((tilt == 1) or (tilt == 0)): #default angle is 90
        angle = 65 
      elif (tilt == 2):
        tilt = 3 #additional incrementations are made here to delay the time as much as possible between each consecutive shot 
      elif (tilt == 3):
        angle = 75
      elif (tilt == 4):
        tilt = 5
      elif (tilt == 5):
        angle = 85
      elif (tilt ==6):
        angle = 90
    return angle



def set_duty_cycle_2(thermal,tilt):
    '''
    Returns the duty cycle required in order to achieve the angle specified by the current tilt phase
    
    Parameters
    ----------
    thermal : AMG88XX object from adafruit_amg88xx library
        Used as an argument for the angle(thermal, tilt) function
        
    tilt : int
        tilt is the current phase of shooting that the bot is in.
        Used as an argument for the angle(thermal, tilt) function

    Returns
    -------
    float
        The reqiured duty cycle in order to move to the specified angle
    '''
    return 2.5 + (angle(thermal,tilt)/18)



def move_to_angle_2(pwm, thermal, tilt):
    '''
    Moves the tilting servo to the angle required for the current tilt phase
    Bigger angle => move down
    Smaller angle => move up
    
    Parameters
    ----------
    pwm : pwm object from RPi.GPIO library
        This is to be the tilting servo's pwm object based on the GPIO pin connected to it
        The duty cycle of this object will be changed to move the servo to the correct angle
        
    thermal : AMG88XX object from adafruit_amg88xx library
        Used as an argument for the set_duty_cycle_2(thermal, tilt) function
        
    tilt : int
        tilt is the current phase of shooting that the bot is in.
        Used as an argument for the set_duty_cycle_2(thermal, tilt) function

    Returns
    -------
    None.
    '''
    
    duty = set_duty_cycle_2(thermal, tilt)
    pwm.ChangeDutyCycle(duty)

 

def firing_mech(p_tilt, tilt, thermal, p_servo ,SERVO_INITIAL_DUTY):
    '''
    This is the main shooting algorithm which is to be carried out after centering and distancing have been carried out.
    The tilting servo will be started at 90 degrees (horizontal)
    The bot will tilt to 65 degrees (elevation of 25 degrees) and shoot
    Then the bot will tilt to 75 degrees (elevation of 15 degrees) and shoot
    Finally, the bot will tilt to 85 degreed (elevation of 5 degrees) and shoot
    1 will be returned to indicate that shooting has completed.
    
    Parameters
    ----------
    p_tilt : pwm object from RPi.GPIO library
        The duty cycle of p_tilt is changed according to the required angle
        
    angle : int
        DESCRIPTION.
        
    tilt : TYPE
        DESCRIPTION.
    thermal : TYPE
        DESCRIPTION.
    p_servo : TYPE
        DESCRIPTION.
    SERVO_INITIAL_DUTY : TYPE
        DESCRIPTION.

    Returns
    -------
    TYPE
        DESCRIPTION.

    '''
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
          p_servo.start(SERVO_INITIAL_DUTY) #For seome reason, starting and stopping immediately help with timing later down the line
          p_servo.stop()
          time1 = time.time() - tim
          print(f'Tilt stage: {tilt}')
          if ((time1 >= 2.0) and (time1 < 3.5)):
            p_servo.start(SERVO_INITIAL_DUTY)
            print('Servo moving')
          elif ((time1 >= 3.5) and (time1 < 4.0)): #servo stops
            p_servo.stop()
            print('servo stopped')
          elif ((time1 >= 4.0) and (time1 < 5.0)):
            print('time between 4.6 and 5s')
            tilt = 3
          
      elif (tilt == 3):
        print(f'Tilt stage: {tilt}')
        move_to_angle_2(p_tilt, thermal, tilt)
        print('Angle set to 95 degrees')
        time1 = time.time() - tim
        if ((time1 >= 8.0) and (time1 < 9.5)): #3 second delay to tilt the barrel to the next angle
            p_servo.start(SERVO_INITIAL_DUTY)
            print('Servo moving')
        if ((time1 >= 9.5) and (time1 < 10.0)):
            p_servo.stop()
            print('Servo stopped')
        if ((time1 >= 10.0) and (time1 < 10.5)):
            tilt = 5
          
      elif (tilt == 5):
        print(f'Tilt stage: {tilt}')
        time1 = time.time() - tim
        move_to_angle_2(p_tilt, thermal, tilt)
        print('Angle set to 100 degrees')
        if ((time1 >= 14.0) and (time1 < 15.5)): #3 second delay to tilt the barrel to the next angle
            p_servo.start(SERVO_INITIAL_DUTY)
            print('Servo moving')
        if ((time1 >= 15.5) and (time1 < 16.0)):
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
    scanning_threshold = 28
    distancing_threshold = 32
    
    SERVO_PIN_TILT = 21 #tilt servo
    SERVO_PIN_SERVO = 20 #continous servo
    PIN_DC = 16 #DC motor 
    SERVO_FREQ = 50 # PWM frequency
    SERVO_INITIAL_DUTY = set_duty_cycle_2(cam, 0)
    
    # GPIO initialisation
    GPIO.setmode(GPIO.BCM) # BCM pin notation
    GPIO.setup(SERVO_PIN_TILT, GPIO.OUT)
    GPIO.setup(SERVO_PIN_SERVO, GPIO.OUT)
    GPIO.setup(PIN_DC, GPIO.OUT)
    p_tilt = GPIO.PWM(SERVO_PIN_TILT ,SERVO_FREQ) # PWM object for tilting servo
    p_servo = GPIO.PWM(SERVO_PIN_SERVO, SERVO_FREQ) # PWM object for plunger servo
    
    tilt = 0
    print('setup complete')
    
    try:
        while(1):
            
            print('main loop starting')
            
            thermal_data(cam)
            if stop(cam, tilt):
                b = Bool()
                b.data=True
                to_nav.publish(b)
                
                sleep(3) #how long will jialun's portion take to suspend?
                print('have yet to stop')
                while not is_centered(cam):
                    print('adjusting to centre')
                    side_adjust(cam, vel_pub)
                print('bot stopping')
                stop_bot(vel_pub)
                print('centered')
                sleep(3)
                
                while not near_enough(cam, distancing_threshold, scanning_threshold):
                    print('spacing')
                    distance_adust(cam, vel_pub, distancing_threshold, scanning_threshold) #remember to leave the 
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
                shooting_completed = firing_mech(p_tilt, tilt, cam, p_servo, SERVO_INITIAL_DUTY)
                if shooting_completed:
                    b.data=False
                    to_nav.publish(b) #is this supposed to be False or True?
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

