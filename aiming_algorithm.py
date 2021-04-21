#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Spyder Editor

This is a temporary script file.
"""

import os  #idk what this does
import rclpy  #This library contains all the classes for the nodes that we need to use with ros i think
import time  #this is for the sleep function which allows real time delays to be used
import RPi.GPIO as GPIO  #This imports the GPIO class which allows us to create GPIO objects to control the 2 servos

from geometry_msgs.msg import Twist  #This imports the Twist class. The Twist objects are what we publish to the 
from rclpy.qos import QoSProfile

from Adafruit_AMG88xx import Adafruit_AMG88xx
from time import sleep

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

'''
class Scanner(Node):
    def __init__(self):
        super().__init__('scanner')
        self.subscription = self.create_subscription(
            LaserScan, # message definition
            'scan', # topic name
            self.listener_callback, # callback function
            qos_profile_sensor_data) # quality of service profile
        self.servo_duty = 2.5 # initial duty cycle, in percentage
        self.delta_direction = 1

    # Every new message in scan topic will invoke this function
    def listener_callback(self, msg):
        if msg.range[0] != 1: # TODO: we may need to allow some epsilon here
            return # to reduce indentation level
        self.servo_duty += SERVO_DUTY_DELTA * self.delta_direction
        if (self.servo_duty > 10) or (self.servo_duty < 2.5):
            self.delta_direction = -self.delta_direction
        p.ChangeDutyCycle(self.servo_duty)
        
'''




'''
def make_simple_profile(output, input, slop):
    if input > output:
        output = min(input, output + slop)
    elif input < output:
        output = max(input, output - slop)
    else:
        output = input
    return output
'''


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
    print('is_centered called')
    data = thermal_cam.readPixels()
    temps = set(data)
    cols = [data[i:63:8] for i in range(0, 8)]
    
    mid = set(cols[3]+cols[4])
    
    midmax = max(mid)
    bigmax = max(temps)
    
    return midmax == bigmax


def side_adjust(thermal_cam, publisher):
    print('side adjust called')
    data = thermal_cam.readPixels()
    
    temps = set(data)
    cols = [data[i:63:8] for i in range(0, 8)]
    
    left = set(cols[0]+cols[1]+cols[2])
    #mid = set(cols[3]+cols[4])
    right = set(cols[5]+cols[6]+cols[7])
    
    #midmax = max(mid)
    leftmax = max(left)
    rightmax = max(right)
    
    bigmax = max(temps)
    
    control_linear_velocity = 0.0
    control_angular_velocity = 0.0
    
    print('data processed')
    
    while not is_centered(thermal_cam):
        if leftmax == bigmax:
            target_angular_velocity = 0.4
        elif rightmax == bigmax:
            target_angular_velocity = -0.4
        twist = set_twist(control_linear_velocity, target_angular_velocity)
        publisher.publish(twist)   
        
    stop_bot(publisher)


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
    while not spaced_well(thermal_cam, threshhold_temp):
        lin_vel = 0.1
        twist = set_twist(lin_vel, 0.0)
        publisher.publish(twist)
        
        #data = thermal_cam.readPixels()
        #c4 = set(data[27:29]+data[35:37])
        #c16 = set(data[18:22]+data[26:30]+data[34:38]+data[42:46])
    
    twist = set_twist(0.0, 0.0)
    
def spaced_well(thermal_cam, threshold_temp):
    data = thermal_cam.readPixels()
    c16 = set(data[18:22]+data[26:30]+data[34:38]+data[42:46])
    return min(c16)>threshold_temp
    
    
        
    
    
        
'''    
def height_adjust(data, servo):
    GPIO.setmode(GPIO.BCM) # BCM pin notation
    GPIO.setup(SERVO_PIN, GPIO.OUT)    
    p = GPIO.PWM(SERVO_PIN, SERVO_FREQ) # PWM object
    
    
 '''


    
def thermal_data(thermal): #function to convert thermal data into something useful - thermal is the array that we get from the IR sensor
    final_lst = [] #the eventual 2D array
    lst = [] #deep array
    for i in thermal:
      if ((((thermal.index(i))%8) == 0) and (thermal.index(i) != 0)): #using multiples of 8 to separate 1D array into 2D (rows of 8) arraw
        lst = lst + [i] 
        final_lst = final_lst + [lst] #add a deep array into the 2D array
        lst = [] #clear the deep array once the 8th element has been added to the 2D array
      else:
        lst = lst + [i] #add an element into the deep array when less than 8 elements are in it
    for j in range (2,6): #checking for the thermal intensity in the centre
      for k in range(0,8): #checking each elemnt in the range is above a certain level (30)
        if (final_lst[j][k] < 30):
          return 0 #return 1 when IR object is found
    return 1


def stop(thermal, tilt): #to tell ttb to stop from navigation
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
    True => nav to stop
    False => nav to start again
    '''


def angle (thermal, tilt):
    if stop(thermal, tilt) == 1:
      if ((tilt == 0)):
        tilt = 1
      elif ((tilt == 1) or (tilt == 0)): #default angle is 90
        angle = 90 
      elif (tilt == 2):
        tilt = 3 #additional incrementations are made here to delay the time as much as possible between each consecutive shot 
      elif (tilt == 3):
        angle = 80
      elif (tilt == 4):
        tilt = 5
      elif (tilt == 5):
        angle = 70 
      elif (tilt ==6):
        angle = 90
    else:
      angle = 90 
    return angle

def set_duty_cycle_2(thermal,tilt):
    return 2.5 + (angle(thermal,tilt)/18)

def move_to_angle_2(pwm, thermal, tilt):
  duty = set_duty_cycle_2(thermal, tilt)
  pwm.ChangeDutyCycle(duty)
  #bigger angle => move down
  #smaller angle => move up
 


def firing_mech(p_tilt, tilt, thermal, p_servo, p_dc ,SERVO_INITIAL_DUTY):
  while True:
    if (thermal_data(thermal) == 1):
      move_to_angle_2(p_tilt, thermal, tilt)
      if (tilt == 0):
        tim = time.time() #once tilt is more than 0, tim is a fixed constant
      elif (tilt == 1):
        time1 = time.time() - tim
        if ((time1 >= 2) and (time1 < 2.8)): #2 second delay, let's predict that 1 revolution is 0.8 second
          p_dc.start(SERVO_INITIAL_DUTY)
          p_servo.start(SERVO_INITIAL_DUTY)
        if ((time1 >= 2.8) and (time1 < 3.8)): #servo stops
          p_servo.stop()
        if ((time1 >= 3.8) and (time1 < 4.8)): #dc motor stops
          p_dc.stop()
        if (time >= 4.8):
          tilt = 2
      elif (tilt == 3):
        time1 = time.time() - tim
        if ((time1 >= 8) and (time1 < 8.8)): #3 second delay to tilt the barrel to the next angle
          p_dc.start(SERVO_INITIAL_DUTY)
          p_servo.start(SERVO_INITIAL_DUTY)
        if ((time1 >= 8.8) and (time1 < 9.8)):
          p_servo.stop()
        if ((time1 >= 9.8) and (time1 < 10.8)):
          p_dc.stop()
        if (time >= 10.8):
          tilt = 4
      elif (tilt == 5):
        time1 = time.time() - tim
        if ((time1 >= 13) and (time1 < 13.8)): #3 second delay to tilt the barrel to the next angle
          p_dc.start(SERVO_INITIAL_DUTY)
          p_servo.start(SERVO_INITIAL_DUTY)
        if ((time1 >= 13.8) and (time1 < 14.8)):
          p_servo.stop()
        if ((time1 >= 14.8) and (time1 < 15.8)):
          p_dc.stop()
        if (time >= 15.8):
          tilt = 6
      else:
        break
  return 0 #no longer any need for this once all 3 shots have been fired
   
print('functions defined')


def main():
    print("main function starting")
    rclpy.init()

    qos = QoSProfile(depth=10)
    aim_node = rclpy.create_node('aiming_algorithm') #I think that I also need to create a listener/subscriber for the 
    vel_pub = aim_node.create_publisher(Twist, 'cmd_vel', qos) 
    to_nav = aim_node.create_publisher(int, 'shoot', qos)
    
    cam = Adafruit_AMG88xx()
    threshhold_temp = 30
    
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
    thermal = cam.readPixels()
    SERVO_INITIAL_DUTY = set_duty_cycle_2(thermal, 0)
    
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
            
            thermal = cam.readPixels()
            
            thermal_data(thermal)
            stop(thermal, tilt)
            while not stop(thermal, tilt):
                print('have yet to stop')
                while not is_centered(cam):
                    print('adjusting to centre')
                    side_adjust(cam, vel_pub)
                print('bot stopping')
                stop_bot(vel_pub)
                print('centered')
                sleep(3)
                
                while not spaced_well(cam, threshhold_temp):
                    print('spacing')
                    distance_adust(cam, vel_pub, threshhold_temp) #remember to leave the 
                stop_bot(vel_pub)
                print('spaced')
            
            while stop(thermal, tilt):
                print('stopped')
                stop(thermal, tilt)
                angle(thermal,tilt)
                move_to_angle_2(p_tilt,thermal,tilt)
                firing_mech(p_tilt, tilt, thermal, p_servo, p_dc, SERVO_INITIAL_DUTY)
                

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

