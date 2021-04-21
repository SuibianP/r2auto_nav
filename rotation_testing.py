#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Apr  6 00:44:14 2021

@author: jfoo
"""
print("rotation_testing.py file accessed")

import rclpy

from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile
from time import sleep

BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = 2.84

WAFFLE_MAX_LIN_VEL = 0.26
WAFFLE_MAX_ANG_VEL = 1.82

LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.1

print('setup complete')

def set_twist(target_linear_velocity, target_angular_velocity):        
    '''
    publisher is to be pub. I.e. the pub = node.create_publisher(Twist, 'cmd_vel', qos) nonsense
    '''
    
    control_linear_velocity = 0.0
    control_angular_velocity = 0.0
    
    twist = Twist()

    control_linear_velocity = make_simple_profile(
        control_linear_velocity,
        target_linear_velocity,
        (LIN_VEL_STEP_SIZE / 2.0))

    twist.linear.x = control_linear_velocity
    twist.linear.y = 0.0
    twist.linear.z = 0.0

    control_angular_velocity = make_simple_profile(
        control_angular_velocity,
        target_angular_velocity,
        (ANG_VEL_STEP_SIZE / 2.0))

    twist.angular.x = 0.0
    twist.angular.y = 0.0
    twist.angular.z = control_angular_velocity 
    
    return twist


def make_simple_profile(output, input, slop):
    if input > output:
        output = min(input, output + slop)
    elif input < output:
        output = max(input, output - slop)
    else:
        output = input

    return output

print('functions defined')

def main():
    print('main function called')
    
    rclpy.init()
            
    qos = QoSProfile(depth=10)
    node = rclpy.create_node('rotation_testing')  #rclpy.init() needs to be called before create_node function
    pub = node.create_publisher(Twist, 'cmd_vel', qos)
    
    try:
        while True:
            
            ang_vel_1 = float(input('Enter an anticlockwise rotation velocity: '))
            rot_time_1 = float(input('How long would you like to rotate for? '))
            twist = set_twist(0.0, ang_vel_1)
            print('rotating by ang_vel_1')
            pub.publish(twist)
            sleep(rot_time_1)
            
            twist = set_twist(0.0, 0.0)
            print('bot stopping 1')
            pub.publish(twist)
            sleep(3)
            
            ang_vel_2 = float(input('Enter a clockwise rotation velocity: '))
            rot_time_2 = float(input('How long would you like to rotate for? '))            
            twist = set_twist(0.0, -ang_vel_2)
            print('rotating by ang_vel -0.1')
            pub.publish(twist)
            sleep(rot_time_2)
            
            twist = set_twist(0.0, 0.0)
            print('bot stopping 2')
            pub.publish(twist)
            sleep(3)
        
        
    except Exception as e:
        print(e)

    finally:
        GPIO.cleanup()

main()
        

