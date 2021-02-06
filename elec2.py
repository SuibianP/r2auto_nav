#!/usr/bin/env python3
# Shebang is just for the convenience of executing this file without typing python3
"""Operate motor and plunger when distance at 0 degree is 1 metre"""

import time # for time.sleep()
import logging # for exception logging
import RPi.GPIO as GPIO # for GPIO
# https://sourceforge.net/p/raspberry-gpio-python/wiki/Home/
import rclpy # ROS related
from rclpy.node import Node # Node base class
from rclpy.qos import qos_profile_sensor_data # quality of service profile
from sensor_msgs.msg import LaserScan # message definition for lidar

# sort-of constants
# pin numbers are in BCM notation
SERVO_PIN = 12
PLUNGER_PIN = 21
SERVO_FREQ = 50 # PWM frequency, Hertz
SERVO_INITIAL_DUTY = 2.5 # equivalent to 0 deg position
SERVO_DUTY_DELTA = 2.5 # equivalent to 45 deg change
PLUNGER_SLEEP = 0.5 # time for plunger to stay at HIGH position, in seconds
EPSILON = 0.2 # how far we allow the distance to go from 1m

# subscriber node
class Scanner(Node):
    """custom node inheriting base class Node"""
    # https://index.ros.org/doc/ros2/Tutorials/Writing-A-Simple-Py-Publisher-And-Subscriber/
    def __init__(self):
        super().__init__('scanner') # call init of parent class
        self.subscription = self.create_subscription(
            # http://docs.ros2.org/latest/api/rclpy/api/node.html#rclpy.node.Node.create_subscription
            LaserScan, # message definition
            # https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/LaserScan.msg
            'scan', # topic name
            self.listener_callback, # callback function
            qos_profile_sensor_data) # quality of service profile
        self.get_logger().info('Subscribed onto the scan topic')
        # https://docs.python.org/3/howto/logging.html
        self.servo_duty = 2.5 # in percentage
        self.get_logger().info(f'Initial servo duty set to {self.servo_duty}')
        self.delta_direction = 1 # initial turning direction is positive

    def listener_callback(self, msg):
        """Every new message in scan topic will invoke this function"""
        self.get_logger().info(f'Received range message, distance at 0 degree is {msg.ranges[0]}')
        if abs(msg.ranges[0] - 1) > EPSILON:
            self.get_logger().info('Distance not around 1m, exiting callback function.')
            return # to reduce indentation level
        self.servo_duty += SERVO_DUTY_DELTA * self.delta_direction
        self.get_logger().info(f'Duty cycle set to {self.servo_duty}')
        if (self.servo_duty > 10) or (self.servo_duty < 2.5):
            self.delta_direction = -self.delta_direction # reverse the direction of turning
            self.get_logger().info(f'Duty cycle at boundary, reversed to {self.delta_direction}')
        p.ChangeDutyCycle(self.servo_duty)
        self.get_logger().info(f'Duty cycle changed to {self.servo_duty}')
        GPIO.output(PLUNGER_PIN, GPIO.HIGH)
        time.sleep(PLUNGER_SLEEP)
        GPIO.output(PLUNGER_PIN, GPIO.LOW)
if __name__ == '__main__':
    try:
        # GPIO initialisation
        GPIO.setmode(GPIO.BCM) # BCM pin notation
        GPIO.setup(SERVO_PIN, GPIO.OUT)
        GPIO.setup(PLUNGER_PIN, GPIO.OUT)
        p = GPIO.PWM(SERVO_PIN, SERVO_FREQ) # PWM object
        p.start(SERVO_INITIAL_DUTY) # start PWM

        rclpy.init() # rclpy initialisation
        scanner_node = Scanner() # create a instance of our custom class
        rclpy.spin(scanner_node) # keep executing
        # http://docs.ros2.org/latest/api/rclpy/api/init_shutdown.html
    except: # pylint: disable=bare-except
        logging.exception("Exception caught")
