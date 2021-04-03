#!/usr/bin/env python3

"""
EG2310 navigation codebase
"""

import cmath
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import Twist
import numpy as np
import tf2_ros
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation
import cv2 as cv
# from tf.transformations import euler_from_quaternion
# The feature was removed in tf2
from dj import dijkstras
from timeout import timeout

class Nav(Node):
    """
    navigation node prototype
    """
    def __init__(self):
        super().__init__('navigation')
        self.get_logger().info('Initialising Nav node')
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10) # TODO: why depth 10? QoS?
        self.get_logger().info('`cmd_vel` topic publisher created.')
        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile_sensor_data
            )
        self.get_logger().info('`scan` topic subscribed.')
        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
            )
        self.get_logger().info('`odom` topic subscribed.')
        self.occ_sub = self.create_subscription(
            OccupancyGrid,
            'map',
            self.occ_callback,
            qos_profile_sensor_data
            )
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.get_logger().info('`map` topic subscribed.')
        self.scan_data = None
        self.occ_data = None
        self.pos = None
        self.ori = None
        self.goal = None
        self.plan = None

    def scan_callback(self, msg):
        """
        The data is mainly used for anti collision purposes
        """
        self.get_logger().info('In scan_callback')
        self.scan_data = np.array(msg.ranges)
        self.scan_data[self.scan_data == 0] = np.nan

    def odom_callback(self, msg):
        """
        self.pos: [x, y]
        self.ori: [x, y]
        """
        self.get_logger().info('In odom_callback')
        pos_r = msg.pose.pose.position
        self.pos = [pos_r.x, pos_r.y]
        ori_r = msg.pose.pose.orientation
        self.ori = Rotation.from_quat([ori_r.x, ori_r.y, ori_r.z, ori_r.w]).as_rotvec()[:1]

    def occ_callback(self, msg):
        """
        self.occ_data 2D np.ndarray
        """
        self.get_logger().info('In occ_callback')
        occ_data = np.array(msg.data)
        occ_data = occ_data + 1
        occ_data = np.uint8(occ_data.reshape(msg.info.height, msg.info.width))
        # TODO: transform the map to match the base_link frame

    def move(self, drtn, rotspd=10, linspd=10): # TODO speed
        """
        Move towards direction at speed
        """
        # drtn is a vector of target direction [x, y]
        cart2comp = lambda cart: cmath.exp(1j * np.arctan2(cart[0], cart[1]))
        c_yaw = lambda: cart2comp(self.ori)
        c_target_yaw = cart2comp(drtn)
        self.get_logger().info(f'Start moving towards direction {drtn} \
                with linear speed {linspd} and rotation speed {rotspd}')
        twist = Twist()
        # use complex numbers to handle angles going from 360->0 / -180->180
        # divide the two complex numbers to get the change in direction
        # get the sign of the imaginary component to figure out which way we have to turn
        c_change_dir = np.sign((c_target_yaw / c_yaw()).imag)
        twist.linear.x = 0.0
        twist.angular.z = c_change_dir * rotspd
        self.cmd_pub.publish(twist)

        # we will use the c_dir_diff variable to see if we can stop rotating
        c_dir_diff = c_change_dir
        # stop when c_dir_diff is reverse from c_change_dir
        while c_change_dir * c_dir_diff > 0:
            rclpy.spin_once(self)
            c_dir_diff = np.sign((c_target_yaw / c_yaw()).imag)

        self.get_logger().info('Rotation finished')
        twist.linear.x = linspd
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)

    def new_goal(self):
        """
        determine a new goal from occmap (boundary between unmapped and empty)
        """
        self.goal = None # TODO

    def global_planner(self):
        """
        Takes current pos and goal point, output a route (nparray of points)
        """
        temp = self.occ_data
        temp[temp != 1] = 120
        temp[temp == 1] = 0
        temp[temp == 120] = 1
        self.plan = dijkstras(self.occ_data, 1, 1, self.pos, self.goal)
        plt.imshow(self.occ_data)
        plt.plot(self.plan[:, 0], self.plan[:, 1])

    def closure(self):
        """
        check if contour is closed
        """
        ALTHRESH = 10
        DILATE_PIXELS = 3
        binimg = cv.threshold(self.occ_data,
                              2, # promote all occupied to
                              255,
                              cv.THRESH_BINARY
                             )
        element = cv.getStructuringElement(cv.MORPH_CROSS, (DILATE_PIXELS, DILATE_PIXELS))
        img4 = cv.dilate(binimg, element)
        _, cnts = cv.findContours(img4, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)
        for cnt in cnts:
            if cv.contourArea(cnt) / cv.arcLength(cnt, True) > ALTHRESH:
                return True

        return False

    @timeout(30) # TODO: timeout duration
    def local_planner(self):
        """
        Takes the route from the global planner, try to get the robot on track to the route
        This keeps running until target is reached or timeout is reached
        """
        THRESH = 0.5 # TODO
        SAFE_DIST = 0.1 # TODO
        curr = 0 # current point in plan
        while (np.linalg.norm(self.pos - self.goal) >= THRESH) and \
            (curr < self.plan.shape[0]): # not approaching goal yet
            if np.amin(self.scan_data) <= SAFE_DIST:
                # TODO: take into account other factors
                self.get_logger().warn(f'Collision risk detected. \
                    Aborting local planner for new plan.')
                return

            while (np.linalg.norm(self.pos - self.plan[curr]) < THRESH) and \
                (curr < self.plan.shape[0]):
                curr = curr + 1
                # find first succeeding point far enough
            self.get_logger().info(f'Chosen next waypoint index {curr}')
            drtn = (self.plan[curr] - self.pos) # the correcton direction
            self.get_logger().info(f'Going in {drtn} direction')

            self.move(drtn) # turn towards that direction
            rclpy.spin_once(self) # allow callbacks to update

    def begin(self):
        """
        Robot mover until closure
        """
        self.get_logger().info('Waiting for messages')
        while self.scan_data is None or self.occ_data is None \
                or self.pos is None or self.ori is None:
                    pass
        self.get_logger().info('Starting operation.')
        while not self.closure():
            self.new_goal()
            self.global_planner()
            self.local_planner()

def main(args=None):
    """
    main routine
    """
    rclpy.init(args=args)
    nav_inst = Nav()
    nav_inst.begin()

if __name__ == '__main__':
    main()
