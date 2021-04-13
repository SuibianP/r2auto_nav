#!/usr/bin/env python3

"""
EG2310 navigation codebase
"""

import cmath
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import Twist
import numpy as np
from scipy.signal import convolve2d
import tf2_ros
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation
import cv2 as cv
import code
# from tf.transformations import euler_from_quaternion
# The feature was removed in tf2
from dj import dijkstras
from timeout import timeout, TimeoutError

pt2array = lambda pt:np.array([pt.x, pt.y, pt.z])
qt2array = lambda qt:np.array([qt.x, qt.y, qt.z, qt.w])

class Nav(Node):
    """
    navigation node prototype
    """
    def __init__(self):
        super().__init__('navigation')
        self.get_logger().info('Initialising Nav node')
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10) # TODO: why depth 10? QoS?
        self.get_logger().info('`cmd_vel` topic publisher created.')
        self.sht_sub = self.create_subscription(
            Bool,
            'shoot',
            self.shoot_callback,
            qos_profile_sensor_data
            )


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
        self.pos_grid = None
        self.shoot = False

    def shoot_callback(self, msg):
        self.shoot = msg.data

    def scan_callback(self, msg):
        """
        The data is mainly used for anti collision purposes
        """
        self.get_logger().debug('In scan_callback')
        self.scan_data = np.array(msg.ranges)
        self.scan_data[self.scan_data == 0] = np.nan

    def odom_callback(self, msg):
        """
        self.pos: [x, y]
        self.ori: [x, y]
        """
        self.get_logger().debug('In odom_callback')
        while True:
            try:
                trans = self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())
            except Exception as e:
                rclpy.spin_once(self)
                self.get_logger().warn(str(e))
                continue
            break
        #self.pos = np.add(pt2array(msg.pose.pose.position), pt2array(trans.transform.translation))[:2]
        self.pos = pt2array(trans.transform.translation)[:2]
        self.get_logger().debug(f'transformed position: {self.pos}')
        #self.ori = np.array(Rotation.from_quat(qt2array(msg.pose.pose.orientation) + qt2array(trans.transform.rotation)).as_rotvec()[2])
        self.ori = np.array(Rotation.from_quat(qt2array(trans.transform.rotation)).as_euler('zyx')[0])
        self.get_logger().debug(f'transformed orientation in degrees: {self.ori * 180 / np.pi}')

    def occ_callback(self, msg):
        """
        self.occ_data 2D np.ndarray
        """
        self.get_logger().info('In occ_callback')
        occ_data = np.array(msg.data)
        occ_data = occ_data + 1
        self.occ_data = np.uint8(occ_data.reshape(msg.info.height, msg.info.width))
        #self.occ_data = np.rot90(self.occ_data, 2)
        if self.pos is None:
            self.get_logger().info('pos infomation not available yet')
            return
        self.pos_grid = ((self.pos - pt2array(msg.info.origin.position)[:2]) / msg.info.resolution).astype(int)
        #assert self.occ_data[tuple(self.pos_grid)] == 1
        self.get_logger().info(f'position grid coordinates: {self.pos_grid}')

    def rotate(self, drtn, rotspd=0.4):
        # self.stop()
        # drtn is a vector of target direction [x, y]
        rad2comp = lambda rad: cmath.exp(1j * rad)
        cart2comp = lambda cart: rad2comp(np.arctan2(cart[1], cart[0]))
        c_dir = lambda: np.sign((cart2comp(drtn) / rad2comp(self.ori)).imag)
        twist = Twist()
        self.get_logger().info(f'dest degree: {np.arctan2(drtn[1], drtn[0]) * 180 / np.pi}')
        # use complex numbers to handle angles going from 360->0 / -180->180
        # divide the two complex numbers to get the change in direction
        # get the sign of the imaginary component to figure out which way we have to turn
        c_change_dir = c_dir()
        self.get_logger().debug(f'c_change_dir: {c_change_dir}')
        # twist.linear.x = 0.0
        twist.angular.z = c_change_dir * rotspd

        # we will use the c_dir_diff variable to see if we can stop rotating
        # stop when c_dir_diff is reverse from c_change_dir
        while c_dir() == c_change_dir:
            self.cmd_pub.publish(twist)
            self.get_logger().debug('Still need to turn more')
            rclpy.spin_once(self)
            self.get_logger().debug(f'updated degrees: {self.ori * 180 / np.pi}')
            self.get_logger().debug(f'c_dir(): {c_dir()}')

        self.get_logger().info(f'Rotation finished, finished degrees: {self.ori * 180 / np.pi}')
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)

    def move(self, drtn, linspd=0.1): # TODO speed
        """
        Move towards direction at speed
        """
        self.rotate(drtn)
        twist = Twist()
        twist.linear.x = linspd
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)
        self.get_logger().info(f'Published straight movement twist. Sleeping for a while... coord {self.pos_grid}')
        time.sleep(1)
        self.get_logger().info(f'Woken up, coord {self.pos_grid}')
        # self.stop()

    def stop(self):
        """
        Move towards direction at speed
        """
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)
        self.get_logger().info('Stopped')

   @timeout(2) # TODO
    def new_goal(self):
        """
        determine a new goal from occmap (boundary between unmapped and empty)
        """
        temp = self.occ_data
        temp[temp == 0] = 80
        temp[temp == 1] = 0
        temp[temp != 0] = 1
        y,x = np.ogrid[-2: 2+1, -2: 2+1]
        mask = (x**2+y**2 <= 2).astype(int)
        temp1 = convolve2d(temp, mask, mode='same')
        temp1 = temp
        temp1[temp1 > 0] = 1
        self.get_logger().info('In new_goal')
        goal = (0, 0)
        corners = np.array(cv.cornerHarris(self.occ_data,2,3,0.04))
        #self.get_logger().info(f'corners(uncensored): {np.count_nonzero(corners)}: {corners}')
        if np.count_nonzero(corners) == 0:
            raise TimeoutError
        cn_ind = list(zip(*np.where(corners>0.03*corners.max()))) # coords of corners
        if len(cn_ind) == 0:
            raise TimeoutError
        self.get_logger().info(f'corners: {cn_ind}')
        far = list(max(cn_ind, key=lambda x: np.linalg.norm(self.pos_grid - x)))
        self.get_logger().info(f'farthest: {far}')
        while (temp1[tuple(far)] != 0):
            far[0] -= 1;
            far[1] -= 1;
        far[0] -= 2;
        far[1] -= 2;

    def global_planner(self):
        """
        Takes current pos and goal point, output a route (nparray of points)
        """
        self.get_logger().info('In global_planner')
        temp = self.occ_data
        temp[temp == 0] = 80
        temp[temp == 1] = 0
        temp[temp != 0] = 1
        plt.imshow(temp)
        plt.pause(1)
        y,x = np.ogrid[-5: 5+1, -5: 5+1]
        mask = (x**2+y**2 <= 2**2).astype(int)
        temp = convolve2d(temp, mask, mode='same')
        temp[temp > 0] = 1
        self.get_logger().info(np.array2string(temp,threshold=100000))
        plt.imshow(temp)
        plt.plot(self.pos_grid[0], self.pos_grid[1], markersize=6)
        plt.plot(self.goal[0], self.goal[1], markersize=6)
        plt.pause(0.0001)
        self.get_logger().info(f'pos grid: {self.pos_grid}, goal: {self.goal}')
        self.plan = dijkstras(temp, 1, 1, self.pos_grid, self.goal)
        self.get_logger().info(f'new plan: {self.plan}')
        if self.plan is False:
            return False
        plt.imshow(self.occ_data)
        plt.plot(self.plan[:, 0], self.plan[:, 1])
        plt.savefig('debug.png')
        plt.pause(0.0001)
        return True

    def closure(self):
        """
        check if contour is closed
        """
        self.get_logger().info('In closure')
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

    @timeout(120) # TODO: timeout duration
    def local_planner(self):
        """
        Takes the route from the global planner, try to get the robot on track to the route
        This keeps running until target is reached or timeout is reached
        """
        self.get_logger().info('in local_planner')
        THRESH = 4 # TODO
        SAFE_DIST = 0.3 # TODO
        curr = 0 # current point in plan
        while (curr < self.plan.shape[0]) and (np.linalg.norm(self.pos_grid - self.goal) >= THRESH): # not approaching goal yet
            if self.shoot:
                self.stop()
                break
            self.get_logger().info('Not approaching goal yet')
            self.get_logger().info(f'Current distance to waypoint {curr}: {np.linalg.norm(self.pos_grid - self.plan[curr])}')
            while (curr < self.plan.shape[0]) and \
                    (np.linalg.norm(self.pos_grid - self.plan[curr]) < THRESH):
                curr = curr + 1
                # find first succeeding point far enough
            self.get_logger().info(f'Chosen next waypoint index {curr} with coords {self.plan[curr]}')
            drtn = (self.plan[curr] - self.pos_grid) # the correcton direction, [x, y]
            self.get_logger().info(f'Going in {drtn} direction')
            if self.scan_data[int(np.arctan2(drtn[0], drtn[1]) * 180 / np.pi)] <= SAFE_DIST:
                self.get_logger().warn(f'Collision risk detected. \
                    Aborting local planner for new plan.')
                return

            self.move(drtn) # turn towards that direction
            rclpy.spin_once(self) # allow callbacks to update

        self.stop()

    def begin(self):
        """
        Robot mover until closure
        """
        self.get_logger().info('Waiting for messages')
        while self.scan_data is None or self.occ_data is None \
                or self.pos is None or self.ori is None or self.pos_grid is None:
                    rclpy.spin_once(self)
        self.get_logger().info('Starting operation.')
        #self.move([math.cos(np.pi / 180 * np.nanargmax(self.scan_data)), math.sin(np.pi / 180 * np.nanargmax(self.scan_data))])
        #self.get_logger().info('Moving to longest direction')
        #while np.nanmin(self.scan_data) > 0.2:
        #    self.get_logger().info('Trying to move forward.')
        #    rclpy.spin_once(self)
        self.stop()
        while True or not self.closure(): # TODO
            try:
                self.get_logger().info('Contour not closed yet')
                self.new_goal()
                if self.global_planner() is False:
                    continue
                self.local_planner()
            except TimeoutError:
                self.get_logger().info('Timeout reached, turning to random direction')
                self.rotate(np.random.randint(-30, 30, 2))
        self.stop()

def main(args=None):
    """
    main routine
    """
    rclpy.init(args=args)
    nav_inst = Nav()
    nav_inst.begin()

if __name__ == '__main__':
    main()
