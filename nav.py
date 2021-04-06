#!/usr/bin/env python3

"""
EG2310 navigation codebase
"""

import cmath
import time
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
import code
# from tf.transformations import euler_from_quaternion
# The feature was removed in tf2
from dj import dijkstras
from timeout import timeout

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
            except:
                rclpy.spin_once(self)
                continue
            break
        #self.pos = np.add(pt2array(msg.pose.pose.position), pt2array(trans.transform.translation))[:2]
        self.pos = pt2array(trans.transform.translation)[:2]
        self.get_logger().debug(f'transformed position: {self.pos}')
        #self.ori = np.array(Rotation.from_quat(qt2array(msg.pose.pose.orientation) + qt2array(trans.transform.rotation)).as_rotvec()[2])
        self.ori = np.array(Rotation.from_quat(qt2array(trans.transform.rotation)).as_euler('zyx')[0])
        self.get_logger().info(f'transformed orientation: {self.ori}')

    def occ_callback(self, msg):
        """
        self.occ_data 2D np.ndarray
        """
        self.get_logger().info('In occ_callback')
        occ_data = np.array(msg.data)
        occ_data = occ_data + 1
        self.occ_data = np.uint8(occ_data.reshape(msg.info.height, msg.info.width))
        if self.pos is None:
            self.get_logger().info('pos infomation not available yet')
            return
        self.pos_grid = ((self.pos - pt2array(msg.info.origin.position)[:2]) / msg.info.resolution).astype(int)
        #assert self.occ_data[tuple(self.pos_grid)] == 1
        self.get_logger().info(f'position grid coordinates: {self.pos_grid}')

    def move(self, drtn, rotspd=0.2, linspd=0.1): # TODO speed
        """
        Move towards direction at speed
        """
        # drtn is a vector of target direction [x, y]
        rad2comp = lambda rad: cmath.exp(1j * rad)
        cart2comp = lambda cart: rad2comp(np.arctan2(cart[0], cart[1]))
        c_dir = lambda: np.sign((rad2comp(self.ori) - cart2comp(drtn)).imag)
        self.get_logger().info(f'Start moving towards direction {drtn} \
                with linear speed {linspd} and rotation speed {rotspd}')
        twist = Twist()
        # use complex numbers to handle angles going from 360->0 / -180->180
        # divide the two complex numbers to get the change in direction
        # get the sign of the imaginary component to figure out which way we have to turn
        c_change_dir = c_dir()
        self.get_logger().info(f'c_change_dir: {c_change_dir}')
        twist.linear.x = 0.0
        twist.angular.z = c_change_dir * rotspd

        # we will use the c_dir_diff variable to see if we can stop rotating
        # stop when c_dir_diff is reverse from c_change_dir
        while c_dir() == c_change_dir:
            self.cmd_pub.publish(twist)
            self.get_logger().debug('Still need to turn more')
            rclpy.spin_once(self)
            self.get_logger().debug(f'updated orientation: {self.ori}')

        self.get_logger().info('Rotation finished')
        twist = Twist()
        twist.linear.x = linspd
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)
        self.get_logger().info('Published straight movement twist. Sleeping for a while...')
        time.sleep(0.5)
        self.get_logger().info('Woken up from sleep')

    @timeout(300) # TODO
    def new_goal(self):
        """
        determine a new goal from occmap (boundary between unmapped and empty)
        """
        self.get_logger().info('In new_goal')
        goal = (0, 0)
        while (self.occ_data[goal] != 1):
            goal = (np.random.randint(0, self.occ_data.shape[0]), np.random.randint(0, self.occ_data.shape[1]))
            self.get_logger().info(f"goal: {goal}, corresponding occdata: {self.occ_data[goal]}")
        self.goal = np.array(goal) # TODO

    def global_planner(self):
        """
        Takes current pos and goal point, output a route (nparray of points)
        """
        self.get_logger().info('In global_planner')
        temp = self.occ_data
        temp[temp != 1] = 120
        temp[temp == 1] = 0
        temp[temp == 120] = 1
        self.get_logger().info(f'pos grid: {self.pos_grid}, goal: {self.goal}')
        self.plan = dijkstras(temp, 1, 1, self.pos_grid, self.goal)
        if self.plan is False:
            return False
        plt.imshow(self.occ_data)
        plt.plot(self.plan[:, 0], self.plan[:, 1])
        plt.plot(self.pos_grid[0], self.pos_grid[1])
        plt.plot(self.goal[0], self.goal[1])
        plt.pause(0.0001)

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

    @timeout(300) # TODO: timeout duration
    def local_planner(self):
        """
        Takes the route from the global planner, try to get the robot on track to the route
        This keeps running until target is reached or timeout is reached
        """
        self.get_logger().info('in local_planner')
        THRESH = 4 # TODO
        SAFE_DIST = 0.1 # TODO
        curr = 0 # current point in plan
        while (curr < self.plan.shape[0]) and (np.linalg.norm(self.pos_grid - self.goal) >= THRESH): # not approaching goal yet
            self.get_logger().info('Not approaching goal yet')
            if np.amin(self.scan_data) <= SAFE_DIST:
                # TODO: take into account other factors
                self.get_logger().warn(f'Collision risk detected. \
                    Aborting local planner for new plan.')
                return

            self.get_logger().info(f'Current distance to waypoint {curr}: {np.linalg.norm(self.pos_grid - self.plan[curr])}')
            while (curr < self.plan.shape[0]) and \
                    (np.linalg.norm(self.pos_grid - self.plan[curr]) < THRESH):
                curr = curr + 1
                # find first succeeding point far enough
            self.get_logger().info(f'Chosen next waypoint index {curr}')
            drtn = (self.plan[curr] - self.pos_grid) # the correcton direction
            self.get_logger().info(f'Going in {drtn} direction')

            self.move(drtn) # turn towards that direction
            rclpy.spin_once(self) # allow callbacks to update

    def begin(self):
        """
        Robot mover until closure
        """
        self.get_logger().info('Waiting for messages')
        while self.scan_data is None or self.occ_data is None \
                or self.pos is None or self.ori is None or self.pos_grid is None:
                    rclpy.spin_once(self)
        self.get_logger().info('Starting operation.')
        while True or not self.closure():
            self.get_logger().info('Contour not closed yet')
            self.new_goal()
            if self.global_planner() is False:
                continue
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
