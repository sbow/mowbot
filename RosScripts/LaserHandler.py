#!/usr/bin/env python
# LaserHandler.py
# Author: Shaun Bowman
# Feb 11 2018
# Handler for ROS sensor_msgs/LaserScan

# Every python controller needs these lines
import roslib
import rospy

# The laser scan message
from sensor_msgs.msg import LaserScan


class LaserHandler:
    def __init__(self, angle_min=-2*3.14*1/4, angle_max=2*3.14*1/4):
        # Setup internal parameters of the LaserHandler
        # Assumes angle_min < 0
        # angle_max > 0, ie: center is 0 radians, angle_min is start of ROI &
        # rotation is clockwise (positive delta(angle))
        self.roi_angle_min = angle_min  # min angle of interest
        self.roi_angle_max = angle_max  # max angle of interest
        self.header = 0             # timestamp of start of scan
        self.angle_min = 0.0        # LaserScan msg angle min
        self.angle_max = 0.0        # LaserScan msg angle min
        self.angle_increment = 0.0  # angular distance between measurements
        self.time_increment = 0.0   # time between measurements
        self.scan_time = 0.0        # time between scans
        self.range_min = 0.0        # min range for range measurements
        self.range_max = 0.0        # max range for range measurements
        self.ranges = 0             # ranges after signal conditioning
        self.raw_ranges = 0.0       # ranges prior to signal conditioning
        self.roi_first_elem = 0     # first array index in roi
        self.roi_last_elem  = 0     # last array index in roi
        self.roi_valid_elem = [None]    # boolean array if valid intensity
        self.roi_angles = [None]        # angles in roi

        self.has_init = False        # has initialized (parsed 1st scan)

        # Subscriber for the laser data
        self.sub = rospy.Subscriber('robot0/laser_0', LaserScan, self.laser_handler_callback)

        # Let the world know we're ready
        rospy.loginfo('LaserHandler initialized')

    def laser_handler_callback(self, scan):
        # When there's a new laser scan messege available
        # compute varios signal conditioning functions
        # output is intended to provide meaningful laser data
        # to a subscriber; likely a robot controller

        self.header             = scan.header
        self.angle_min          = scan.angle_min
        self.angle_max          = scan.angle_max
        self.angle_increment    = scan.angle_increment
        self.time_increment     = scan.time_increment
        self.scan_time          = scan.scan_time
        self.range_min          = scan.range_min
        self.range_max          = scan.range_max
        self.raw_ranges         = scan.ranges           # pre signal conditioning copy

        rospy.logdebug('Status: LaserHandler received scan')

        self.__parse_laser()    # call signal conditioner


    def __parse_laser(self):
        # Parse raw laser data
        # this calls other internal methods defined elsewhere
        self.__roi_ranges()
        self.__set_valid_thresh()

        if not self.has_init:    # if was first scan call init complete
            self.has_init = True

    def __roi_ranges(self):
        # limit self.ranges to roi_angle_min and roi_angle_max

        if not self.has_init:
            sweep_angle = self.angle_min
            found_min = False
            found_max = False
            for i in range(len(self.raw_ranges)):
                #rospy.loginfo('temp range {0} angle inc {1}'.format(sweep_angle, self.angle_increment))
                if sweep_angle > self.roi_angle_min:
                    if not found_min:
                        self.roi_first_elem = i
                        found_min = True
                if sweep_angle > self.roi_angle_max:
                    if not found_max:
                        self.roi_last_elem = i
                        found_max = True
                sweep_angle = sweep_angle + self.angle_increment

            self.ranges = [None]*len( range( self.roi_last_elem - self.roi_first_elem))
            self.roi_angles = [(self.roi_angle_min + self.angle_increment*i) for i in range( self.roi_last_elem - self.roi_first_elem)]
            #self.ranges = [None]*len( range( (self.roi_first_elem - len(self.raw_ranges)) + self.roi_last_elem))
            # Let the world know we're ready

#        # DONT NEED THIS - This is a circular array implementation, not needed
#        # because LaserScan.ranges[0] is at angle_min not 0 radians
#
#        # get subset of raw ranges inside ROI and assign to self.ranges; note %
#        # to treat raw_ranges as a circular array. Assumes angle_min >
#        # angle_max, ie: center is 0 radians, angle_min is start of ROI &
#        # rotation is clockwise (positive delta(angle))
#        i_ranges = 0
#        for i in range( (len(self.raw_ranges) % self.roi_first_elem) + self.roi_last_elem + 1):
#            self.ranges[i_ranges] = self.raw_ranges[ (i + self.roi_first_elem) % len( self.raw_ranges )]
#            i_ranges += 1

        # assign elements from raw_ranges to self.ranges if inside roi
        self.ranges = self.raw_ranges[self.roi_first_elem:self.roi_last_elem]

    def __set_valid_thresh(self):
        # sets roi_valid_elem false for elements outside range min / max
        if not self.has_init:
            self.roi_valid_elem = [True for i in range( len(self.ranges))]

        for i in range( len(self.roi_valid_elem) ):
            range_to_check = self.ranges[i]
            self.roi_valid_elem[i] = self.roi_valid_elem[i] and \
                    (range_to_check > self.range_min  and \
                     range_to_check < self.range_max)

#        rospy.loginfo('roi_valid_elem: {0} of {1} elem {2} min {3}\
#                        max {4} snsr_min {5} snsr_max'.format(sum(self.roi_valid_elem), \
#                        len(self.roi_valid_elem), self.range_min, self.range_max,\
#                        min(self.ranges), max(self.ranges)))

if __name__ == '__main__':
    rospy.init_node('LaserHandler')

    # Set up the LaserHandler
    laser_handler = LaserHandler()

    # Hand control over to ROS
    rospy.spin()
