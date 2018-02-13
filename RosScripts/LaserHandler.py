#!/usr/bin/env python
# LaserHandler.py
# Author: Shaun Bowman
# Feb 11 2018
# Handler for ROS sensor_msgs/LaserScan

# Every python controller needs these lines
import roslib
import rospy



class LaserHandler:
    def __init__(self, angle_min=2*3.14*3/4, angle_max=2*3.14*1/4):
        # Setup internal parameters of the LaserHandler
        # Assumes angle_min >
        # angle_max, ie: center is 0 radians, angle_min is start of ROI &
        # rotation is clockwise (positive delta(angle))
        self.roi_angle_min = angle_min  # min angle of interest
        self.roi_angle_max = angle_max  # max angle of interest
        self.header = 0             # timestamp
        self.angle_increment = 0.0  # angular distance between measurements
        self.time_increment = 0.0   # time between measurements
        self.scan_time = 0.0        # time between measurements when moving
        self.range_min = 0.0        # min range for range measurements
        self.range_max = 0.0        # max range for range measurements
        self.ranges = 0             # ranges after signal conditioning
        self.raw_ranges = 0.0       # ranges prior to signal conditioning

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
        self.angle_increment    = scan.angle_increment
        self.time_increment     = scan.time_increment
        self.scan_time          = scan.scan_time
        self.range_min          = scan.range_min
        self.range_max          = scan.range_max
        self.raw_ranges         = scan.ranges           # pre signal conditioning copy
        self.is_init            = False
        self.roi_first_elem     = 0
        self.roi_last_elem      = 0

        rospy.logdebug('Status: LaserHandler received scan')

        self.__parse_laser()    # call signal conditioner


    def __parse_laser(self):
        # Parse raw laser data
        # this calls other internal methods defined elsewhere
        self.__angle_range()

        if not self.is_init:    # if was first scan call init complete
            self.is_init = True

    def __angle_range(self):
        # limit self.ranges to roi_angle_min and roi_angle_max

        if not self.is_init:
            temp_range = self.range_min
            found_min = False
            found_max = False
            for i in range(len(self.raw_ranges)):
                if temp_range > self.roi_angle_min:
                    if not found_min:
                        self.roi_first_elem = i
                        found_min = True
                if temp_range > self.roi_angle_max:
                    if not found_max:
                        self.roi_last_elem = i
                        found_max = True
                temp_range = temp_range + self.angle_increment

            self.ranges = [None]*len( range( (self.roi_first_elem - len(self.raw_ranges)) + self.roi_last_elem))

        # get subset of raw ranges inside ROI and assign to self.ranges; note %
        # to treat raw_ranges as a circular array. Assumes angle_min >
        # angle_max, ie: center is 0 radians, angle_min is start of ROI &
        # rotation is clockwise (positive delta(angle))
        i_ranges = 0
        for i in range( (len(self.raw_ranges) % self.roi_first_elem) + end + 1):
            self.ranges[i_ranges] = self.raw_ranges[ (i + self._elem) % len( self.raw_ranges )]

if __name__ == '__main__':
    rospy.init_node('LaserHandler')

    # Set up the LaserHandler
    laser_handler = LaserHandler()

    # Hand control over to ROS
    rospy.spin()
