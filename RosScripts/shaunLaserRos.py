#!/usr/bin/env python
import roslib
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan

def callback(scan):
    min_dist = min(scan.ranges)
    max_dist = max(scan.ranges)
    rospy.loginfo(rospy.get_caller_id() + "Min Distance %s", min_dist)
    rospy.loginfo(rospy.get_caller_id() + "Max Distance %s", max_dist)
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    #rospy.Subscriber("robot0/laser_0", LaserScan, callback)
    rospy.Subscriber("scan", LaserScan, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
