#!/usr/bin/env python


import numpy as np
import rospy
import pymap3d
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry

class GPSPublisher:
    """ 
    """
    def __init__(self):
        self.float_name = rospy.get_param("~float_name", "float2")
        rospy.Subscriber(self.float_name + "/pose_gt", Odometry, self.poseCallback)
        self.datum = [-30.2424625,153.1932348]
        self.gpsPub_ = rospy.Publisher("/nav_sat_fix", NavSatFix, queue_size=1)
    

    def poseCallback(self, msg):
        nmsg = NavSatFix()
        nmsg.header = msg.header
        llh = pymap3d.enu2geodetic(e=msg.pose.pose.position.x, n=msg.pose.pose.position.y, u=0, lat0=self.datum[0], lon0=self.datum[1], h0=0.)
        nmsg.latitude = llh[0]
        nmsg.longitude = llh[1]
        self.gpsPub_.publish(nmsg)

if __name__ == "__main__":
    rospy.init_node('gps_pub', anonymous=True)
    fs = GPSPublisher()

    rospy.spin()
