#!/usr/bin/env python


import numpy as np
import rospy
from sensor_msgs.msg import Range
from bff_msgs.msg import FloatStatus
from float_sensor_msgs.msg import Depth

class FilterStatus:
    """ Republishes the ping values at a specified lag
    """
    def __init__(self):
        self.float_name = rospy.get_param("~float_name", "float2")
        self.depth = 0.0
        rospy.Subscriber(self.float_name + "/status", FloatStatus, self.statCallback)
        rospy.Subscriber("/pressure_sensor/depth", Depth, self.depthCallback)
        
        self.statPub_ = rospy.Publisher(self.float_name + "/status_surface", FloatStatus, queue_size=1)
    
    def depthCallback(self, msg):
        self.depth = msg.depth

    def statCallback(self, msg):
        if self.depth < 1.0:
            self.statPub_.publish(msg)

if __name__ == "__main__":
    rospy.init_node('stat_filter', anonymous=True)
    fs = FilterStatus()

    rospy.spin()
