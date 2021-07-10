#!/usr/bin/env python


import numpy as np
import rospy
from sensor_msgs.msg import Range

class PingLag:
    """ Republishes the ping values at a specified lag
    """
    def __init__(self):
        self.ping_lag = rospy.get_param("~ping_lag", 2.0)
        self.pings_list = []
        self.times_list = []
        self.localisationSub_ = rospy.Subscriber("ping", Range, self.pingCallback)
        self.lagPub_ = rospy.Publisher('ping_lag', Range, queue_size=1)


    def pingCallback(self, msg):
        self.pings_list.append(msg.range)
        self.times_list.append(rospy.Time.now().to_sec())

        idx_lag = np.abs(np.array(self.times_list) - rospy.Time.now().to_sec() + self.ping_lag).argmin()
        if idx_lag < 0:
            rospy.logwarn("Not enough pings received yet")
        ping_lag = self.pings_list[idx_lag]
        self.times_list = self.times_list[idx_lag:]
        self.pings_list = self.pings_list[idx_lag:]

        lag_msg = msg
        lag_msg.range = ping_lag

        self.lagPub_.publish(lag_msg)

if __name__ == "__main__":
    rospy.init_node('lagger', anonymous=True)
    controller = PingLag()

    rospy.spin()
