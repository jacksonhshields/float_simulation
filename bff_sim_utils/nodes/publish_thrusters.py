#!/usr/bin/env python

import numpy as np
import rospy
from uuv_gazebo_ros_plugins_msgs.msg import FloatStamped



if __name__ == "__main__":
    rospy.init_node('thruster_pub', anonymous=True)
    pub0 = rospy.Publisher('/bffv1/thrusters/0/input', FloatStamped, queue_size=1, latch=True)
    pub1 = rospy.Publisher('/bffv1/thrusters/1/input', FloatStamped, queue_size=1, latch=True)
    r = rospy.Rate(10)
    val = rospy.get_param("~cmd", 500.0)
    while not rospy.is_shutdown():
        msg = FloatStamped()
        msg.header.stamp = rospy.Time.now()
        msg.data = val
        msg0 = msg
        msg1 = msg
        msg1.data = msg0.data
        msg0.header.frame_id = '/bffv1/thruster_0'
        msg1.header.frame_id = '/bffv1/thruster_1'
        pub0.publish(msg0)
        pub1.publish(msg1)
        r.sleep()