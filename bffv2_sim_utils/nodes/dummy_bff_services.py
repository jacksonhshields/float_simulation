#!/usr/bin/env python

import numpy as np
import rospy
from bff_msgs.msg import FloatMission, FloatStatus, FloatMissionResult
from bff_msgs.srv import AddFloatMission, AddFloatMissionResponse
from bff_msgs.srv import NotifyFloatDeploying, NotifyFloatDeployingResponse
from bff_msgs.srv import NotifyFloatRecovered, NotifyFloatRecoveredResponse
from std_srvs.srv import SetBool, SetBoolResponse, Trigger, TriggerResponse
from float_msgs.srv import SetState, SetStateResponse


class DummyBFFServices:
    """ Republishes the ping values at a specified lag
    """
    def __init__(self):
        self.float_name = rospy.get_param("~float_name", "float2")
        
        self.strobe_service = rospy.Service("camera_strobe_enable", SetBool, self.strobeCallback)
        self.imaging_start_service = rospy.Service("start_imaging", Trigger, self.imagingStartCallback)
        self.imaging_stop_service = rospy.Service("stop_imaging", Trigger, self.imagingStopCallback)
        self.localisation_service = rospy.Service("manage_localisation", SetBool, self.localisationCallback)
        self.state_service = rospy.Service("set_state", SetState, self.stateCallback)

    def strobeCallback(self, req):
        return SetBoolResponse(True, "strobe enabled")
    def imagingStartCallback(self, req):
        return TriggerResponse(True, "Cameras started")
    def imagingStopCallback(self, req):
        return TriggerResponse(True, "Cameras stopped")
    def localisationCallback(self, req):
        return SetBoolResponse(True, "localisation stopped")
    def stateCallback(self, req):
        # print("Set state %d" %req.state)
        return SetStateResponse(True)
    


if __name__ == "__main__":
    rospy.init_node('bff_services', anonymous=True)
    bff_servicer = DummyBFFServices()

    rospy.spin()