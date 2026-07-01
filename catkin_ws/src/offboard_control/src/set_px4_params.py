#!/usr/bin/env python3
import rospy
# mavros services for setting and pulling PX4 parameters
from mavros_msgs.srv import ParamSet, ParamPull
from mavros_msgs.msg import ParamValue
from mavros_msgs.msg import State

def wait_connection():
    rospy.loginfo("[SET_PARAMS] waiting for MAVROS connection...")
    # blocks until one mavros/state message arrives
    state = rospy.wait_for_message("mavros/state", State)
    # loops until PX4 is connected
    while not state.connected:
        state = rospy.wait_for_message("mavros/state", State)
    rospy.loginfo("[SET_PARAMS] MAVROS connected...")

def wait_parameters():
    rospy.loginfo("[SET_PARAMS] pulling parameter list from PX4...")
    # waits for MAVROS parameter pull service
    rospy.wait_for_service("mavros/param/pull")
    try:
        pull_srv = rospy.ServiceProxy("mavros/param/pull", ParamPull)
        # have MAVROS fetch the full parameter list from PX4
        response = pull_srv(force_pull=True)
        rospy.loginfo("[SET_PARAMS] pulled %d parameters", response.param_received)
    except rospy.ServiceException as e:
        rospy.logerr("[SET_PARAMS] parameter pull failed: %s", e)

def set_parameter(name, integer_value, real_value):
    rospy.wait_for_service("mavros/param/set")
    try:
        set_param_srv = rospy.ServiceProxy("mavros/param/set", ParamSet)
        # builds the MAVROS parameter value
        param_value = ParamValue(integer=integer_value, real=real_value)
        response = set_param_srv(name, param_value)
        if response.success:
            rospy.loginfo("[SET_PARAMS] set %s successfully", name)
        else:
            rospy.logerr("[SET_PARAMS] failed to set %s", name)
    except rospy.ServiceException as e:
        rospy.logerr("[SET_PARAMS] service call failed: %s", e)

if __name__ == "__main__":
    rospy.init_node("px4_param_setter")
    wait_connection()
    wait_parameters()   # ← wait for full parameter list before setting

    # COM_RC_LOSS_T is float
    set_parameter("COM_RC_LOSS_T", 0, 60.0)
    # COM_DL_LOSS_T is integer
    set_parameter("COM_DL_LOSS_T", 60, 0.0)

    rospy.loginfo("[SET_PARAMS] parameter setup complete")