#!/usr/bin/env python3

import rospy
import time
from prost_ros.srv import StartPlanning

def test_reset():
    rospy.init_node('test_reset_client')
    rospy.wait_for_service('/prost_bridge/start_planning')
    start_planning = rospy.ServiceProxy('/prost_bridge/start_planning', StartPlanning)

    # 1. Start First Session (Test Domain)
    rospy.loginfo("Starting Session 1...")
    with open('test_domain.rddl', 'r') as f: d = f.read()
    with open('test_instance.rddl', 'r') as f: i = f.read()
    
    resp = start_planning(d, i, 10)
    if resp.success:
        rospy.loginfo("Session 1 Started.")
    else:
        rospy.logerr("Session 1 Failed.")
        return

    time.sleep(2)
    
    # 2. Start Second Session (Restart)
    rospy.loginfo("Starting Session 2 (Reset)...")
    resp = start_planning(d, i, 10)
    if resp.success:
        rospy.loginfo("Session 2 Started (Reset Successful).")
    else:
        rospy.logerr("Session 2 Failed to Reset.")

if __name__ == "__main__":
    test_reset()
