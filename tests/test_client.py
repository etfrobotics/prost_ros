#!/usr/bin/env python3

import rospy
from prost_ros.srv import StartPlanning, SubmitObservation
from prost_ros.msg import KeyValue

def test_client():
    rospy.init_node('test_client')
    
    rospy.wait_for_service('/prost_bridge/start_planning')
    rospy.wait_for_service('/prost_bridge/submit_observation')
    
    start_planning = rospy.ServiceProxy('/prost_bridge/start_planning', StartPlanning)
    submit_obs = rospy.ServiceProxy('/prost_bridge/submit_observation', SubmitObservation)
    
    # Load RDDL files
    with open('test_domain.rddl', 'r') as f:
        domain = f.read()
    with open('test_instance.rddl', 'r') as f:
        instance = f.read()
        
    rospy.loginfo("Calling StartPlanning...")
    # Timeout 10s
    resp = start_planning(domain, instance, 10)
    if not resp.success:
        rospy.logerr("StartPlanning failed!")
        return
        
    rospy.loginfo("StartPlanning success. Starting loop...")

    kv_list = [KeyValue("x", "0.0")]
    reward = 0.0
    
    for i in range(5):
        rospy.loginfo(f"Step {i}: sending obs x={kv_list[0].value}, reward={reward}")
        obs_resp = submit_obs(kv_list, reward)
        rospy.loginfo(f"Received Action: {obs_resp.action_name} {obs_resp.action_params}")
        
        # Simulate environment (x += 1 if action is 'a')
        # Note: PROST might send 'noop' if it doesn't find 'a' useful or exploring. 
        # In this domain, 'a' increases x, reward is x. So 'a' is good.
        # But wait, reward is x (current state). 
        # So increasing x increases FUTURE reward.
        
        val = float(kv_list[0].value)
        if obs_resp.action_name == "a":
             val += 1.0
             
        kv_list = [KeyValue("x", str(val))]
        reward = val # Reward depends on state
        
    rospy.loginfo("Test complete.")

if __name__ == "__main__":
    test_client()
