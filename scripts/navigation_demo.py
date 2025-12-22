#!/usr/bin/env python3

import rospy
import os
import re
from prost_ros.srv import StartPlanning, SubmitObservation
from prost_ros.msg import KeyValue

class NavigationSim:
    def __init__(self):
        # Hardcoded map for navigation_inst_mdp__1
        # Objects:
        # xpos : {x6,x14,x21,x9};
        # ypos : {y12,y20,y15};
        
        # Grid Adjacency (from non-fluents):
        # y: y12 <-> y15 <-> y20 (South to North)
        # x: x6 <-> x9 <-> x14 <-> x21 (West to East)
        
        self.y_map = ['y12', 'y15', 'y20'] # Sorted S->N
        self.x_map = ['x6', 'x9', 'x14', 'x21'] # Sorted W->E
        
        # Initial State: robot-at(x21,y12)
        # x21 is index 3, y12 is index 0
        self.x_idx = 3
        self.y_idx = 0

        self.curr_x = []
        self.curr_y = []
        
        self.goal = ('x6', 'y15')

    def get_obs(self):
        obs = []
        
        # robot-at(x, y)
        # Technically in RDDL we should enumerate all false ones too? 
        # Usually RDDL assumes closed world assumption for state-fluents if they are default false.
        # But PROST might expect the true ones explicitly.
        
        self.curr_x = self.x_map[self.x_idx]
        self.curr_y = self.y_map[self.y_idx]
        
        obs.append(KeyValue(f"robot-at({self.curr_x}, {self.curr_y})", "true"))

        return obs

    def step(self, action_name, action_params):
        rospy.loginfo(f"Executing: {action_name}")


        if action_name == "move-north":
            if self.y_idx < len(self.y_map) - 1:
                self.y_idx += 1
        elif action_name == "move-south":
            if self.y_idx > 0:
                self.y_idx -= 1
        elif action_name == "move-east":
            if self.x_idx < len(self.x_map) - 1:
                self.x_idx += 1
        elif action_name == "move-west":
            if self.x_idx > 0:
                self.x_idx -= 1

        
        self.curr_x = self.x_map[self.x_idx]
        self.curr_y = self.y_map[self.y_idx]
        rospy.loginfo(f"Robot at: ({self.curr_x}, {self.curr_y})")

def run_demo():
    rospy.init_node('navigation_demo_client')
    
    rospy.wait_for_service('/prost_bridge/start_planning')
    rospy.wait_for_service('/prost_bridge/submit_observation')
    
    start_planning = rospy.ServiceProxy('/prost_bridge/start_planning', StartPlanning)
    submit_obs = rospy.ServiceProxy('/prost_bridge/submit_observation', SubmitObservation)
    
    with open( "navigation_mdp.rddl", 'r') as f:
        domain = f.read()
    with open("navigation_inst_mdp__1.rddl", 'r') as f:
        instance = f.read()
        
    rospy.loginfo("Sending StartPlanning request...")
    resp = start_planning(domain, instance, 60)
    if not resp.success:
        rospy.logerr("StartPlanning failed")
        return

    sim = NavigationSim()
    
    # Initial State
    count = 0
    obs = sim.get_obs()
    rospy.loginfo("Sending Initial State...")
    act_resp = submit_obs(obs, 0.0)
    

    horizon = 40
    rospy.loginfo(act_resp.action_name)
    while act_resp.action_name != "ROUND_END" and count < horizon:
        sim.step(act_resp.action_name, act_resp.action_params)
        obs = sim.get_obs(count)
        
        # Reward: 0 if goal reached, -1 otherwise (as per domain file)
        # reward = [sum_{?x : xpos, ?y : ypos} -(GOAL(?x,?y) ^ ~robot-at(?x,?y))]; 
        # So if NOT at goal, reward is -1. If at goal, reward is 0 (assuming goal is unique).
        
        reward = -1.0
        if (sim.curr_x, sim.curr_y) == sim.goal:
            reward = 0.0
            rospy.loginfo("GOAL REACHED! (Stopping!!!)")
            break
            
        act_resp = submit_obs(obs, reward)
        count += 1
        rospy.loginfo(act_resp.action_name)


if __name__ == "__main__":
    run_demo()
