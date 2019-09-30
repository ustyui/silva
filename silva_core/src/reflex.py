#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Thu Sep 9  16:47:41 2019
reflex node
@author: ustyui
"""
import rospy
from silva_core.msg import Evans
from sensor_msgs.msg import Joy

import numpy as np
import utils, topics
import sys

robot_name = sys.argv[1]

class Poseblock():
    def __init__(self):
        
        self._pub_msg = Evans()
        
        self.pub = rospy.Publisher(topics.com['reflex'], Evans, queue_size = 3)
        #init : sub channels, channel callback subloads
        self.sub = [[]]*len(topics.reflex) 
        self.subload = [[]]*len(topics.reflex) 
        
        while not rospy.is_shutdown():
            if rospy.has_param(robot_name+'/DRIVE_UNITS'):
                break
            rospy.sleep(0.1)
            rospy.loginfo("Waiting for dynamic parameters to be updated...")
        rospy.loginfo("Parameters updated.")

        for i in range(len(self.subload)):
            self.subload[i] = [0] * rospy.get_param(robot_name+'/DRIVE_UNITS')
        
        for i in range(0, len(topics.reflex)):
            
            self.sub[i]=rospy.Subscriber(topics.reflex[i], Evans, self.channel_cb, i)
        
    def channel_cb(self, msg, args): 
        instance = args
        self.subload[instance] = utils.sort_labeled(self.subload[instance], msg, _SEQ_OF_JOINTNAME)
        
    def start(self):
        rospy.loginfo("Silva reflex Rate at "+str(_RATE)+"Hz OK")
        loop_rate = rospy.Rate(_RATE)
        while not rospy.is_shutdown():
            # submixer
            mux = np.array(self.subload)
            summux = mux[0]
            for i in range(1, len(mux)):
                summux = summux + mux[i]
                
            # print summux
            # make message
            utils.make_message(self._pub_msg, 'reflex', 1, 0, summux)
            self.pub.publish(self._pub_msg)
            
            loop_rate.sleep()
            
        

if __name__ == "__main__":
    nh = rospy.init_node("reflex_filter")
    param_config = utils.read_param(robot_name, robot_name)
    
    _SEQ_OF_JOINTNAME = param_config['SequenceOfJoints']
    _RATE = param_config['Rates']['reflex']
    
    Spose = Poseblock()
    
    Spose.start()
