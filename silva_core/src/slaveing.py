#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Thu Aug  1 16:47:41 2019
Slave node
@author: ustyui
"""
import rospy
from silva_core.msg import Evans
from sensor_msgs.msg import Joy

import numpy as np
import utils, topics

class Poseblock():
    def __init__(self):
        
        self._pub_msg = Evans()
        
        self.pub = rospy.Publisher(topics.com['slave'], Evans, queue_size = 3)
        
        chn_num_slave = rospy.get_param('CHN_NUM_SLAVE')
        
        self.sub = [[]]*chn_num_slave        
        
        for i in range(0, chn_num_slave):
            
            self.sub[i] = rospy.Subscriber(topics.subchn['s'+str(i)], Evans, self.channel_cb, i)
        
    def channel_cb(self, msg, args):
        instance = args
        if instance == 
        
    def start(self):
        rospy.loginfo("")
        loop_rate = rospy.Rate(_RATE)
        while not rospy.is_shutdown():
            loop_rate.sleep()
            
        

if __name__ == "__main__":
    nh = rospy.init_node("slave_filter")
    param_config = utils.read_param()
    dyna_params = utils.read_param('dynamic_params')
    
    _SEQ_OF_JOINTNAME = param_config['SequenceOfJoints']
    _RATE = param_config['Rates']['slave']
    
    # rosparam set
    rospy.set_param('CHN_NUM_SLAVE', dyna_params['CHN_NUM_SLAVE'])
    
    Spose = Poseblock()
    
    Spose.start()