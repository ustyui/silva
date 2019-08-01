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
        
        self.

if __name__ == "__main__":
    nh = rospy.init_node("slave_filter")
    param_config = utils.read_param()
    
    _SEQ_OF_JOINTNAME = param_config['SequenceOfJoints']
    _RATE = param_config['Rates']['slave']
    
    Spose = Poseblock()