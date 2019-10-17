#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Wed Sep 25 17:16:43 2019
# commu adapter

@author: ustyui
"""
import rospy
from silva_core.msg import Evans
import modules.utils as utils
import modules.topics as topics
import sys

dev_name = sys.argv[1]

class Joint():
    def __init__(self, name = 'void'):
        self._name = name
        self._payload = []
        
def callback(msg, args):
    instance = args
    instance._payload = msg.payload
    
# TODO: raspi living state exceptions

if __name__ == "__main__":
    
    nh = rospy.init_node('Act_'+dev_name, anonymous = True)
    
    param_config = utils.read_param(dev_name, dev_name)
    dyna_params = utils.read_param(dev_name, 'dyna_params')
    
    _SEQ_OF_JOINTNAME = param_config['SequenceOfJoints']
    _RATE = param_config['Rates']['actuator']

    rospy.set_param('CUR_OUT_MASK', dyna_params['CUR_OUT_MASK'])
    
    rate = rospy.Rate(_RATE)    
    
    Raspi_joint = Joint(dev_name)

    sub = rospy.Subscriber(topics.com['mixer'], Evans, callback, Mbed_joint)
    
    