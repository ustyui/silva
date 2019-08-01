#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Thu Aug  1 13:30:09 2019
# actuators
# subscriber message name: /fusion
@author: ustyui
"""
import rospy
from silva.msg import Evans
import utils

import socket, errno, sys, threading

dev_name = sys.argv[1]

class Joint(dev_name):
    def __init__(self):
        self._payload = []
        self._pub_msg = Evans()
        
def callback(msg, args):
    instance = args
    _cut = _SEQ_OF_JOINTNAME[instance._name]
    # instance._seq = msg.seq
    instance._payload = msg.payload[_cut*5:(_cut+1)*5]

if __name__ == "__main__":
    
    motorsock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    nh = rospy.init_node('Act_'+dev_name, anonymous = True)
    param_config = utils.read_param()
    
    _SEQ_OF_JOINTNAME = param_config['SequenceOfJoints']
    _RATE = param_config['Rates']['jointinterface']
    _IP = param_config['IP']
    _PORT = param_config['PORT']
    
    rate = rospy.Rate(_RATE)    
    
    Mbed_joint = Joint(dev_name)
    
    sub = rospy.Subscriber('/silva/joint_local/mixer', Evans, callback, Mbed_joint)
    
    while not rospy.is_shutdown():
        embed_msg = utils.merge(Mbed_joint.payload)
        if (embed_msg == 0):
            rospy.logfatal_once("Acutator: Encoded Embed Command Overflow : Code 11")
        try:
            motorsock.sendto(embed_msg, (_IP[dev_name], _PORT[dev_name]))
        except socket.error as error:
                        
            
            
            
            
            