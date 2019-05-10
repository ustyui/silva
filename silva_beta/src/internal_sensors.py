#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Fri May 10 13:30:43 2019
# internal sensor node
# get feed back from FB_thread from mbed firmware.

@author: ibukidev
"""

import rospy
from silva_beta.msg import Evans

import transfromations as tform
import socket, errno
import sys

### console parameters ###
dev_name = sys.argv[1]

### read parameters ###
param_config = tform.read_param()

seq_of_jointname = param_config['SequenceOfJoints']
_RATE = param_config['Rates']['internalsensors']
ip = param_config['IP']
portf = param_config['PORT'][dev_name+'f']

# 9 nodes
if __name__ == "__main__":
### use UDP to read FB_thread except wheel ###
    # vars
    position = ''

    # socket initialization
    fb_client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  
    
    # node init
    rospy.init_node("FB_"+dev_name, anonymous=True)
    rate = rospy.Rate(_RATE)
    
    pub_p = rospy.Publisher('/silva/reflex_local/ch0', Evans, queue_size=10)
    pub_c = rospy.Publisher('/silva/reflex_local/ch1', Evans, queue_size=10)
    
    pub_msg_p = Evans()
    pub_msg_c = Evans()
    
    while not rospy.is_shutdown():
        
        # GET FB_thread by asking 
        fb_client.sendto('h', (ip[dev_name], portf))
        position, addr = fb_client.recvfrom(4096)
        
        payload_p = []
        payload_c = []
        
        # split the position and currents
        pac = position.split(',')
        p_pos = pac[0].split()
        p_cur = pac[1]
        
        for elements in p_pos:
            payload_p.append(int(int(elements)/10))
        for idx in range(0,5):
            payload_c.append(int(p_cur[idx*5:(idx+1)*5]))
            
        # make message
        tform.make_message(pub_msg_p,2,dev_name,3,payload_p)
        tform.make_message(pub_msg_c,2,dev_name,4,payload_c)
        
        # publish
        pub_p.publish(pub_msg_p)
        pub_c.publish(pub_msg_c)
        
        rate.sleep()
# If the value cannot be obtain, try again until get the value

__version = "1.1.0"