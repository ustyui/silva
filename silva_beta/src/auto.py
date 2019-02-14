#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Thu Nov 15 15:59:58 2018
# Auto Motion
# As the parent liabary of other 4 motion filters.
# MF receives intentions message, and mask them to the final publish message.

@author: ustyui
"""

import rospy
from silva_beta.msg import Evans

import os, threading
import numpy as np

import transformations as tform
# import numpy as np

### environment variables ###

_RATE = 30 # ros rate

# TODO: change this to a file load function
seq_of_jointname = {'neck':0,
                    'arml':1,
                    'armr':2,
                    'handl':3,
                    'handr':4,
                    'headl':5,
                    'headc':6,
                    'headr':7,
                    'hip':8,
                    'wheel':9}

# initialize
class poseblock():
    def __init__(self):

        # global variables
        self._rel = [] # relative value
        self._bias =[[],[],[],[]] # bias value for subposition
        self._payload = [] # final payload
        self._default = [] # origin default

        # messages
        self._default_rec = Evans()
        self._pub_msg = Evans()
        self._intention = Evans()
        
        # publishers
        self.pub = rospy.Publisher('/silva/joint_local/auto', Evans, queue_size=10)

        # subscribers
        self.sub_ch0 = rospy.Subscriber('/silva/auto_local/ch0', Evans, self.ch0_cb)    # ch0 for blink message (headl)
        self.sub_ch1 = rospy.Subscriber('/silva/auto_local/ch1', Evans, self.ch1_cb)    # ch1 for realsense (neck)
        self.sub_ch2 = rospy.Subscriber('/silva/auto_local/ch2', Evans, self.ch2_cb)    # ch2 for rplidar (neck)
        self.sub_ch3 = rospy.Subscriber('/silva/auto_local/ch3', Evans, self.ch3_cb)    # ch3 for  
        
        self.sub_default = rospy.Subscriber('/silva/joint_local/default', Evans, self.default_cb)
        
        #init the default message
        tform.set_zeros(self._default)
        tform.set_zeros(self._rel)  # set zeros of rel
        for i in range(len(self._bias)):
            tform.set_zeros(self._bias[i]) # set zeros of bias
    
    
    ### callback functions ###
    def decision_cb(self, msg):
        return None
    
    ## multichannel callback ##
 
    def ch0_cb(self, msg):  
        _cut = seq_of_jointname[msg.name]        # cut method : from where
        _payload = msg.payload                   # get the payload
        for _idx in range (0, len(_payload)):
            self._bias[0][_cut*5 + _idx] = _payload[_idx] # store payload in bias
                
    def ch1_cb(self, msg):
        _cut = seq_of_jointname[msg.name]        
        _payload = msg.payload                  
        for _idx in range (0, len(_payload)):
            self._bias[1][_cut*5 + _idx] = _payload[_idx] 
            
    def ch2_cb(self, msg):
        _cut = seq_of_jointname[msg.name]        
        _payload = msg.payload                   
        for _idx in range (0, len(_payload)):
            self._bias[2][_cut*5 + _idx] = _payload[_idx] 
            
    def ch3_cb(self, msg):
        _cut = seq_of_jointname[msg.name]        
        _payload = msg.payload                   
        for _idx in range (0, len(_payload)):
            self._bias[3][_cut*5 + _idx] = _payload[_idx] 

    def default_cb(self, msg):
        # callback default
        self._default_rec = msg
        self._default = msg.payload
        
        
    ### merge bias ###
    def set_msg_from_pos(self):
        
        mult_ch = np.array(self._bias)
        self._rel = mult_ch[0] + mult_ch[1] + mult_ch[2] +mult_ch[3]
        # add intentions to default
        self._payload = self._rel
    
        
    # TODO: make the filter
    def joint_filter(self, mask):
        
        # filter the desired pose,
        # the masked pose will act as default
        return None
        
    def start(self):
        rospy.loginfo("silva_AUTO")
        # publish the /joint_local/auto message
        
        loop_rate = rospy.Rate(_RATE)
        
        while not rospy.is_shutdown():
            
            # set message from callback
            self.set_msg_from_pos()
            
            # make the message
            tform.make_message(self._pub_msg, 4, 'auto', 0, self._payload)
            
            # publish the message
            self.pub.publish(self._pub_msg)
            
            # print type(self._payload)
            loop_rate.sleep()
        

if __name__ == "__main__":
    
    # poseblock class, in auto
    Apose = poseblock()
    
    # init nodes
    nh = rospy.init_node("filter_AUTO")    

    Apose.start()
