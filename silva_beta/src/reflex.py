#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Sun Jan  6 23:24:36 2019
# reflex Motion
# library succeed from Auto
# TODO: add feedback to reflex
@author: ustyui
"""

import rospy
from silva_beta.msg import Evans

import os, threading
import numpy as np

import transformations as tform
from math import *
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
        self._rel = []
        self._bias = [[],[],[]]
        self._payload = []
        self._default = []
        
        self._neck = [0.0, 0.0, 0.0, 0.0, 0.0]
        self._armr = [0.0, 0.0, 0.0, 0.0, 0.0]
        self._arml = [0.0, 0.0, 0.0, 0.0, 0.0]
        self._hip = [0.0, 0.0, 0.0, 0.0, 0.0]
        
        # messages
        self._default_rec = Evans()
        self._pub_msg = Evans()
        self._intention = Evans()
        
        # publishers
        self.pub = rospy.Publisher('/silva/joint_local/reflex', Evans, queue_size=10)
        
        # subscribers
        self.sub_int = rospy.Subscriber('/silva/reflex_local/feedback', Evans, self.intention_cb)
        self.sub_default = rospy.Subscriber('/silva/joint_local/default', Evans, self.default_cb)
        # 2 channels
        self.sub_ch0 = rospy.Subscriber('/silva/reflex_local/ch0', Evans, self.ch0_cb)
        self.sub_ch1 = rospy.Subscriber('/silva/reflex_local/ch1', Evans, self.ch1_cb)
        self.sub_ch2 = rospy.Subscriber('/silva/reflex_local/ch2', Evans, self.ch2_cb)
        
        tform.set_zeros(self._default)
        tform.set_zeros(self._rel)
        for i in range(len(self._bias)):
            tform.set_zeros(self._bias[i]) # set zeros of bias

    ### callback functions ###

    def intention_cb(self, msg):
        self._intention = msg

        # cut method : from where
        _cut = seq_of_jointname[msg.name]
        
        # get the payload
        _payload = msg.payload
        _x = msg.payload[0]
        
        # if the msgid = 1 then to relative
        if msg.msgid == 1:
            
            # place payload to the cut place
            for _idx in range (0, len(_payload)):
                self._bias[0][_cut*5 + _idx] = _payload[_idx]
                
        # use this message to make the upperbody motion
        
        self._bias[1][seq_of_jointname['arml']*5] = -28
        self._bias[1][seq_of_jointname['arml']*5+2] = 50 + 60 * sin(pi*_x/180) #45
        self._bias[1][seq_of_jointname['arml']*5+3] = -130
        
        self._bias[1][seq_of_jointname['armr']*5] = +28
        self._bias[1][seq_of_jointname['armr']*5+2] = -45 + 60* sin(pi*_x/180) #45
        self._bias[1][seq_of_jointname['armr']*5+3] = 130
        
        self._bias[1][seq_of_jointname['hip']*5+1] = -5 * sin(pi*(_x+21)/180)
        self._bias[1][seq_of_jointname['hip']*5+2] = 20 * sin (pi*_x/180) #20
        
        self._bias[1][seq_of_jointname['neck']*5+0] = -10 + 140*sin(pi*_x/180) #120
        self._bias[1][seq_of_jointname['neck']*5+1] = 10 + 140*sin(pi*_x/180)  #120
        
        self._bias[1][seq_of_jointname['wheel']*5] = 100*sin(pi*_x/180)
        
        
        # make the message of arml, armr, neck, hip
        
        
    def default_cb(self, msg):
        # callback default
        self._default_rec = msg
        self._default = msg.payload
        
    def ch0_cb(self, msg):
        _cut = seq_of_jointname[msg.name]        # cut method : from where
        _payload = msg.payload                   # get the payload
#        for _idx in range (0, len(_payload)):
#            self._bias[1][_cut*5 + _idx] = _payload[_idx] # store payload in bias
                
    def ch1_cb(self, msg):
        _cut = seq_of_jointname[msg.name]        
        _payload = msg.payload                  
#        for _idx in range (0, len(_payload)):
#            self._bias[2][_cut*5 + _idx] = _payload[_idx]    
        
    def ch2_cb(self, msg):    
        _payload = msg.payload                  

        self._bias[2] = _payload 
        
    def set_msg_from_pos(self):
        mult_ch = np.array(self._bias)
        self._rel = mult_ch[2]
        #self._rel = mult_ch[1] + mult_ch[2] # MENTION HERE: feedback not included in the sum!
        # add intentions to default
        self._payload = list(np.array(self._rel))
    
    def start(self):
        rospy.loginfo("silva_REFLEX")
        
        loop_rate = rospy.Rate(_RATE)
        
        while not rospy.is_shutdown():
            
            # set message from callback
            self.set_msg_from_pos()
            
            # make the message
            tform.make_message(self._pub_msg, 2, 'reflex', 0, self._payload)
            
            # publish the message
            self.pub.publish(self._pub_msg)
            
            loop_rate.sleep()
        
        
if __name__ == "__main__":
    
    # poseblock class, in reflex
    Rpose = poseblock()
    
    # init nodes
    nh = rospy.init_node("filter_REFLEX")
    
    # start node
    Rpose.start()