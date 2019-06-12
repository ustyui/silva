#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Tue Jan  8 12:04:44 2019
blink function

@author: ustyui
"""

import rospy
from silva_beta.msg import Evans

import os, threading
from numpy import random as rd
import numpy as np
from math import *

from sensor_msgs.msg import Joy

import time

### environment variables ###
_RATE = 50 # ros rate
_driveunits = 5 # drivable units of the robot

### blink function ###
class eyelid():
    # input physical and mental state
    def __init__(self, physical=0, mental=0):
        
        self._physical = physical
        self._mental = mental
        self._tiredness = 0
        self._eyelid = [0, 0]
        self._count = 0
        self._payload = [0,0,0,0,0]
        self._bias = 0
        self._flag = 'noblink'
        # message publish
        self._pub_msg = Evans()
        self._pub_msg_a = Evans()
        self._pub_msg_b = Evans()
        
        # physical gives a bias on the rythem decider
        self._decider = physical + mental
        # mental decides the period and the amplitude
        
        # publishers
        self.pub = rospy.Publisher('/silva/idle_local/intention', Evans, queue_size=10)
        self.pub_a = rospy.Publisher('/silva/auto_local/ch0', Evans, queue_size=10)
        #self.pub_b = rospy.Publisher('/silva/reflex_local/ch0', Evans, queue_size=10)
        
        # subscribers
        self.sub_auto = rospy.Subscriber('/silva/auto_local/intention', Evans, self.state_cb)
        self.sub_joy = rospy.Subscriber('/joy', Joy, self.joy_cb)
        
        
    ### callback functions ###
        
    def state_cb(self, msg):
        
        if msg.seq == 4 and msg.msgid == 10:
            self._physical = msg.payload[0]
            self._mental = msg.payload[1]
            
    def joy_cb(self,msg):
        self._physical = 0.5*(msg.axes[0] + 1)
        
    # rythem function
    def rythem_d(self, _count):
        self._decider = self._physical + self._mental
        if self._flag == 'blink':
            self._eyelid = [1, 1]
        elif self._decider >= 0 and self._decider <0.25:
            # constant, vary from 0 to 1
            self._eyelid[0] = 0 
            self._eyelid[1] = 0
            
        elif self._decider >= 0.25 and self._decider < 0.5:
            
            # block wave
            if _count <= 0.5*_RATE + self._bias:
                self._eyelid[0] = 0.7 + _count*0.001
                self._eyelid[1] = 0.7 + _count*0.001
            else:
                self._eyelid[0] = 0
                self._eyelid[1] = 0
            
        elif self._decider >= 0.5 and self._decider < 0.75:
            # sin wave
            if _count <= 3*_RATE + self._bias:
                self._eyelid[0] = sin(pi/(3*_RATE+self._bias)*_count)
                self._eyelid[1] = self._eyelid[0]
            else:
                self._eyelid[0] = 0
                self._eyelid[1] = 0
                
        elif self._decider >= 0.75:
            # tanh
            if _count <= 3*_RATE + self._bias:
                self._eyelid[0] = 0.9*np.tanh(_count/(150.0+self._bias))
                self._eyelid[1] = self._eyelid[0]
                
            else:
                self._eyelid[0] = 0
                self._eyelid[1] = 0
            
        # multiply, upper scale:1500 lower scale: 1000
        self._payload[1] = int(150 * self._eyelid[0])
        self._payload[2] = int(100 * self._eyelid[1])
            
    ## blink output            
    def blink(self, gene, run_event):
        while run_event.is_set() and not rospy.is_shutdown():
            
            # generate a random time
            gain_k = 2.5
            gain_l = 3.0
            threshold_blink = 3.0
            
            blink_interval = gain_k + gain_l *rd.rand()
            
            if blink_interval < threshold_blink:
                new_blink_interval = log(blink_interval, 5)
                blink_interval = new_blink_interval
                
            if blink_interval > 6.9:
                blink_interval = 7.8
                
            # delay it and blink
            print(blink_interval)
            
            # blink!
            self._flag = 'blink'
            time.sleep(0.1)
            self._flag = 'stopblink'
            # how to blink?
            time.sleep(blink_interval)

            
        
    def make_message(self):
        #make message
        self._pub_msg.header.stamp = rospy.Time.now()
        self._pub_msg.seq = 1
        self._pub_msg.name = 'headl'
        self._pub_msg.msgid = 1
        self._pub_msg.payload = self._payload
        return None
        
    def make_message_a(self):
        self._pub_msg_a.header.stamp = rospy.Time.now()
        self._pub_msg_a.seq = 1
        self._pub_msg_a.name = 'headl'
        self._pub_msg_a.msgid = 4
        self._pub_msg_a.payload = self._payload        
    
    def make_message_b(self):
        self._pub_msg_b.header.stamp = rospy.Time.now()
        self._pub_msg_b.seq = 2
        self._pub_msg_b.name = 'headl'
        self._pub_msg_b.msgid = 1
        self._pub_msg_b.payload = self._payload      
        
    def start(self):
        rospy.loginfo("BLINK")
        
        loop_rate = rospy.Rate(_RATE)
        
        # signal flag for running threads
        run_event = threading.Event()
        run_event.set()
        
        # thread that changes blinks
        move_blink = threading.Thread(target = self.blink, args = \
        (2, run_event))
        
        move_blink.start()
        
        while not rospy.is_shutdown():
            self._count+=1
            # print self._count
            # do rythem
            self.rythem_d(self._count)
            #print self._physical
            print self._eyelid
            
            # make message
            self.make_message()
            self.make_message_a()
            #self.make_message_b()
            
            # publish
            self.pub.publish(self._pub_msg)
            self.pub_a.publish(self._pub_msg_a)
            #self.pub_b.publish(self._pub_msg_b)
            if self._count > 10 * _RATE: #every 10 seconds
                self._count = 0
                self._bias = int(2*_RATE*rd.rand()) #bias within 2 seconds
            loop_rate.sleep()


if __name__ == "__main__":
    eyemotion = eyelid()
    
    nh = rospy.init_node("eye_motion")
    
    eyemotion.start()