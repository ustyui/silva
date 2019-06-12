#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Thu Jan 10 15:21:38 2019
tanh(sigmoid) idle function

@author: ustyui
"""

import rospy, rospkg
from silva_beta.msg import Evans

import threading
import numpy as np
from numpy import random as rd
from math import *
import yaml, sys

# read params
rospack = rospkg.RosPack()
param_path = rospack.get_path('ibuki_extra')+'/params/mask_rsigmoid.yaml'
f = open(param_path, "r+")
param_config = yaml.load(f)

# env variables
_RATE = param_config['Rate']
dev_name = sys.argv[1]

params_dev = param_config[dev_name]
WEIGHT_AMP = param_config[dev_name]['enable']

# memory
_sinscope = params_dev['sinscope']
_gotoduty = params_dev['gotoduty']
_motionamp = params_dev['motionamp']
_plt = params_dev['periodlengthtrigger']
_rellower = params_dev['rellower']
_relupper = params_dev['relupper']
_tanhamp = params_dev['tanhamp']
_threshold = params_dev['threshold']

class tanh():
    
    ### initialization ###
    def __init__(self):
        
        self._gene = [0, 0, 0, 0, 0]
        self._count = 0
        self._ct = [1, 1, 1, 1, 1]
        self._timebias = 0
        self._payload = [0,0,0,0,0]
        

        # random variables
        self._length = [0, 0, 0, 0, 0]
        self._startpos = [0.0, 0.0, 0.0, 0.0, 0.0]
        self._starttime = [0.0, 0.0, 0.0, 0.0, 0.0]
        self._sinscope = [0.0, 0.0, 0.0, 0.0, 0.0]
        self._nextpos = [0.0, 0.0, 0.0, 0.0, 0.0]
        self._gototime = [0.0, 0.0, 0.0, 0.0, 0.0]
        self._sinscopeB = [0.0, 0.0, 0.0, 0.0, 0.0]
        
        # values
        self._rel = [0.0, 0.0, 0.0, 0.0, 0.0]
        
        #message publish
        self._pub_msg = Evans()
        
        # publishers
        self.pub = rospy.Publisher('/silva/idle_local/ch0', Evans, queue_size=10)
        self.pub_a = rospy.Publisher('/silva/auto_local/ch0', Evans, queue_size = 10)
        self.pub_s = rospy.Publisher('/silva/slave_local/intention', Evans, queue_size=10)
        # subscribers
        
    ### callback functions ###
    
    ### threading functions ###
    # _amp: time base , _sn: serial number 
    def tanh_idle(self, _amp, _sn, run_event):
        # create a random for time
        rate = rospy.Rate(_RATE)
        while run_event.is_set() and not rospy.is_shutdown():
            # add counter
        
            self._ct[_sn] += 1           
            
            # judge
            "VARIABLE GENERATION, UPDATE EVERY PERIOD"
            if self._ct[_sn] > self._length[_sn]:
                
                # count to zero
                self._ct[_sn] = 0
                
                _interval = _amp + _amp * rd.rand()
                _randval = _interval - _amp
                
                self._length[_sn] = _interval * _RATE # period length
                
                # generate start time, between 0 to _amp
                self._starttime[_sn] = _amp * rd.rand() * _RATE
                
                # generate sin scope, small 
                self._sinscope[_sn] = _sinscope * rd.rand()
                "SIGN, CONTROLED BY REL"
                sgn = np.sign(rd.rand()-0.5 - self._rel[_sn]*0.5)
                
                # generate goto position
                self._nextpos[_sn] = rd.rand()
                
                # generate goto time
                self._gototime[_sn] = (_gotoduty * _randval + (1-_gotoduty)*_randval*rd.rand())*_RATE
                # goto time -> 8 factor
                _tanhf = 8.0/self._gototime[_sn]
                # generate next sin scope
                self._sinscopeB[_sn] = _sinscope * rd.rand()
                
                # init start position with rel
                self._startpos[_sn] = self._rel[_sn]
                
            "WAVE GENERATION, UPDATE EVERY RATE"
            # do cal
            # if counter is below start time
            if self._ct[_sn] < self._starttime[_sn]:
                ## do flat
                self._rel[_sn] = self._startpos[_sn] + sgn * self._sinscope[_sn] * \
                                sin(float(self._ct[_sn])/_RATE)
                                
            # if counter is between starttime and goto time
            elif self._ct[_sn] > self._starttime[_sn] and self._ct[_sn] < (self._starttime[_sn]+self._gototime[_sn]):
                ## do tanh
                _phase = self._ct[_sn] - self._starttime[_sn] 
                ## when enter, transfer value
                if _phase <= 1:
                    self._startpos[_sn] = self._rel[_sn]
                    #print 'sp',self._startpos[_sn]
                else:
                
                    self._rel[_sn] = self._startpos[_sn] + sgn * \
                    _tanhamp*self._nextpos[_sn]*(1 + np.tanh(_tanhf*_phase - 4))
                    #print 'rl',self._rel[_sn]
            elif self._ct[_sn] >= (self._starttime[_sn]+self._gototime[_sn]):
                _phase = self._ct[_sn] - self._starttime[_sn]-self._gototime[_sn]
                if _phase <= 1:
                    self._startpos[_sn] = self._rel[_sn]
                else:
                    self._rel[_sn] = self._startpos[_sn] + sgn * self._sinscopeB[_sn] * \
                                sin(float(self._ct[_sn])/_RATE)
                
            ## check the board
            if self._rel[_sn] >= _relupper:
                self._rel[_sn] = 1.0
            if self._rel[_sn] <= _rellower:
                self._rel[_sn] = -1.0
                
            self._gene[_sn] = _motionamp*self._rel[_sn]
            rate.sleep()
        
    ### make message ###
    def make_message(self, msgid, seq, payload):
        # make message
        self._pub_msg.header.stamp = rospy.Time.now()
        self._pub_msg.seq = seq 
        self._pub_msg.name = dev_name
        self._pub_msg.msgid = msgid
        self._pub_msg.payload = payload
        
    def start(self):
        rospy.loginfo(dev_name)
        
        loop_rate = rospy.Rate(_RATE)
        
        # signal flag for running threads
        run_event = threading.Event()
        run_event.set()
        
        # thread open
    
        move_0 = threading.Thread(target = self.tanh_idle, args = (_plt, 0, run_event))
        
        move_1 = threading.Thread(target = self.tanh_idle, args = (_plt, 1, run_event))
        
        move_2 = threading.Thread(target = self.tanh_idle, args = (_plt, 2, run_event))
        
        move_3 = threading.Thread(target = self.tanh_idle, args = (_plt, 3, run_event))
        
        move_4 = threading.Thread(target = self.tanh_idle, args = (_plt, 4, run_event))
        
        move_0.start()
        move_1.start()
        move_2.start()
        move_3.start()
        move_4.start()
        
        
        while not rospy.is_shutdown():
            
            
            ### main function ###
            
            self._payload[0] = int(WEIGHT_AMP[0]*self._gene[0]) + _threshold[0]
            self._payload[1] = int(WEIGHT_AMP[1]*self._gene[1]) + _threshold[1]
            self._payload[2] = int(WEIGHT_AMP[2]*self._gene[2]) + _threshold[2]
            self._payload[3] = int(WEIGHT_AMP[3]*self._gene[3]) + _threshold[3]
            self._payload[4] = int(WEIGHT_AMP[4]*self._gene[4]) + _threshold[4]
            
            self.make_message(1,1,self._payload)
            self.pub.publish(self._pub_msg)   
            if dev_name == 'headr':
                self.make_message(4,1,self._payload)
                self.pub_a.publish(self._pub_msg)              
            # else, do tanh/ sin move
            print self._gene
            
            loop_rate.sleep()
            
if __name__ == "__main__":
    tanhmotion = tanh()
    
    nh = rospy.init_node(dev_name+"_motion")
    
    tanhmotion.start()
            
            