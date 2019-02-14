#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Thu Jan 17 18:06:36 2019
input: Humanoid Simulation model csv file
output: slave operation message
@author: ustyui
"""
# read csv
import pandas as pd

import rospy,rospkg
from silva_beta.msg import Evans

import threading, sys, time
import numpy as np
import transformations as tform

from sensor_msgs.msg import Joy
### environment variables ###

_RATE = 50  # ros rate
_driveunits = 50
_KDC = 127.32395447
_RES = 0.01999
# TODO: make this mask universal
_MASK = [
        2.5, -2.5, 1, -1, 1,
        -1.3, -1, -1.3, -1.2, -1,
        -1.3, -1, 1.3, -1.2, -1
        ]

_filename = sys.argv[1]
#_filename = 'lookaround'
# open the .csv
rospack = rospkg.RosPack()
csvpath = rospack.get_path('silva_beta')+'/src/csv/'+_filename+'.csv'

df = pd.read_csv(csvpath, delimiter=',')

class csvslave():
    def __init__(self, _df):
        # csv
        self._df =df
        self._timelist = []
        self._motionlist = []
        self._lastmotion = []
        self._payload = []
        self._payload_float = []
        
        self._margin_to_target = []
        self._time_interval = 0.0
        
        #initilization
        tform.set_zeros(self._payload)
        tform.set_zeros(self._payload_float)
        tform.set_zeros(self._lastmotion)
        
        # messages
        self._pub_msg = Evans()
        
        # publishers
        self.pub = rospy.Publisher('/silva/slave_local/hsm', Evans, queue_size = 10)
        
        # subscribers
    ### calculations ###
    def rad2cmd(self):
        # multiply mask first
        for i in range(0,len(_MASK)):
            self._df[self._df.columns[i+1]] = _MASK[i] * self._df[self._df.columns[i+1]]
        
        for i in range(0, len(self._df)):
            # time append
            if (i == 0):
                self._timelist.append(float(self._df.ix[i,0])) # init
            else:
                self._timelist.append(float(self._df.ix[i,0])-float(self._df.ix[i-1,0])) # interval
            
            # listize panda frame
            templist = list(self._df.ix[i,1:])

            self._motionlist.append(templist)
            
    def make_msg_and_pub(self, msgid, seq, payload, publisher):
        # make message
        self._pub_msg.header.stamp = rospy.Time.now()
        self._pub_msg.seq = seq 
        self._pub_msg.name = 'hsmap'
        self._pub_msg.msgid = msgid
        self._pub_msg.payload = payload
        
        # publish message
        publisher.publish(self._pub_msg)
            
    def joint_to_where(self):
        tform.set_zeros(self._margin_to_target)
        
        self._lastmotion = self._payload # current motion
        for i in range(len(self._motionlist)):
            if (i==0):
                pass
            else:
                # save lastmotion
                       
                self._margin_to_target = self._lastmotion # image the motion
                self._margin_to_target = np.array(self._motionlist[i])-np.array(self._lastmotion)  # compare margin
                
                lines_nb = int(self._timelist[i]/_RES) #times of given res frequency

                step_to_target = _KDC * self._margin_to_target/lines_nb # for each frequency, linear subposition
                
                # add process
                for lines in range(0, lines_nb):
                    self._payload_float = self._payload_float + step_to_target
                    self._payload = list(np.array(self._payload_float, dtype = 'int32'))
                    
                    self.make_msg_and_pub(3,2, self._payload, self.pub)
                    
                    #print(self._payload)
                    time.sleep(_RES)
                #print(self._payload_float)
                self._lastmotion = self._motionlist[i]
        return None

    def start(self):
        rospy.loginfo("HUMANOID SIMULATION MODEL MAPPING")
        loop_rate = rospy.Rate(_RATE)
        
        self.rad2cmd()
        self.joint_to_where()
        
        #rospy.spin()
        

## generate command by comparing it with maximum minimum

# send message


if __name__ == "__main__":
    
    # pose class
    hsm = csvslave(df)
    nh = rospy.init_node("HSM_csv")    
    hsm.start()
    # init nodes

    
    