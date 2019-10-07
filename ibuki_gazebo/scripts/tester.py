#!/usr/bin/env python

import rospy
from ibuki_gazebo.ibuki import Ibuki
from silva_core.msg import Evans
import numpy as np

dev_name = 'ibuki'

_RATE = 30

seq_of_jointname ={
	0:'shoulderr_p',
	1:'shoulderl_p',
	2:'neck_r',
	3:'neck_y',
	4:'neck_p',
	5:'arml_r',
	6:'arml_y',
	7:'arml_p',
	8:'wristl_y',
	9:'wristl_r',
	10:'armr_r',
	11:'armr_y',
	12:'armr_p',
	13:'wristr_y',
	14:'wristr_r',
	40:'hip_p',
	41:'hip_r',
	42:'hip_y'
}

defaults = [798, 310, 490, 505, 610, 739, 452, 495, 499, 531, 250, 546, 477, 502,
            483, 488, 480, 496]
margin = [-10, -10, 0, 0, 0, 40, 40, 70, -20, 0, 40, 40, 70, -20, 0, 0, 0, 0]

diffmap = [1, -1, 1, -1, 1, 1, 1, -1, 1, -1, -1, -1, 1, -1, 1, -1, -1, 1]

defaults = list(np.array(defaults)-np.array(margin)*np.array(diffmap))

def callback(msg, args):
    instance = args
    instance._payload = msg.payload
    
class Joint():
    def __init__(self, name='void'):
        self._name = name
        self._payload = []
        
    def dict_making(self, dic):
        for (i, k) in enumerate(seq_of_jointname):
            dic[seq_of_jointname[k]] = (self._payload[k]- defaults[i])*0.33*0.0314*diffmap[i]
            
if __name__ == "__main__":
    rospy.init_node("ibk_simulation")
    rate = rospy.Rate(_RATE)
    
    joint = Joint(dev_name)
    
    sub = rospy.Subscriber('/silva/joint_local/mixer', Evans, callback, joint)
    
    ibuki = Ibuki()
    
    motion_dict = {
	'shoulderl_p':0,
	'shoulderr_p':0,
	'neck_r':0,
	'neck_y':0,
	'neck_p':0,
	'armr_r':0,
	'armr_y':0,
	'armr_p':0,
	'wristr_y':0,
	'wristr_r':0,
	'arml_r':0,
	'arml_y':0,
	'arml_p':0,
	'wristl_y':0,
	'wristl_r':0,
	'hip_p':0,
	'hip_r':0,
	'hip_y':0
	}
    
    while not rospy.is_shutdown():
        
        joint.dict_making(motion_dict)
        
        print(motion_dict)
        
        ibuki.set_angles(motion_dict)
        
        rate.sleep()
        
    
    
