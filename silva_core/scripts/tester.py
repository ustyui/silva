#!/usr/bin/env python

import rospy
from darwin_gazebo.darwin import Darwin
from silva_beta.msg import Evans

dev_name = 'darwin'

_RATE = 30

seq_of_jointname = {1:'j_shoulder_r',
                    2:'j_shoulder_l',
                    3:'j_pan',
                    4:'j_tilt',
                    5:'j_high_arm_l',
                    6:'j_low_arm_l',
                    7:'j_wrist_l',
                    8:'j_gripper_l',
                    9:'j_high_arm_r',
                    10:'j_low_arm_r',
                    11:'j_wrist_r',
                    12:'j_gripper_r',
                    
                    }

def callback(msg, args):
    instance = args
    # put the fusion payload into the class
    instance._payload = msg.payload



class Joint():
    
    def __init__(self, name ='void'):
        self._name = name
        self._payload = []
        
    def dict_making(self, dic):
        for i in range(0,len(self._payload)):
            dic[seq_of_jointname[i+1]] = self._payload[i]
        
if __name__=="__main__":
    rospy.init_node("darwin_silva")
    rate = rospy.Rate(_RATE)
    
    joint = Joint(dev_name)
    
    sub = rospy.Subscriber('/silva/joint_local/fusion', Evans, callback, joint)
    darwin=Darwin()
    
    motion_dict = {'j_shoulder_r':0,
                   'j_shoulder_l':0,
                   'j_pan':0,
                   'j_tilt':0,
                   'j_high_arm_l':0,
                   'j_low_arm_l':0,
                   'j_wrist_l':0,
                   'j_gripper_l':0,
                   'j_high_arm_r':0,
                   'j_low_arm_r':0,
                   'j_wrist_r':0,
                   'j_gripper_r':0,}
    while not rospy.is_shutdown():
        # make motion dict
        joint.dict_making(motion_dict)
        print motion_dict
        darwin.set_angles(motion_dict)
        
        rate.sleep()

