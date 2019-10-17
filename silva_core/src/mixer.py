#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Wed Jul 31 12:14:14 2019
Motion Mixer
@author: ustyui
"""

import rospy, rospkg
from silva_core.msg import Evans
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Joy
import sys, threading, yaml
import numpy as np
import modules.utils as utils
import modules.topics as topics

robot_name = sys.argv[1]

class Mixer():

    
    def __init__(self, dev_name):
        # _name: dev_name
        self._name = dev_name
        # load init map file with the name robotname.map
        self._default, __void = utils.load_map(self._name, self._name)
        
        # joint array in driveunits numbers
        self._joint_idle = np.zeros(_DRIVEUNITS)
        self._joint_reflex = np.zeros(_DRIVEUNITS)
        self._joint_slave = np.zeros(_DRIVEUNITS)
        self._joint_auto = np.zeros(_DRIVEUNITS)
        
        # load limit configuration file of the robot with the name limit_robotname.map
        self._joint_min, self._joint_max = np.array(utils.load_map(self._name, 'limit'))
        self._temp = np.array([0, 0, 2, 0])
        self._covs = self._temp
        self._hw_adj = 0.0
        
        self._default_msg = Evans()
        self._state_msg = Float32MultiArray()
        self._pub_msg = Evans()
        self._state_msg.data = [0.0, 0.0, 1.0, 0.0]
        self._mhn = _DRIVEUNITS/2
        self._mask_h = np.zeros(self._mhn)
        if _DRIVEUNITS%2 != 0:
            self._mask_l = np.zeros(_DRIVEUNITS/2 + 1)
            self._mln = self._mhn + 1
        else:
            self._mask_l = np.zeros(_DRIVEUNITS/2)
            self._mln = self._mhn
        self._mask = []
                
        
        # publishers
        self.pub = rospy.Publisher(topics.com['mixer'], Evans, queue_size = 5)
        self.pub_d = rospy.Publisher(topics.com['default'], Evans, queue_size = 2 )
        self.pub_s = rospy.Publisher(topics.com['states'], Float32MultiArray, queue_size = 2)
        
        # subscribers
        self.sub_joy = rospy.Subscriber(topics.com['joy'], Joy, self.joy_cb)
        self.sub_idle = rospy.Subscriber(topics.com['idle'], Evans, self.joint_idle_cb)
        self.sub_reflex = rospy.Subscriber(topics.com['reflex'], Evans, self.joint_reflex_cb)
        self.sub_slave = rospy.Subscriber(topics.com['slave'], Evans, self.joint_slave_cb)
        self.sub_auto = rospy.Subscriber(topics.com['auto'], Evans, self.joint_auto_cb)
        

        
    def move_pub_d(self, rate, pub, msg, run_event):
        rate = rospy.Rate(rate)
        while run_event.is_set() and not rospy.is_shutdown():
            if isinstance(msg, Evans):
                msg.header.stamp = rospy.Time.now()
            pub.publish(msg)
            rate.sleep()
    
    def joy_cb(self, msg):
        # main joystick controller, if there are many joy messages
        if msg.header.frame_id == 'main':
            _axes = msg.axes
            self._temp = np.array([_axes[4]+1.0, _axes[5]+1.0, _axes[6]+1.0, _axes[7]+1.0])
            if sum(self._temp) == 0.0:
                # default slave, on demand
                self._temp = np.array([0,0,2,0]) 
    
    def joint_idle_cb(self, msg):
        self._joint_idle = np.array(msg.payload)
    
    def joint_reflex_cb(self, msg):
        self._joint_reflex = np.array(msg.payload)
    
    def joint_slave_cb(self, msg):
        self._joint_slave = np.array(msg.payload)
    
    def joint_auto_cb(self, msg):
        self._joint_auto = np.array(msg.payload)
            
    
    def start(self):
        loop_rate = rospy.Rate(_RATE)
        self._default_msg = utils.make_message(self._default_msg, 'default', 0, 0, self._default)
        
        run_event = threading.Event()
        run_event.set()
        move_t = threading.Thread(target = self.move_pub_d, args = (_TRATE, self.pub_d, self._default_msg, run_event))
        move_state = threading.Thread(target = self.move_pub_d, args = (_TRATE, self.pub_s, self._state_msg, run_event))
        move_t.start()
        move_state.start()
        
        while not rospy.is_shutdown():
            self.fusion()
            self.pub.publish(self._pub_msg)
            loop_rate.sleep()     
    
    def fusion(self):
        # designate means: _sum = 2.0 because the scale of midi slider is 2.0
        _sum = 2.0
        if (rospy.get_param('WEIGHT_HW_ADJUST') == 1): 
            self._covs = self._temp/_sum
        else:
            self._covs = [rospy.get_param('WEIGHT_IDLE'),
                          rospy.get_param('WEIGHT_REFLEX'),
                          rospy.get_param('WEIGHT_SLAVE'),
                          rospy.get_param('WEIGHT_AUTO')]
        # print self._joint_auto
        # print self._joint_slave
        self._jointmeans = np.array(self._covs[0]*self._joint_idle+ self._covs[1]*self._joint_reflex +\
                                self._covs[2]*self._joint_slave + self._covs[3]*self._joint_auto)
        # mask
        if (rospy.has_param('JOINT_MASK_H') and rospy.has_param('JOINT_MASK_L')):
            temp_mask = format(rospy.get_param('JOINT_MASK_H'), '#0'+str(self._mhn+2)+'b')
            self._mask_h = np.array([int(m) for m in temp_mask[2:]])
            temp_mask = format(rospy.get_param('JOINT_MASK_L'), '#0'+str(self._mln+2)+'b')
            self._mask_l = np.array([int(m) for m in temp_mask[2:]])
        else:
            rospy.logwarn_once("Mixer: Some dynamic parameters cannot be set for some reasons. This would lead to a constant output.")
        self._mask = np.concatenate((self._mask_h, self._mask_l), axis=0)
        
        self._jointmeans = self._mask * self._jointmeans
        
        self._payload = np.add(self._default, self._jointmeans)
        # compare bounds#
        self._payload = list(np.clip(self._payload, self._joint_min, self._joint_max))
        
        utils.make_message(self._pub_msg, 'fusion', 0, 0, self._payload)
        # update state message?
        
        return None
        
if __name__ == "__main__":
    
    nh = rospy.init_node("Mixer")
    param_config = utils.read_param(robot_name, robot_name)
    if (param_config == 0):
        rospy.logfatal("Mixer: Fail to load robot parameters. Check /params folder to ensure the yaml file are correctly set. Code 10")
    # clean dyna parameters
    
    # static parameters
    dev_name = param_config['Config']['robotname']
    _DRIVEUNITS = param_config['Config']['driveunits']
    _RATE = param_config['Rates']['mixer']
    _TRATE = param_config['Rates']['broadcast']
    
    # rosparam set: all init set in mixer node
    dyna_params = utils.read_param(dev_name, 'dyna_params')    

    rospy.set_param('JOINT_MASK_H', dyna_params['JOINT_MASK_H'])
    rospy.set_param('JOINT_MASK_L', dyna_params['JOINT_MASK_L'])
    rospy.set_param('WEIGHT_HW_ADJUST', dyna_params['WEIGHT_HW_ADJUST'])
    rospy.set_param('WEIGHT_IDLE', dyna_params['WEIGHT_IDLE'])
    rospy.set_param('WEIGHT_REFLEX', dyna_params['WEIGHT_REFLEX'])
    rospy.set_param('WEIGHT_SLAVE', dyna_params['WEIGHT_SLAVE'])
    rospy.set_param('WEIGHT_AUTO', dyna_params['WEIGHT_AUTO'])
    rospy.set_param('ROBOT_NAME', dyna_params['ROBOT_NAME'])
    rospy.set_param(robot_name+'/DRIVE_UNITS', _DRIVEUNITS)
    
    Mbus = Mixer(dev_name)
    rospy.loginfo("Silva Core Mixer Rate at "+str(_RATE)+"Hz OK")
    Mbus.start()
