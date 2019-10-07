#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Thu Aug  1 13:30:09 2019
# actuators
# subscriber message name: /fusion
@author: ustyui
"""
import rospy
from silva_core.msg import Evans
import utils, topics

import socket, errno, sys, threading, os

dev_name = sys.argv[1]

class Joint():
    def __init__(self, name = 'void'):
        self._name = name
        self._payload = []
        self._pub_msg = Evans()
        
        # for encoder feedbacks
        self._position = ''
        self._payload_w = [0, 0, 0, 0, 0]
        self._tmp = self._payload_w
        
        self._network_flag = 0
        
def callback(msg, args):
    instance = args
    _cut = _SEQ_OF_JOINTNAME[instance._name]
    # instance._seq = msg.seq
    instance._payload = msg.payload[_cut*5:(_cut+1)*5]

def mbed_cb(_sock, _str, run_event, cls):

    rate = rospy.Rate(_RATE)
    while run_event.is_set() and not rospy.is_shutdown():

        _sock.sendto(_str, (_IP[dev_name], 10019))
        if (_sock.recv != None):
            cls._position, addr_rt =  _sock.recvfrom(1024)
            cls.tmp = cls._position.split('a') # fake wheel payload
            cls._payload_w = [int(cls.tmp[1]),int(cls.tmp[2]),0,0,0]
        else:
            rospy.logwarn_throttle(1, "No receive from actuators. Code: 1")
        
        rate.sleep()

def mbed_living_state(run_event, cls):
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        response = os.system("ping -c 1 " +_IP[dev_name]+" -w 1 >/dev/null 2>&1")
        if response != 0:
            cls._payload_w = [0, 0, 0, 0, 0]
            cls._network_flag = 99
            rospy.logwarn(dev_name+": Cannot connect to mbed by ethernet. Check the connections. Code 2")
        else:
            if cls._network_flag == 99:
                cls._network_flag = 0
                rospy.loginfo(dev_name+": Network re-established.")
            else:
                pass
            
        rate.sleep()
    
    

if __name__ == "__main__":
    
    motorsock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    recallsock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    nh = rospy.init_node('Act_'+dev_name, anonymous = True)
    param_config = utils.read_param()
    dyna_params = utils.read_param(dev_name, 'dyna_params')
    
    _SEQ_OF_JOINTNAME = param_config['SequenceOfJoints']
    _RATE = param_config['Rates']['actuator']
    _IP = param_config['IP']
    _PORT = param_config['PORT']
    
    rospy.set_param('CUR_OUT_MASK', dyna_params['CUR_OUT_MASK'])
    
    rate = rospy.Rate(_RATE)    
    
    Mbed_joint = Joint(dev_name)
    
    sub = rospy.Subscriber(topics.com['mixer'], Evans, callback, Mbed_joint)
    
    # if wheel args
    if dev_name == 'wheel':
        # reduce queue size for acceleration
        pub = rospy.Publisher(topics.feedback['wheelencoder'],Evans, queue_size = 2)
        pub_msg = Evans()
        
        run_event = threading.Event()
        run_event.set()
        move_t = threading.Thread(target = mbed_cb, args = \
        (recallsock, 'h', run_event, Mbed_joint))
        move_t.start()
    
    rospy.loginfo("Silva "+dev_name+" Actuators at "+str(_RATE)+"Hz OK")
    
    # check living state of ethernet
    run_event_check = threading.Event()
    run_event_check.set()
    monitor_t = threading.Thread(target = mbed_living_state, args = \
                                 (run_event_check, Mbed_joint))
    monitor_t.start()
    
    while not rospy.is_shutdown():
        embed_msg = utils.merge(Mbed_joint._payload)
        # print embed_msg 
        if (embed_msg == 0):
            rospy.logfatal_once("Acutator: Encoded Embed Command Overflow : Code 11")
            break
        try:
            motorsock.sendto(embed_msg, (_IP[dev_name], _PORT[dev_name]))
        except socket.error as error:
            if error.errno == errno.ENETUNREACH:
                rospy.logfatal_once("Actuator: Connection to mbed lost. Check your ethernet connections. Code 99")
            else:
                raise
        rate.sleep()
    if (dev_name == 'wheel'):
        move_t.join()
        
__version = "2.0.0"
                        
            
            
            
            
            