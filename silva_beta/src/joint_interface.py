#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Tue Nov 13 11:19:56 2018
# joint interface
# subscribe message name: /fusion,
# devide it according to different device names.
@author: ustyui
"""

import rospy, rospkg
from silva_beta.msg import Evans

import transformations as tform

import numpy as np
import socket, errno
import sys
import threading

### read parameters ###
param_config = tform.read_param()

seq_of_jointname = param_config['SequenceOfJoints']
_RATE = param_config['Rates']['jointinterface']
ip = param_config['IP']
port = param_config['PORT']

# import sensor_msgs
"-------------------------------------------------------------parameter input"
dev_name = sys.argv[1]

"-------------------------------------------------------------global functions"
def callback(msg, args):
    
    instance = args
    
    # check the device name and find cut
    _cut = seq_of_jointname[instance._name]
           
    instance._seq = msg.seq
    instance._msgid = msg.msgid

    _payload = msg.payload[_cut*5:(_cut+1)*5]
        
    instance._payload = _payload
        
"------------------------------------------------------------------joint class"

class Joint():

    def __init__(self, name = 'void', dev = 'mbed', withfb = False, silence = False):
        self._name = name
        self._dev = dev
        self._withfb = withfb
        self._silence = silence
        self._msgid = 0
        self._seq = 0
        
        self._payload = []
        self._payload_p = []
        self._payload_c = []       
        self._payload_w = [0,0,0,0,0]
        
        self._position = ''
        self._current = ''
        self._pub_msg = Evans()
        self._pub_msg_p = Evans()
        self._pub_msg_c = Evans()
        
        self.tmp = [0,0,0,0,0]
        
        
"------------------------------------------------------------------main func"

def mbed_cb(_sock, _sockb, _str, run_event, cls):
    # send hello
    rate = rospy.Rate(_RATE)
    _flag = 0
    # which device?
    if dev_name == 'arml':
        _port = 11014
        _flag = 1
#    elif dev_name == 'armr':
#        _port = 10022
#        _flag = 0
    elif dev_name == 'wheel':
        _port = 10019
        _flag = 2
        
    while run_event.is_set() and not rospy.is_shutdown():
        if _flag == 1:
            # TODO: add timeout?
            _sock.sendto(_str, (ip[dev_name], _port))
            cls._position, addr_rt =  _sock.recvfrom(4096)
            
            templist = []
            templista = []
            
            # split the position and curretn
            pac = cls._position.split(',')
            p_pos = pac[0].split()
            p_cur = pac[1]

            for elements in p_pos:
                templista.append(int(int(elements)/10))
            cls._payload_p = templista
            for index in range(0,5):
                templist.append(int(p_cur[index*5:(index+1)*5]))
            cls._payload_c = templist
            
        if _flag == 2:
            _sock.sendto(_str, (ip[dev_name], _port))
            cls._position, addr_rt =  _sock.recvfrom(1024)
            cls.tmp = cls._position.split('a') # fake wheel payload
            cls._payload_w = [int(cls.tmp[1]),int(cls.tmp[2]),0,0,0]
            
#            
        rate.sleep()
        
def make_message(seq, name, msgid, payload):
    # make message
    msg = Evans()
    
    msg.header.stamp = rospy.Time.now()
    msg.seq = seq
    msg.name = name
    msg.msgid = msgid
    msg.payload = payload

    return msg
        

if __name__ == "__main__":
    
    "node, UDP init"
    motorsock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rtclient = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    cur_client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)    
    
    rospy.init_node('JI_'+dev_name, anonymous = True)
    rate = rospy.Rate(_RATE)
    

    
#---------------------------------------------------------------------------    
    "define this joint"
    
    joint = Joint(dev_name)
#---------------------------------------------------------------------------     
    "read Evans message and write into the joint"   
    
    sub = rospy.Subscriber('/silva/joint_local/fusion', Evans, callback, joint)
    
    pub = rospy.Publisher('/silva/reflex_local/feedback', Evans, queue_size =10)
    pub_p = rospy.Publisher('/silva/reflex_local/ch0', Evans, queue_size=10)
    pub_c = rospy.Publisher('/silva/reflex_local/ch1', Evans, queue_size=10)
    
    
    "thread"    
    run_event = threading.Event()
    run_event.set()
    move_t = threading.Thread(target = mbed_cb, args = \
    (rtclient,cur_client,  'hello', run_event, joint))
    move_t.start()
    
#---------------------------------------------------------------------------     
    while not rospy.is_shutdown():
        
        "judge msg id"
        if joint._seq == 0:   # write only
        
            "generate one time message"    
            otm = tform.merge(joint._payload)
            
#---------------------------------------------------------------------------            
            try:
                "UDP send launch"
                motorsock.sendto(otm, (ip[dev_name], port[dev_name]))
                
                
            except socket.error as error:
                if error.errno == errno.ENETUNREACH:
                    rospy.WARN("connection to mbed lost.")
                else:
                    raise
        if dev_name == 'wheel':
            pub_msg = make_message(2, dev_name, 2, joint._payload_w)
            pub.publish(pub_msg)
            
        # make some message
        tform.make_message(joint._pub_msg_p,2,dev_name,3, joint._payload_p)
        tform.make_message(joint._pub_msg_c,2,dev_name,4, joint._payload_c)
        
        pub_p.publish(joint._pub_msg_p)
        pub_c.publish(joint._pub_msg_c)

#---------------------------------------------------------------------------         
        rate.sleep()

    move_t.join()
    
    

__version = "1.0.1"