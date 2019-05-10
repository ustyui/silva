#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Tue Nov 13 11:19:56 2018
# actuators
# subscribe message name: /fusion,
# devide it according to different device names.
@author: ustyui
"""

import rospy
from silva_beta.msg import Evans

import transformations as tform

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
        self._payload_w = [0,0,0,0,0]
        
        self._position = ''
        self._current = ''
        self._pub_msg = Evans()
        
        self.tmp = [0,0,0,0,0]
        
        
"------------------------------------------------------------------main func"

def mbed_cb(_sock, _sockb, _str, run_event, cls):
    
    # send hello
    rate = rospy.Rate(_RATE)

    while run_event.is_set() and not rospy.is_shutdown():

        _sock.sendto(_str, (ip[dev_name], 10019))
        cls._position, addr_rt =  _sock.recvfrom(1024)
        cls.tmp = cls._position.split('a') # fake wheel payload
        cls._payload_w = [int(cls.tmp[1]),int(cls.tmp[2]),0,0,0]
            
#            
        rate.sleep()

if __name__ == "__main__":
    
    "node, UDP init"
    motorsock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rtclient = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    cur_client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)    
    
    rospy.init_node('Act_'+dev_name, anonymous = True)
    rate = rospy.Rate(_RATE)
#---------------------------------------------------------------------------    
    "define this joint"
    
    joint = Joint(dev_name)
#---------------------------------------------------------------------------     
    "read Evans message and write into the joint"   
    
    sub = rospy.Subscriber('/silva/joint_local/fusion', Evans, callback, joint)
    
    pub = rospy.Publisher('/silva/reflex_local/feedback', Evans, queue_size =10)
    
    pub_msg = Evans()
    "thread"    
    if dev_name == 'wheel':
        run_event = threading.Event()
        run_event.set()
        move_t = threading.Thread(target = mbed_cb, args = \
        (rtclient,cur_client,  'h', run_event, joint))
        move_t.start()
    
#---------------------------------------------------------------------------     
    while not rospy.is_shutdown():
        
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
            tform.make_message(pub_msg, 2, dev_name, 2, joint._payload_w)
            pub.publish(pub_msg)

#---------------------------------------------------------------------------         
        rate.sleep()
    if dev_name == 'wheel':
        move_t.join()

__version = "1.1.0"