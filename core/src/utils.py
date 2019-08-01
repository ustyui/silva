#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Wed Jul 31 12:32:56 2019
Utils
@author: ustyui
"""
import rospy, rospkg, yaml
import numpy as np

#==============================================================================
# LOAD PARAMS
# read from .yaml file in /params folder
# _name: filename, _pkg: from which package 
#==============================================================================   
def read_param(_name = 'ibuki', _pkg = 'silva'):
    rospack = rospkg.RosPack()
    param_path = rospack.get_path(_pkg)+'/params/'+_name+'.yaml'
    try:
        f = open(param_path, "r+")
        return yaml.load(f)
    except IOError:
        return 0

#==============================================================================
# LOAD MAP
# read from .map file in /params folder
# _name: filename, _pkg: from which package 
    # --------------------------------------------------- #
    ## only read values for operation
    ## make default message
    ## params_valuea: column 2 valuesb: column 3
    # --------------------------------------------------- #
#==============================================================================        
def load_map(_name = 'ibuki', _pkg = 'silva'):
    params_valuea = []
    params_valueb = []
    
    # open the .map
    rospack = rospkg.RosPack()
    mappath = rospack.get_path(_pkg)+'/params/'+_name+'.map'

    f = open(mappath)
    lines = f.readlines()
    f.close()
    
    # get serial, name and value
    for index in range(4, len(lines)):        
        # delete \n 
        string_lines = lines[index][0:-1]        
        # delete tabs
        params_list = string_lines.split('\t')
        # get lists
        params_valuea.append(int(params_list[2]))
        try:
            params_valueb.append(int(params_list[3]))
        except IndexError:
            pass
            
    return params_valuea, params_valueb    

#==============================================================================
# MAKE EVANS MESSAGE
# 
# make evans message
#==============================================================================        
def make_message(msg, name, seq, msgid, payload):
    msg.header.stamp = rospy.Time.now()
    msg.seq = seq
    msg.name = name
    msg.msgid = msgid
    msg.payload = payload  
    
    return msg

#==============================================================================
# MERGE FUNCTION OF IBUKI
# 
# merge the int list [x,xx,xxx,xxxx,xxxx] into a complete string
# return sendable mbed command string
# _global_joint_now : int list   _index: to how many digits for each scalar
#==============================================================================
def merge(_global_joint_now, _index = 3):
    _message = ''
    _joint_send = []
    
    for them in range(len(_global_joint_now)):
        _joint_send.append(str(_global_joint_now[them]).zfill(_index))
    _message = _message.join(_joint_send)
    _message.replace(" ","")
    "check overflow"
    if (len(_message) % _index == 0):
        return _message
    else:
        # raise OverflowError
        return 0
