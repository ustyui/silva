#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy, rospkg, yaml
_driveunits = 50 # 50 is ibuki driveunits number

"""
Transform
=====

Provides
    1.  seperate
    2.  merge
    3.  isplit(by char)
    4.  set zeros
"""

#==============================================================================
# SEPERATE COMMAND STRING
# 
# seperate the command string ["0xxxx0xxxx0xxxx0xxxx0xxxx"] to 
# an int list [xxxx,xxxx,xxxx,xxxx,xxxx]
#==============================================================================
def seperate(_command):
    list_command =[]
    _n_digits = 5
    _n_digits = len(_command)
    "if length is not correct, raise exception"
    if (_n_digits % 5 > 0):
        print('[WARN] The length of command is not correct')
    else:
        _checksum = _n_digits * 0.2
        _every = int(_checksum)
        
        for index in range(0,5):
            list_command.append(int(_command[index*_every:(index+1)*_every]))
    return list_command

#==============================================================================
# SEPERATE CURRETN STRING
# 
# seperate the command string ["0xxxx0xxxx0xxxx0xxxx0xxxx"] to 
# an int list [xxxx,xxxx,xxxx,xxxx,xxxx]
#==============================================================================
def seperateCurrent(_command):
    list_command =[]
    _n_digits = 5
    _n_digits = len(_command)
    "if length is not correct, raise exception"
    if (_n_digits % 5 > 0):
        print('[WARN] The length of command is not correct')
    else:
        _checksum = _n_digits * 0.2
        _every = int(_checksum)
        
        for index in range(0,5):
            _hex = int(_command[index*_every:(index+1)*_every],16)
            list_command.append(_hex)
    return list_command


#==============================================================================
# MERGE FUNCTION OF IBUKI
# 
# merge the int list [x,xx,xxx,xxxx,xxxx] into a complete string
# return sendable mbed command string
# the INPUT must be in global use
#==============================================================================
def merge(_global_joint_now, _index = 5):
    _message = ''
    _joint_send = []
    
    for them in range(len(_global_joint_now)):
        _joint_send.append(str(_global_joint_now[them]*10).zfill(_index))
    _message = _message.join(_joint_send)
    _message.replace(" ","")
    "check overflow"
    if (len(_message) % _index == 0):
        #print(_message)
        return _message
    else:
        raise OverflowError

#==============================================================================
# SEPERATE ANGLES
# 
# usually used in reading value of mbed sensors
# protocal: 'axxaxxxa' xx is sensor value (int)
# return list of two sensor data
# TODO: increase the usablility
#==============================================================================
def isplit(_rosmessage, _splitsign = 'a'):
    if type(_rosmessage.data) != int:
        
        _string = _rosmessage.data
        _list_angle = [0,0,0]
        if _string.startswith(_splitsign):
            _list_angle = _string.split(_splitsign)
        #why I chose from 1? because 0 is an empty value
        _list_angle[1] = int(_list_angle[1])
        _list_angle[2] = int(_list_angle[2])
        
        print ([_list_angle[1], _list_angle[2]])
        return [_list_angle[1], _list_angle[2]]
        
#==============================================================================
# SET ZEROS
# 
# set zero matrix
#==============================================================================
def set_zeros(_varname, nums = _driveunits):
    # 50 is ibuki driveunits number
    
    for _idx in range (0, nums):
        _varname.append(0)

#==============================================================================
# SET ONES
# 
# set one matrix
#==============================================================================
def set_ones(nums = _driveunits):
    _varname = []
    for _idx in range (0, nums):
        _varname.append(1)
    
    return _varname


#==============================================================================
# MAKE EVANS MESSAGE
# 
# make evans message
#==============================================================================        
def make_message(msg, seq, name, msgid, payload):
    msg.header.stamp = rospy.Time.now()
    msg.seq = seq
    msg.name = name
    msg.msgid = msgid
    msg.payload = payload  
    
#==============================================================================
# LOAD MAP
# read from .map file
    # --------------------------------------------------- #
    ## only read values for operation
    ## make default message
    ## only run once
    # --------------------------------------------------- #
#==============================================================================        

def load_map(_which = 'ibuki'):
    
    params_valuea = []
    params_valueb = []
    
    # open the .map
    rospack = rospkg.RosPack()
    mappath = rospack.get_path('silva_beta')+'/src/defaults/'+_which+'.map'

    f = open(mappath)
    lines = f.readlines()
    f.close()
    
    # get serial, name and value
    for index in range(4, len(lines)):
        
        # delete \n 
        string_lines = lines[index][0:-1]
        
        # delete tabs
        params_list = string_lines.split('\t')
        # print params_list
        
        # get lists
        params_valuea.append(int(params_list[2]))
        try:
            params_valueb.append(int(params_list[3]))
        except IndexError:
            pass
            
    return params_valuea, params_valueb
    
#==============================================================================
# LOAD PARAMS
# read from .yaml file

#==============================================================================   
def read_param(_name = 'ibuki'):
    rospack = rospkg.RosPack()
    param_path = rospack.get_path('silva_beta')+'/params/'+_name+'.yaml'
    f = open(param_path, "r+")
    param_config = yaml.load(f)
    
    return param_config

        
_version = "2019"
