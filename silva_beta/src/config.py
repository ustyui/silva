#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Defaults of ibuki
=====

Provides
    1.  ip
    2.  port
"""
#==============================================================================
# IBUKI'S CONTROLLER IP LIST
# 
# The definition of ibuki Raspis, mbeds ip
# there is no need to change anything here if no additional device is added
#==============================================================================
ip_dict = {'master':"192.168.99.123",
           'ibuki1':"192.168.99.101",
           'ibuki2':"192.168.99.102",
           'ibuki3':"192.168.99.103",
           'lrf':"192.168.99.104",
           'handr':"192.168.99.2",
           'handl':"192.168.99.3",
           'armr':"192.168.99.12",
           'arml':"192.168.99.13",
           'neck':"192.168.99.22",
           'headl':"192.168.99.31",
           'headc':"192.168.99.32",
           'headr':"192.168.99.33",
           'hip':"192.168.99.42",
           'wheel':"192.168.99.52",
           'codex':"192.168.99.99",
           'test':"119.63.197.151" #baidu
}
#==============================================================================
# IBUKI'S CONTROLLER PORT LIST
# 
# The definition of ibuki Raspis, mbeds por
# there is no need to change anything here if no additional device is added
#==============================================================================
port_dict = {'handl':10006,
             'handr':10007,
             'tts':10008,
             'tts2':10009,
             'tts3':10010,
             'headl':10011,
             'headc':10012,
             'headr':10013,
             'arml':10014,
             'armr':10015,
             'neck':10016,
             'hip':10017,
             'wheel':10018,
             'wheel2':10019,
             'executor':10099,
             'test':56  #baidu
}


def ip(__devicename):
    return ip_dict[__devicename]

def port(__portname):
    return port_dict[__portname]
    

_version = "2018"
