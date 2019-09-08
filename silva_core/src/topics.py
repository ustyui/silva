#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Thu Aug  1 15:06:54 2019
topic management
@author: ustyui
"""

com = \
{'mixer':   '/silva/joint_local/mixer',
 'default': '/silva/joint_local/default',
 'states':  '/silva/states',
 'joy':     '/joy',
 'idle':    '/silva/joint_local/idle',
 'reflex':  '/silva/joint_local/reflex',
 'slave':   '/silva/joint_local/slave',
 'auto' :   '/silva/joint_local/auto'
        }

feedback = \
{'wheelencoder':    '/silva/reflex_local/feedback'
 }

subchn = \
{'s0':  '/silva/slave_local/intention',
 's1':  '/silva/slave_local/operation',
 's2':  '/silva/slave_local/decision',
 's3':  '/silva/slave_local/walking',
 's4':  '/silva/slave_local/hsm',
 'i0':  '/silva/idle_local/ch0',
 'i1':  '/silva/idle_local/intention',
 'r0':  '/silva/reflex_local/ch0',
 'r1':  '/silva/reflex_local/ch1',
 'r2':  '/silva/reflex_local/ch2',
 'r3':  '/silva/reflex_local/feedback',
 'a0':  '/silva/auto_local/ch0',
 'a1':  '/silva/auto_local/ch1',
 'a2':  '/silva/auto_local/ch2',
 'a3':  '/silva/auto_local/ch3'
 }

# comment: maybe it is okay to add a temp topics here, for specific uses;


        

