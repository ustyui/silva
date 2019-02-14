#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Mon Jan  7 09:59:56 2019
debug graphical user interface

@author: ustyui
"""

dev_name = 'ibuki'

from Tkinter import *
from struct import *

import rospy, rospkg
from silva_beta.msg import Evans
from std_msgs.msg import String, Float32MultiArray

import numpy as np
import threading

import transformations as tform
from sensor_msgs.msg import Joy

_RATE = 20 # ros rate

# TODO: change this to a file load function
seq_of_jointname = {'neck':0,
                    'arml':1,
                    'armr':2,
                    'handl':3,
                    'handr':4,
                    'headl':5,
                    'headc':6,
                    'headr':7,
                    'hip':8,
                    'wheel':9}

# Tk class used to store joint variables
class Tkpose(object):
    def __init__(self, dev_name):
        # robot name # 
        self._name = dev_name
                
        self._params_length = 0         # the number of DOFs
        
        self._covs = [0, 0, 1, 0, 0]    # cov matrix, 1 is slave ctrl
        
        self._params_serial = []        # serial number of each DoF
        self._params_name = []          # name of each DoF
        self._params_value = []         # value of each DoF
        
        self._default = []              # default value
        self._payload = []              # payload value
        
        self._dict_name_value = {}      # default name-value dict
        self._dict_serial_name = {}     # default serial-name dict
        self._dict_serial_value = {}    # defaule serial-value dict
        
        self._default_msg = Evans()     # default msg used to pub
        self._pub_msg = Evans()         # memeory msg used to pub
        
            

    def load_default(self, _which = 'ibuki'):
        
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
            self._params_serial.append(int(params_list[0]))
            self._params_name.append(params_list[1])
            self._params_value.append(int(params_list[2]))
            
            # get param length
            self._params_length = len(self._params_value)
            
#            # get dicts
#            self._dict_name_value[params_list[1]] = params_list[2]
#            self._dict_serial_name[params_list[0]] = params_list[1]
#            self._dict_serial_value[params_list[0]] = params_list[2] 
            
            
        # zip the lists
        self._dict_name_value = dict(zip(self._params_name, self._params_value))
        self._dict_serial_name = dict(zip(self._params_serial, self._params_name))
        self._dict_serial_value = dict(zip(self._params_serial, self._params_value))
        
        
        # initialize attributes
        tform.set_zeros(self._default)
        tform.set_zeros(self._payload)
        self._pub_msg.payload = self._default
    
    def start(self):
        
        self.load_default()

def opt_pub(rate, pub, msg, run_event):
    
    # frequency
    rate = rospy.Rate(_RATE)
    # publish the fusion
    while run_event.is_set() and not rospy.is_shutdown():
        # make the message
        msg.header.stamp = rospy.Time.now()
        msg.seq = 3
        msg.name = 'slave'
        msg.msgid = 1
        # payload set by main function        
        
        pub.publish(msg)
        
        rate.sleep()    

class GUI(object):
    def __init__(self):
        ### Initialization Varialbes ###
        
        self.window = []
        self.canvas = [[],[],[],[]]
        self.msg = []
        self.state = [0.0, 0.0, 0.0, 0.0]
        self.statetag = ['I', 'R', 'S', 'A']
        
        # speech
        self._contents = ''
        
        tform.set_zeros(self.window)
        tform.set_zeros(self.msg)
        
        #inner publishers
        pub_s = rospy.Publisher('/silva/speech_global/jp', String, queue_size=10)
        
        # inner subscribers
        sub_cov = rospy.Subscriber('/silva/states', Float32MultiArray, self.state_cb)
        
        # get minimum, maximum, and default
        default, blank = tform.load_map('ibuki')
        min_v, max_v = tform.load_map('limit')
        min_rel = []
        max_rel = []
        # get rel minimum and maximum
        for i in range (len(min_v)):
            if (min_v[i]!=-1 or max_v[i]!=-1):
                min_rel.append(default[i] - min_v[i])
                max_rel.append(max_v[i]- default[i])
            else:
                min_rel.append(100)
                max_rel.append(100)
        
        ### function definition ###
        
        def show_entry_fields():
            self._contents = self.textbox.get()
            print(self._contents)
            self.textbox.delete(0,END)
            pub_s.publish(self._contents)        
    
        ### GUI initialization ###
        self.master = Tk()
        self.master.title("Ibuki System Configuration ver 2.0")
        ## First frame ##
        # 50 slider bars
        for idx in range(0, 5):
            # label in row 0,3,6,9,12
            self.label = Label(self.master, text = \
            list(seq_of_jointname.keys())[list(seq_of_jointname.values()).index(idx)])
            self.label.grid(row=idx*3, column = 0, columnspan = 5)            
            
            # slier in row 4,7,10,13,16, column in 0,1,2,3,4
            for i in range(idx*5, idx*5+5):
                self.window[i] = Scale(self.master, from_=-min_rel[i], to=max_rel[i], length = 100)
                self.window[i].set(0)
                self.window[i].grid(row=idx*3+1,column = i-5*idx, rowspan =2)
                
        ## Middle line##
        
        ## Second Frame ##
        for idx in range(0, 5):
            # label in row 0,3,6,9,12, column in 6
            self.label = Label(self.master, text = \
            list(seq_of_jointname.keys())[list(seq_of_jointname.values()).index(idx+5)])
            self.label.grid(row=idx*3, column = 6, columnspan = 4)
            
            # slier in row 4,7,10,13,16, column in 6,7,8,9,10
            for i in range((idx+5)*5, (idx+5)*5+5):
                self.window[i] = Scale(self.master, from_=-min_rel[i], to=max_rel[i], length = 100)
                self.window[i].set(0)
                self.window[i].grid(row=idx*3+1,column = i-5*idx-19, rowspan=2)
                
        ## Third frame ##
        self.line = Canvas(self.master, width = 5, height = 100, bg = "pink")
        self.line.grid(row = 5 ,column = 5, rowspan = 11)
        # dialogue texboxes
        # label in row0, column 11
        self.label = Label(self.master, text = 'speech interface')
        self.label.grid(row = 0, column = 11, columnspan = 4)
        
        self.textbox = Entry(self.master)
        self.textbox.insert(10,"対話内容")
        self.textbox.grid(row = 1, column = 11, columnspan = 4)
        
        self.button = Button(self.master, text = "send speech", command = show_entry_fields)
        self.button.grid(row = 2, column = 11, rowspan = 2, columnspan =4)
        
        # state canvas
        # label in row3, column 11
        self.label = Label(self.master, text = 'state')
        self.label.grid(row = 3, column =11, columnspan = 4)
        for idx in range(0, len(self.canvas)):
            self.canvas[idx] = Canvas(self.master, width = 20, height = 60, bg = "white")
            self.canvas[idx].grid(row = 4 ,column = idx+11)
        for idx in range(0, len(self.statetag)):
            self.label = Label(self.master, text = self.statetag[idx])
            self.label.grid(row = 5, column = idx+11)
        # update canvas
        for idx in range(0, len(self.canvas)):
            self.canvas[idx].create_rectangle(0,58,20,60, fill = 'red', tag= 'bar')
        
        # label in row16, column 11
        self.label = Label(self.master, text = 'state')
        self.label.grid(row = 15, column =11, columnspan = 4)        
                       
    def state_cb(self, msg):
        self.state = msg.data
        
    def update_state(self):
                
        # update canvas
        for idx in range(0, len(self.canvas)):
            self.canvas[idx].delete('all')
            self.canvas[idx].create_rectangle(0,59-self.state[idx]*58,20,60, fill = 'red')
            

if __name__ == "__main__":
    
    # ROS loginfo
    rospy.loginfo("Debug GUI")
    # GUI class
    ibk = GUI()
    
    # pose class
    Dpose = Tkpose(dev_name)
    Dpose.start()
    
    # signal flag for running threads
    run_event = threading.Event()
    run_event.set()
    
    # init node 
    nh = rospy.init_node("debug_GUI")
    loop_rate = rospy.Rate(_RATE)
    
    # publisher
    pub = rospy.Publisher('/silva/slave_local/operation', Evans, queue_size=10)
    
    # thread that publish the operation message
    move_t = threading.Thread(target = opt_pub, args = \
    (20, pub, Dpose._pub_msg, run_event))
    
    move_t.start()
    
    # loop
    while not rospy.is_shutdown():
        
        # do gui
        ibk.master.update_idletasks()
        ibk.master.update()

        ibk.update_state()
        
        # send message
        for index in range(0,50):
            Dpose._payload[index] = int(ibk.window[index].get())
            
        Dpose._pub_msg.payload = Dpose._payload
        
        
        
        
    
    