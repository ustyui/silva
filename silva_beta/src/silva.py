#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Wed Apr 24 09:59:56 2019
debug graphical user interface PyQt5

@author: ustyui
"""
import sys
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
import sip

import rospy, rospkg, threading, webbrowser
from silva_beta.msg import Evans
from std_msgs.msg import String, Float32MultiArray

import numpy as np
import transformations as tform
from sensor_msgs.msg import Joy

param_config = tform.read_param()

seq_of_jointname = param_config['SequenceOfJoints']
_RATE = param_config['Rates']['gui']
dev_name = param_config['Config']['robotname']

class QtPose(object):
    def __init__(self, dev_name):
        # robot name # 
        self._name = dev_name
                
        self._params_length = 0         # the number of DOFs
        
        self._covs = [0, 0, 1, 0, 0]    # weight matrix, 1 is slave ctrl
        
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

    def load_default(self, _which = dev_name):
        
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
        
### publish operator ###
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
        
###

class MainWindow(QMainWindow):
    def __init__(self, parent=None):
        super(MainWindow, self).__init__(parent)
        # decoration
        rospack = rospkg.RosPack()
        self.picpath = rospack.get_path('silva_beta')+'/src/assets/logo.png'
        
        # speech
        self._contents = ''
        
        # inner publishers
        self.pub_s = rospy.Publisher('/silva/speech_global/jp', String, queue_size=10)
        
        
        # GUI
        self.title = 'silva'
        self.left = 300
        self.top = 50
        self.width = 1024
        self.height = 768
        
        self.initUI()
        
    def initUI(self):
        self.setWindowTitle(self.title)
        self.setGeometry(self.left, self.top, self.width, self.height) #1024*768
        
        # Create Statusbar
        
        self.statusBar().showMessage('Ready')
        
        # Create Menu
        mainMenu = self.menuBar()
        viewMenu = mainMenu.addMenu('View')
        toolsMenu = mainMenu.addMenu('Tools')
        helpMenu = mainMenu.addMenu('Help')
        
        # Create Labels
        
        label0 = QLabel(self)
        pixmap = QPixmap(self.picpath)
        label0.resize(191,196)
        label0.setPixmap(pixmap)
        label0.move(self.width-201,self.height-246)
        
        label1 = QLabel('Teleoperation Speech', self)
        label1.move(20,30)
        
        label2 = QLabel('Main Gate', self)
        label2.move(20,140)
        
        label3 = QLabel('ERATO Symbiotic HRI project. All rights reserved.',self)
        label3.resize(491,20)
        label3.move(self.width-350,self.height-40)
        
        # Create textbox
        self.textbox = QLineEdit(self)
        self.textbox.move(20, 60)
        self.textbox.resize(280,40)

        # Create a button in the window
        button = QPushButton('Send Speech', self)
        button.setStatusTip('Send the textbox content to ibuki terminal.')
        button.move(110,110)
        button.clicked.connect(self.on_click)
        
        # Create Button groups
        
        
        ## Menu buttons
        helpbutton1 = QAction('About on Github',self)
        helpbutton1.setShortcut('Ctrl+F1')
        helpbutton1.setStatusTip('Find documentation and usage on github.')
        helpbutton1.triggered.connect(lambda: self.link_to('https://github.com/ustyui/silva'))
        helpMenu.addAction(helpbutton1) 
        

    @pyqtSlot()
    def on_click(self):
        self._contents = self.textbox.text()
        print(self._contents)
        self.textbox.clear()
        self.pub_s.publish(self._contents)
    @pyqtSlot()
    def link_to(self, url):
        webbrowser.open(url)
        
        
        
        
if __name__ == '__main__':
    
    #ros function
    
    # ROS loginfo
    rospy.loginfo("Debug GUI")  
    
    # pose class
    Dpose = QtPose(dev_name)
    Dpose.start()    
    
    # signal flag for running threads
    run_event = threading.Event()
    run_event.set()

    # init node 
    nh = rospy.init_node("silva")
    loop_rate = rospy.Rate(_RATE)

    # publisher
    pub = rospy.Publisher('/silva/slave_local/operation', Evans, queue_size=10)
    
    # thread that publish the operation message
    move_t = threading.Thread(target = opt_pub, args = \
    (20, pub, Dpose._pub_msg, run_event))
    
    move_t.start()
    
    app = QApplication(sys.argv)
    main_window = MainWindow()
    main_window.show()
    sys.exit(app.exec_())
    
    