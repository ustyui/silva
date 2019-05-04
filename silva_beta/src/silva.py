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
        
        # object init
        self.sld = []        
        self.sldh = [] #high: highest limit
        self.sldl = [] #low: lowest limit
        self.sldn = [] #now: current value
        self.labels = []

        tform.set_zeros(self.sld)
        tform.set_zeros(self.sldh)
        tform.set_zeros(self.sldl)
        tform.set_zeros(self.sldn)
        tform.set_zeros(self.labels)

        self.progress = [0,0,0,0,0] 
        
        self.index =0
        
        # speech
        self._contents = ''
        
        # inner publishers
        self.pub_s = rospy.Publisher('/silva/speech_global/jp', String, queue_size=10)
        
        
        # GUI
        self.title = 'silva'
        self.left = 300
        self.top = 50
        self.width = 1366
        self.height = 768
        
        # param        
        self.state = [0.0, 0.0, 0.5, 0.0]
        
        # yaml params
        self.param_config = tform.read_param('gui_config')
        
        self.table_widget = MyTableWidget(self)
        
        self.table_widget.setGeometry(10,295,300,450)
        #self.setCentralWidget(self.table_widget)        
        
        
        self.table_widget_b = ButtomWidget(self)
        
        self.table_widget_b.setGeometry(300,460,850,265)
        
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
        label0.move(self.width-201,self.height-255)
        
        label1 = QLabel('Teleoperation Speech', self)
        label1.move(20,30)
        
        label2 = QLabel('Main Gate', self)
        label2.move(20,140)
        
        label3 = QLabel('ERATO Symbiotic HRI project. All rights reserved.',self)
        label3.resize(491,20)
        label3.move(self.width-350,self.height-40)
        
        label4 = QLabel('Idle', self)
        label4.move(33, 175)
        
        label5 = QLabel('Reflex', self)
        label5.move(33, 205)
        
        label6 = QLabel('Slave', self)
        label6.move(33, 235)

        label7 = QLabel('Auto', self)
        label7.move(33, 265)        
        
        label8 = QLabel('Real-time Motion Adjustment',self)
        label8.resize(300,20)
        label8.move(320,30)
        
        # Slider Labeling
        
        ## neck
        for i in range (0,5):
            self.labels[i] = QLabel(self.param_config['neck'][i], self)
            self.labels[i].move(320, 60+i*40)
        
        ## arml
        for i in range (0,5):
            self.labels[i] = QLabel(self.param_config['arml'][i], self)
            self.labels[i].move(320, 260+i*40)
        
        ## armr
        for i in range (0,5):
            self.labels[i] = QLabel(self.param_config['armr'][i], self)
            self.labels[i].move(520, 60+i*40)
            
        ## handl
        for i in range (0,5):
            self.labels[i] = QLabel(self.param_config['handl'][i], self)
            self.labels[i].move(520, 260+i*40)
            
        ## handr
        for i in range (0,5):
            self.labels[i] = QLabel(self.param_config['handr'][i], self)
            self.labels[i].move(720, 60+i*40)      
            
        ## headl
        for i in range (0,5):
            self.labels[i] = QLabel(self.param_config['headl'][i], self)
            self.labels[i].move(720, 260+i*40)
            
        ## heac
        for i in range (0,5):
            self.labels[i] = QLabel(self.param_config['headc'][i], self)
            self.labels[i].move(920, 60+i*40)          
        
        ## headr
        for i in range (0,5):
            self.labels[i] = QLabel(self.param_config['headr'][i], self)
            self.labels[i].move(920, 260+i*40)
        
        ## hip
        for i in range (0,5):
            self.labels[i] = QLabel(self.param_config['hip'][i], self)
            self.labels[i].move(1120, 60+i*40)     
        
        ## wheel
        for i in range (0,5):
            self.labels[i] = QLabel(self.param_config['wheel'][i], self)
            self.labels[i].move(1120, 260+i*40)
        
        # Create textbox
        self.textbox = QLineEdit(self)
        self.textbox.move(20, 60)
        self.textbox.resize(280,40)

        # Create a button in the window
        button = QPushButton('Send Speech', self)
        button.setStatusTip('Send the textbox content to ibuki terminal.')
        button.move(110,110)
        button.clicked.connect(self.on_click)
        
        # Create progressbar
        for index in range(0,4):
            self.progress[index] = QProgressBar(self)
            self.progress[index].setGeometry(100,180+index*30,180,20)
            self.progress[index].valueChanged[int].connect(self.changeValue)
        
        # Create Sliders
        ## neck arml
        ### Note: The qt slider only generate from 0 to 99, so an external .conf file of the upper lower limit of joints is needed to judge the real output of the sliders
        for oidx in range(0,5):
            for self.index in range(0,10):
                self.sld[self.index+oidx*10] = QSlider(Qt.Horizontal, self)
                self.sld[self.index+oidx*10].setFocusPolicy(Qt.NoFocus)
                self.sld[self.index+oidx*10].setGeometry(390+oidx*200 ,60+self.index*40,100,30)
                self.sld[self.index+oidx*10].setTickPosition(QSlider.TicksBothSides)
                self.sld[self.index+oidx*10].setValue(50)
                self.sld[self.index+oidx*10].valueChanged[int].connect(self.changeValue)
                ### labels
                self.sldn[self.index+oidx*10] = QLabel(str(0),self)
                self.sldn[self.index+oidx*10].move(490+oidx*200,60+self.index*40)
        
        ## armr, handl       
        ## handr, headl
        
        ## headc, headr
        
        ## hip, wheel
        
        
        ## Menu buttons
        helpbutton1 = QAction('About on Github',self)
        helpbutton1.setShortcut('Ctrl+F1')
        helpbutton1.setStatusTip('Find documentation and usage on github.')
        helpbutton1.triggered.connect(lambda: self.link_to('https://github.com/ustyui/silva'))
        helpMenu.addAction(helpbutton1) 
        
    def changeValue(self, value):
        realvalue = value - 50
        self.sldn[self.index].setText(str(realvalue))
        print value

    @pyqtSlot()
    def on_click(self):
        self._contents = self.textbox.text()
        print(self._contents)
        self.textbox.clear()
        self.pub_s.publish(self._contents)
    @pyqtSlot()
    def link_to(self, url):
        webbrowser.open(url)
#    
#    def download(self):
#        self.progress3.setValue(self.state[2])
        
class MyTableWidget(QWidget):
    def __init__(self, parent):
        super(QWidget, self).__init__(parent)
        self.layout = QGridLayout(self)
        
        # init tabs
        self.tabs = QTabWidget()
        self.tab1 = QWidget()
        self.tab2 = QWidget()
        self.tabs.resize(300,200)
        
        # add tabs
        self.tabs.addTab(self.tab1, "Motion")
        self.tabs.addTab(self.tab2, "State")
        
        # init buttons
        self.pushButton = []
        tform.set_zeros(self.pushButton, 10) # max 10 buttons now
        
        # Create Motion tab
        self.tab1.layout = QVBoxLayout(self)
        # Buttons
        self.pushButton[0] = QPushButton("Reset")
        self.pushButton[1] = QPushButton("Look around")
        self.pushButton[2] = QPushButton("Wave Hand")  
        
        for index in range(3,10):
            self.pushButton[index] = QPushButton("Motion"+str(index))
        
        for index in range(0,10):
            self.tab1.layout.addWidget(self.pushButton[index])
        self.tab1.setLayout(self.tab1.layout)
        
        # Add tabs to widget
        self.layout.addWidget(self.tabs)
        self.setLayout(self.layout)
        
class ButtomWidget(QWidget):
    def __init__(self, parent):
        super(QWidget, self).__init__(parent)
        self.layout = QGridLayout(self)
        
        # init tabs
        self.tabs = QTabWidget()
        self.tab1 = QWidget()
        self.tab2 = QWidget()
        self.tabs.resize(300,200)
        
        # add tabs
        self.tabs.addTab(self.tab1, "Feedbacks")
        self.tabs.addTab(self.tab2, "Network Status")
        
        # init buttons
        self.pushButton = []
        tform.set_zeros(self.pushButton, 10) # max 10 buttons now
        
        # Create Motion tab
        self.tab1.layout = QVBoxLayout(self)
 
        self.tab1.setLayout(self.tab1.layout)
        
        # Add tabs to widget
        self.layout.addWidget(self.tabs)
        self.setLayout(self.layout)    
        
        
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
    
    