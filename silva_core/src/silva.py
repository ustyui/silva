#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Wed Apr 24 09:59:56 2019
debug graphical user interface PyQt5

@author: ustyui
"""
import sys, time
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
import sip, os

import rospy, rospkg, threading, webbrowser
from silva_core.msg import Evans
from std_msgs.msg import String, Float32MultiArray

import numpy as np
import utils, topics
from sensor_msgs.msg import Joy

param_config = utils.read_param()

driveunits = param_config['Config']['driveunits']
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
        
        mappath = rospack.get_path('silva_core')+'/params/'+_which+'/'+_which+'.map'
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
        utils.set_zeros(self._default, driveunits)
        utils.set_zeros(self._payload, driveunits)
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
        msg.level = 3
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
        self.picpath = rospack.get_path('silva_core')+'/src/assets/logo.png'
        
        # object init
        self.sld = []        
        self.sldh = [] #high: highest limit
        self.sldl = [] #low: lowest limit
        self.sldn = [] #now: current operation value
        self.sldr = [] #real: feedback value label from ibuki
        self.labels = []
        # init labels
        self.fb_value = [] # feedback value buffer

        utils.set_zeros(self.sld, driveunits)
        utils.set_zeros(self.sldh, driveunits)
        utils.set_zeros(self.sldl, driveunits)
        utils.set_zeros(self.sldn, driveunits)
        utils.set_zeros(self.sldr, driveunits)
        utils.set_zeros(self.labels, driveunits)
        utils.set_zeros(self.fb_value, driveunits)

        
        self.progress = [0,0,0,0] 
        
        # expressions
        self.expression = []
        # obtain expressions
        param_config = utils.read_param(dev_name, 'momentary_motion')
        self.expression = param_config['Expression']
        
        self.index =0
        
        # speech
        self._contents = ''
        
        # inner publishers
        self.pub_s = rospy.Publisher('/silva/speech_global/jp', String, queue_size=10)
        
        # get minimum, maximum, and default
        self.default, blank = utils.load_map(dev_name, 'ibuki')
        min_v, max_v = utils.load_map(dev_name, 'limit')
        self.min_rel = []
        self.max_rel = []
        
        # get rel minimum and maximum
        for i in range (len(min_v)):
            if (min_v[i]!=-1 or max_v[i]!=-1):
                self.min_rel.append(self.default[i] - min_v[i])
                self.max_rel.append(max_v[i]- self.default[i])
            else:
                self.min_rel.append(100)
                self.max_rel.append(100)
                
        # GUI
        self.title = 'silva'
        self.left = 300
        self.top = 50
        self.width = 1366
        self.height = 768
        
        # param        
        self.state = [0.0, 0.0, 1.0, 0.0]
        
        # yaml params
        self.param_config = utils.read_param(dev_name, 'gui_config')
        
        self.table_widget = MyTableWidget(self)
        
        self.table_widget.setGeometry(10,295,300,450)
        #self.setCentralWidget(self.table_widget)        
        
        
        self.table_widget_b = ButtomWidget(self)
        
        self.table_widget_b.setGeometry(300,460,850,265)
        
        self.initUI()
        
        time.sleep(2) # load delay for stable
        # subscribers
        sub_states = rospy.Subscriber('/silva/states', Float32MultiArray, self.states_cb)
        sub_fbs = rospy.Subscriber('/silva/reflex_local/ch0', Evans, self.fb_cb)

        
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
        
        #### Button Actions of Table Widgets ###
        self.table_widget.pushButton[0].clicked.connect(self.on_reset)
        self.table_widget.pushButton[1].clicked.connect(self.on_lookaround)
        self.table_widget.pushButton[2].clicked.connect(self.on_wavehand)
        self.table_widget.pushButton[3].clicked.connect(self.on_happiness)
        self.table_widget.pushButton[4].clicked.connect(self.on_sadness)
        self.table_widget.pushButton[5].clicked.connect(self.on_surprise)  
        self.table_widget.pushButton[6].clicked.connect(self.on_fear)  
        self.table_widget.pushButton[7].clicked.connect(self.on_anger)  
        self.table_widget.pushButton[8].clicked.connect(self.on_disgust) 
        self.table_widget.pushButton[9].clicked.connect(self.on_contempt)  
        
        
        # Create progressbar of IRSA
        for index in range(0,4):
            self.progress[index] = QProgressBar(self)
            self.progress[index].setGeometry(100,180+index*30,180,20)
            self.progress[index].setMaximum(100)
        
        # Create Sliders
        ## neck arml
        ### Note: The qt slider only generate from 0 to 99, so an external .conf file of the upper lower limit of joints is needed to judge the real output of the sliders
        for oidx in range(0,5):
            for index in range(0,10):
                self.sld[index+oidx*10] = QSlider(Qt.Horizontal, self)
                self.sld[index+oidx*10].setFocusPolicy(Qt.NoFocus)
                self.sld[index+oidx*10].setGeometry(390+oidx*200 ,60+index*40,90,30)
                self.sld[index+oidx*10].setTickPosition(QSlider.TicksBothSides)
                # set range of sliders
                self.sld[index+oidx*10].setRange(-self.min_rel[index+oidx*10],self.max_rel[index+oidx*10])
                self.sld[index+oidx*10].setValue(0)

                ### labels
                # operate value
                self.sldn[index+oidx*10] = QLabel(str(0),self)
                self.sldn[index+oidx*10].move(480+oidx*200,50+index*40)
                
                self.sldr[index+oidx*10] = QLabel(str(0),self)
                self.sldr[index+oidx*10].move(480+oidx*200,70+index*40)
                
        # slider value changed to corresponding lbel sldn
        for idx in range(0, 50):
            self.sld[idx].valueChanged.connect(self.sldn[idx].setNum)
        
        ## Menu buttons
        helpbutton1 = QAction('About on Github',self)
        helpbutton1.setShortcut('Ctrl+F1')
        helpbutton1.setStatusTip('Find documentation and usage on github.')
        helpbutton1.triggered.connect(lambda: self.link_to('https://github.com/ustyui/silva'))
        helpMenu.addAction(helpbutton1) 
        
    # ROS States callback
    def states_cb(self, msg):
        self.state = msg.data # map the value to states
        #print self.state
#        for index in range (0, len(self.state)):
#            self.progress[index].setValue(int(self.state[index]*100))
        
    # ROS Feedback Functions
    
    def fb_cb(self, msg):
        temp_buffer = msg.payload
        sn = seq_of_jointname[msg.name]
        # 45 fb's in the fb_value list
        for idx in range(0,5):
            self.fb_value[sn*5+idx] = temp_buffer[idx]
            
    @pyqtSlot()
    def on_click(self):
        self._contents = self.textbox.text()
        print(self._contents)
        self.textbox.clear()
        self.pub_s.publish(self._contents)
    def on_reset(self):
        print('Motion reset.')
        for i in range(0, 50):
            self.sld[i].setValue(0)
    def on_lookaround(self):
        print('Look around.')
        os.system('rosrun silva_core HSM_csv.py lookaround')
    def on_wavehand(self):
        os.system('rosrun silva_core HSM_csv.py wavehand')
        
    def on_happiness(self):
        print('Happiness.')
        for i in range(0,50):
            self.sld[i].setValue(self.expression['happiness'][i])
            
    def on_sadness(self):
        print('Sadness.')
        for i in range(0,50):
            self.sld[i].setValue(self.expression['sadness'][i])
            
    def on_surprise(self):
        print('Surprise')
        for i in range(0,50):
            self.sld[i].setValue(self.expression['surprise'][i])
    
    def on_fear(self):
        print('Fear.')
        for i in range(0,50):
            self.sld[i].setValue(self.expression['fear'][i])
    
    def on_anger(self):
        print('Anger.')
        for i in range(0,50):
            self.sld[i].setValue(self.expression['anger'][i])
    
    def on_disgust(self):
        print('Disgust.')
        for i in range(0,50):
            self.sld[i].setValue(self.expression['disgust'][i])
    
    def on_contempt(self):
        print('Contempt.')
        for i in range(0,50):
            self.sld[i].setValue(self.expression['contempt'][i])
        
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
        utils.set_zeros(self.pushButton, 10) # max 10 buttons now
        
        # Create Motion tab
        self.tab1.layout = QVBoxLayout(self)
        # Buttons
        self.pushButton[0] = QPushButton("Reset")
        self.pushButton[1] = QPushButton("Look around")
        self.pushButton[2] = QPushButton("Wave Hand")  
        self.pushButton[3] = QPushButton("Hapiness")
        self.pushButton[4] = QPushButton("Sadness")
        self.pushButton[5] = QPushButton("Surprise")
        self.pushButton[6] = QPushButton("Fear")
        self.pushButton[7] = QPushButton("Anger")
        self.pushButton[8] = QPushButton("Disgust")
        self.pushButton[9] = QPushButton("Contempt")
        
        # StatusTip
        self.pushButton[0].setStatusTip('Reset Operation Inputs of the User to 0.')
        self.pushButton[1].setStatusTip('Ibuki looks around motion preset.')
        self.pushButton[2].setStatusTip('Ibuki waves his left hand preset.')
        self.pushButton[3].setStatusTip('Ibuki show hapiness.')
        self.pushButton[4].setStatusTip('Ibuki show sadness.')
        self.pushButton[5].setStatusTip('Ibuki show surprise.')
        self.pushButton[6].setStatusTip('Ibuki show fear.')
        self.pushButton[7].setStatusTip('Ibuki show anger.')
        self.pushButton[8].setStatusTip('Ibuki show disgust.')
        self.pushButton[9].setStatusTip('Ibuki show contempt.')
        
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
        utils.set_zeros(self.pushButton, 10) # max 10 buttons now
        
        # init labels
        self.current_labels = []
        self.current_values = []
        self.cur_value = [] # current value feedback
        
        utils.set_zeros(self.cur_value, driveunits)
        
        utils.set_zeros(self.current_labels, driveunits)
        utils.set_zeros(self.current_values, driveunits)
        
        
        # Create Motion tab
        self.tab1.layout = QGridLayout(self)
        
        self.initUI()
 
        self.tab1.setLayout(self.tab1.layout)
        
        # Add tabs to widget
        self.layout.addWidget(self.tabs)
        self.setLayout(self.layout)    
        
        # rospy subscriber
        sub_cur = rospy.Subscriber('/silva/reflex_local/ch1', Evans, self.cur_cb)
        
    def initUI(self):
        for oidx in range(0,10):
            for index in range(0,5):
                self.current_labels[oidx*5+index] = QLabel('C['+str(oidx*5+index)+']:')
                self.current_values[oidx*5+index] = QLabel(str(0)+' mA')
                
                self.tab1.layout.addWidget(self.current_labels[oidx*5+index],oidx,index*2)
                self.tab1.layout.addWidget(self.current_values[oidx*5+index],oidx,index*2+1)
                
                #self.current_labels[oidx*5+index].move(400+oidx*90, 500+index*20)



#        self.table = QTableWidget()
#        self.tableItem = QTableWidgetItem()
#        
#        # init table
#        self.table.resize(800,250)
#        self.table.setRowCount(5)
#        self.table.setColumnCount(10)
        
        # set value 
#        for oidx in range(0,10):
#            for index in range(0,5):
#                table.setItem(index, oidx, QTableWidgetItem("0"))
        
        #self.tab1.layout.addWidget(self.table)
        
                # slider value changed to corresponding lbel sldn
            
    def cur_cb(self, msg):
        temp_buffer = msg.payload
        sn = seq_of_jointname[msg.name]
        
        for idx in range(0,5):
            self.cur_value[sn*5+idx] = (temp_buffer[idx])
            
            
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
    
    main_counter = 0
    
    while not rospy.is_shutdown(): ## note: this is GUI loop
        
        main_counter+=1
        # update the elements
        for idx in range (0, len(main_window.state)):
            
            main_window.progress[idx].setValue(int(main_window.state[idx]*100))
            #main_window.sld[2].setValue(int(main_window.sldn[1].text()))
        
        for idx in range (0, 45):
            fb = int(main_window.fb_value[idx]*0.1)-main_window.default[idx]
            main_window.sldr[idx].setText(str(fb)) # set feedback
            
            cr = int(main_window.table_widget_b.cur_value[idx])
            main_window.table_widget_b.current_values[idx].setText(str(cr)+' mA')
            
        #main_window.table_widget_b.table.setItem(1,1,QTableWidgetItem('1'))
        #main_window.table_widget_b.tab1.
        app.processEvents()
        
        # update the payloads ever two times
        if main_counter ==2:
            for index in range (0,50):
                Dpose._payload[index] = int(main_window.sldn[index].text())
            
            Dpose._pub_msg.payload = Dpose._payload
            main_counter = 0
            
        print main_window.fb_value
        
        loop_rate.sleep()
    
    sys.exit(app.exec_())

    
    
    
