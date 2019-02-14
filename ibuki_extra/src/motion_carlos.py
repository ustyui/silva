#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Thu Jan 24 13:31:49 2019
# CommuTTS -> wav generation -> carlos server -> motion data back
# play sound and transfer slave motion

@author: usytui on nvidia
"""
INTERVAL = 0.0092
# ros
import rospy, rospkg
from silva_beta.msg import Evans
from std_msgs.msg import String
import transformations as tform

# socket 
import socket, errno
import sys, os, time
import threading, struct

# tts
import numpy as np
import json, requests, getpass
# sound
import pyaudio, wave

# environment variables
_driveunits = 50
ip_localhost = "127.0.0.1"
ip_xavierserver = "192.168.100.111"
#port_carlossound = 11000
#port_carlosmotion = 22000
url_tts = "http://127.0.0.1:9031/aitalk/"
speak_speed = 0.85

rospack = rospkg.RosPack()
#TMPPATH = rospack.get_path('ibuki_extra')+'/src/tmp.txt'

# initialize a class for sending and receving message
class rosmessages():
    def __init__(self):
        
        # global variables
        self._rel = []
        self._payload = []
        self._default = []

        self._content = 'ss'
        self._data = {}
        tform.set_zeros(self._default)
        tform.set_zeros(self._rel, 5)
        tform.set_zeros(self._rel)
        tform.set_zeros(self._payload)

        
        self.sock = ''
        self.addr = ''
        self.sockmot = ''
        self.addrmot = ''
        self.moveaxis = ''
        
        self._data = [[]for row in range(_driveunits)]
        # set zeros
        

        print self._payload
        self._tmp = np.array(self._payload)
        
        # messages
        self._default_rec = Evans()
        self._pub_msg = Evans()
        
        # publishers
        self.pub = rospy.Publisher('/silva/slave_local/decision', Evans, queue_size=10)
        
        # subscribers
        self.sub_default = rospy.Subscriber('/silva/joint_local/default', Evans, self.default_cb)
        self.sub_speech = rospy.Subscriber('/silva/speech_global/jp', String, self.speech_cb)
        
        # socket initilization
        
#        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#        self.sock.bind((ip_xavierserver,port_carlossound))
#        self.motionsock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#        self.motionsock.bind((ip_xavierserver,port_carlosmotion))
        
#        print "default socket timeout: %s" %self.sock.gettimeout()
#        self.sock.settimeout(1000)
#        self.motionsock.settimeout(1000)
#
#        print 'waiting for connection...'
#        print "default socket timeout: %s" %self.sock.gettimeout()
        
    ### callback functions ###
    def default_cb(self, msg):
        self._default_rec = msg         # callback default
        self._default = msg.payload
        
    ### player ###
    "Play the sound"
    def ibuki_player(self,filename):
        
        chunk = 1024
        
        wav_file = wave.open(filename,"rb")      #open the wav
        
        #instantiate pyaudio
        player = pyaudio.PyAudio()
        #open stream
        stream = player.open(format = player.get_format_from_width(wav_file.getsampwidth()),
                             channels = wav_file.getnchannels(),
                             rate = wav_file.getframerate(),
                             output = True)
        #read data
        data = wav_file.readframes(chunk)
        
        #play stream
        while data:
            stream.write(data)
            data = wav_file.readframes(chunk)
        
        #stop stream
        stream.stop_stream()
        stream.close()
        
        #close PyAudio
        player.terminate()
        os.system("pulseaudio --kill")
        
        time.sleep(0.3)
    

    ### Functions ###    
    def getTTS(self):
        self._data = {'text':self._content,'speed':0.85}
        tts_log = requests.get(url_tts, params = self._data)
        ret = json.loads(tts_log.text)
        username = getpass.getuser() # get username
        wav_name = "/home/"+username+"/mAITalkHttpServer/jar/wav/"+ret['wav-file']
        # file_name = "/home/"+username+"/mAITalkHttpServer/jar/wav"+ret['wav-file']
        
        return wav_name
    
    def player_i(self, wavname, run_event):
        self.ibuki_player(wavname)
    
    # NOT be used now because of bugs
    def tcplink(self, socks, addrs, wavpath, run_event):
        print 'accept new connection from %s:%s...' % addrs
        #sock.send('welcome!')
        file_open = open(wavpath, 'rb')
        
        # seperate it into 324b packs, because carlos svtools protocol rules
        readedDataSize = 4
        header = 'RIFF'
        chunkSize = 320
        contentLength = os.path.getsize(wavpath)
        
        while readedDataSize < contentLength:
#            file_open.seek(readedDataSize)
            data = file_open.read(chunkSize)
            data_header = header + data
            print len(data_header)
            socks.send(data_header)
            
            # update readed data startpoint
            readedDataSize = readedDataSize + len(data)
        
        file_open.close()
        
    def load_presets(self, filename = 'hello'):
    # load preset motion files and perform joint_to_where(map version)
        filepath = rospack.get_path('ibuki_extra')+'/src/preset/'+ filename +'.txt'

        f = open(filepath)
        lines = f.readlines()
        f.close()
        data = {}
        data = [[]for row in range(_driveunits)]
        
        for index in range(4,len(lines)-1):
            string_lines = lines[index][0:-1]
            params_list = string_lines.split('\t')
            # map the value to the right place
            # 1. calculate
            headr2 = 0.549* int(params_list[8])
            headr3 = -0.549* int(params_list[8])
            headr0 = 0.431* int(params_list[10])
            headr1 = -0.353* int(params_list[10])
            headr4 = 0.137* int(params_list[12])
            if params_list[14] =='-1':
                params_list[14] = 128
            neck4 = -4.0* (int(params_list[14])-128)
            hip0 = 0.05* int(params_list[19])

            # 2. append
            data[37].append(int(headr2))
            data[38].append(int(headr3))
            data[35].append(int(headr0))
            data[36].append(int(headr1))
            data[39].append(int(headr4))
            data[4].append(int(neck4))
            data[40].append(int(hip0))         
        
        return data        
        
    def make_carlos_message(self, data, index):
        for i in range(0, len(data)):
            if i>40 or i<35:
                if i != 4:
                    self._tmp[i] = 0
                elif i == 4:
                    self._payload[i] = data[i][index]
            else:
                self._payload[i] = data[i][index]
                
        
    
    ### Callback Functions ###    
    def speech_cb(self, msg):
        if msg.data == 'hello':
            self._content = 'こんにちは'
            filename = 'hello'
        elif msg.data == 'agree':
            self._content = 'そうですね。'
            filename = 'agree'
        elif msg.data == 'goodbye':
            self._content = 'さようならー'
            filename = 'goodbye'
        elif msg.data == 'selfintro1':
            self._content = 'はじめまして。ー。ー。なまえは、いぶきです。いきる、っていういみがあります。ー。としわ、じっさい。'
            filename = 'selfintro1'
        elif msg.data == 'selfintro2':
            self._content = 'すきなことは、いろんなばしょに、いくこと。ー。ー。ー。やまや、うみ、たくさんのどうぶつや、むしたち、。ー。いろんなものをみて、。ー。さわって、。ー。かんじる。ー。いろんなばしょで、いっしょに、あそんでくれるともだちがほしいです。'
            filename = 'selfintro2'
        elif msg.data == 'selfintro3':
            self._content = 'どうぞよろしくおねがいします。'
            filename = 'selfintro3'
        elif msg.data == 'thankyou':
            self._content = 'ありがとうございます'
            filename = 'thankyou'
            
        # because we only want contents be carried out once, so do commu tts here
        
        WAVNAME = self.getTTS() # get wave name 
        self._data = self.load_presets(filename)
           
        print(self._data[4])
        run_event = threading.Event()
        run_event.set()
        
#        send_w = threading.Thread(target = self.tcplink, args = (self.sock, self.addr, WAVNAME, run_eventw))
        move_t = threading.Thread(target = self.player_i, args = (WAVNAME, run_event))
        
#        send_w.start()
        move_t.start()
        
        time.sleep(0.19) # threshold
        
        # make the message, and send the message to slave operation
        for index in range(0, len(self._data[4])):
            self.make_carlos_message(self._data, index)
            tform.make_message(self._pub_msg, 3, 'slave', 3, self._payload)
            print(self._payload)
            self.pub.publish(self._pub_msg)
            time.sleep(INTERVAL)        
        
    

if __name__ == "__main__":
    
    # pose class
    mds = rosmessages()
#    mds.sock.listen(5)
#    mds.motionsock.listen(5)
    # init 
    nh = rospy.init_node("Calros_interpret")
    rate = rospy.Rate(0.5)
    
    run_events = threading.Event()
    run_events.set()
    
    while not rospy.is_shutdown():
        #print(mds._content)
#        mds.sock, mds.addr = mds.sock.accept()
#        mds.sockmot, mds.addrmot = mds.motionsock.accept()
        
#        recv_s = threading.Thread(target = motion_peer, args = (mds.sockmot, mds.addrmot, run_events))
#        recv_s.start()
#            
#        test = mds.moveaxis
        rate.sleep()
