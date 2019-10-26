#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
demo program for interview
# including functions:
# 1. close eyes
# 2. open eyes
# 3. finger motion
# 4. grip
# 5. release grip
"""

import rospy
from silva_beta.msg import Evans
import transformations as tform
from std_msgs.msg import String

driveunits = 50

class motions():
    def __init__(self):
        self._payload = [0]*50
        self._pub_msg = Evans()
        
        self.pub = rospy.Publisher('/silva/slave_local/demo', Evans, queue_size=3)
        rospy.Subscriber('keys', String, self.keys_cb)
    def keys_cb(self, msg):
        vels = msg.data[0]
        if vels == 'w':
            self._payload[26] = -20
            self._payload[27] = -20
        elif vels == 'e':
            self._payload[26] = 130
            self._payload[27] = 80
            "open eyes"
            print "eee"
        elif vels == '1':
            "little fingers left"
            self._payload[19] = -100
            rospy.sleep(0.5)
            self._payload[19] = 100     
        elif vels == '2':
            "ring"
            self._payload[18] = -100
            rospy.sleep(0.5)
            self._payload[18] = 100
        elif vels == '3':
            "middle"
            self._payload[17] = -100
            rospy.sleep(0.5)
            self._payload[17] = 100  
        elif vels == '4':
            "index"
            self._payload[16] = -100
            rospy.sleep(0.5)
            self._payload[16] = 100 
        elif vels == '5':
            "thumb"
            self._payload[15] = -100
            rospy.sleep(0.5)
            self._payload[15] = 100   
        elif vels == '6':
            "little fingers right"
            self._payload[20] = -100
            rospy.sleep(0.5)
            self._payload[20] = 100
        elif vels == '7':
            "ring"
            self._payload[21] = -100
            rospy.sleep(0.5)
            self._payload[21] = 100
        elif vels == '8':
            "middle"
            self._payload[22] = -100
            rospy.sleep(0.5)
            self._payload[22] = 100
        elif vels == '9':
            "index"
            self._payload[23] = -100
            rospy.sleep(0.5)
            self._payload[23] = 100
        elif vels == '0':
            "thumb"
            self._payload[24] = -100
            rospy.sleep(0.5)
            self._payload[24] = 100
        elif vels == 'g':
            "grip"
            for idx in range(15,25):
                self._payload[idx] = -100
        elif vels == 'r':
            "release"
            for idx in range(15,25):
                self._payload[idx] = 0
        elif vels == 'b':
            for idx in range(0,50):
                self._payload[idx] = 0
        
    def start(self):
        rospy.loginfo("DEMO")
        loop_rate = rospy.Rate(50)
        
        while not rospy.is_shutdown():
            "make message"
            tform.make_message(self._pub_msg, 0, 'demo', 0, self._payload)
            "publish message"
            self.pub.publish(self._pub_msg)
            
            loop_rate.sleep()

if __name__ == "__main__":
    demo = motions()
    nh = rospy.init_node("DEMO_NODE")
    demo.start()

    
    