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

driveunits = 50

class motions():
    def __init__(self):
        self._payload = [0]*50
        self._pub_msg = Evans()
        
        self.pub = rospy.Publisher('/silva/slave_local/demo', Evans, queue_size=3)
        rospy.Subscriber('keys', String, keys_cb)
    def keys_cb(self, msg):
        vels = msg.data[0]
        if vels == 'w':
            self._payload[26] = -50
            self._payload[27] = -50
        elif vels == 'e':
            self._payload[26] = -50
            self._payload[27] = -50
            "open eyes"
            print "eee"
        elif vels == '1':
            "little fingers left"
            self._payload[19] = -50
            rospy.sleep(0.5)
            self._payload[19] = 0       
        elif vels == '2':
            "ring"
            self._payload[18] = -50
            rospy.sleep(0.5)
            self._payload[18] = 0   
        elif vels == '3':
            "middle"
            self._payload[17] = -50
            rospy.sleep(0.5)
            self._payload[17] = 0   
        elif vels == '4':
            "index"
            self._payload[16] = -50
            rospy.sleep(0.5)
            self._payload[16] = 0   
        elif vels == '5':
            "thumb"
            self._payload[15] = -50
            rospy.sleep(0.5)
            self._payload[15] = 0   
        elif vels == '6':
            "little fingers right"
            self._payload[20] = -50
            rospy.sleep(0.5)
            self._payload[20] = 0   
        elif vels == '7':
            "ring"
            self._payload[21] = -50
            rospy.sleep(0.5)
            self._payload[21] = 0   
        elif vels == '8':
            "middle"
            self._payload[22] = -50
            rospy.sleep(0.5)
            self._payload[22] = 0  
        elif vels == '9':
            "index"
            self._payload[23] = -50
            rospy.sleep(0.5)
            self._payload[23] = 0  
        elif vels == '0':
            "thumb"
            self._payload[24] = -50
            rospy.sleep(0.5)
            self._payload[24] = 0  
        elif vels == 'g':
            "grip"
        elif vels == 'r':
            "release"
            for idx in range(15,25):
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

    
    