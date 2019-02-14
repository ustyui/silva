#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Sun Jan 27 23:11:42 2019
joystick
@author: nvidia
"""
import rospy
from silva_beta.msg import Evans
from sensor_msgs.msg import Joy
import transformations as tform

_RATE = 40

#class joystic
class Joystick():
    # initial ros message
    def __init__(self):
        self._forward = 0.0
        self._steering = 0.0
        self._speed_right = 0.0
        self._speed_left = 0.0
        self._break = 1
        self._payload = []
        tform.set_zeros(self._payload,5)

        # message
        self._pub_msg = Evans()
        # publishers
        self.pub = rospy.Publisher('/silva/slave_local/walking', Evans, queue_size=10)
        
        self.sub = rospy.Subscriber('/joy', Joy, self.joy_cb)
        
    ### callback functions ###
    def joy_cb(self, msg):
        if msg.header.frame_id != 'main': # my bad
            self._forward = msg.axes[1]
            self._steering = msg.axes[0]
            
            # use this two values to decide left and right
            # use cases:
                # [1.0, 0] full power unclockwise return
                # [-1.0, 0 ] full power clockwise return
                # [>0, >0] left go
                # [<0, >0] right go
            # when speed:(left, right)
                # [-1 , 1] unclock return
                # [1, -1] clock return
                # [>0, 1] go right
                # [1, >0] go left
            if not (self._forward == 0.0 and self._steering == 0.0):
                self._break = 0
                if (self._forward == 0.0 and self._steering!=0.0):
                    self._speed_left = -self._steering - 0.1
                    self._speed_right = self._steering -0.1
                else:
                    self._speed_left = self._forward*(0.9 - 1.2*self._steering) # max 1.5
                    self._speed_right = self._forward*(0.9 + 1.2*self._steering)
                    if self._speed_left < 0.0 and self._speed_right < 0.0:
                        self._speed_left = 0.5*self._speed_left
                        self._speed_right = 0.5*self._speed_right
                    
            else:
                self._break = 1
                
#    def move_pub(self, run_event):
#        rate = rospy.Rate(_RATE)
#        
#        while run_event.is_set() and not rospy.is_shutdown():
#            
        
    def start(self):
        rospy.loginfo("Joystick")
        
        loop_rate = rospy.Rate(_RATE)
        
        while not rospy.is_shutdown():
            
            # make payload
            self._payload[0] = 0
            self._payload[1] = 0
            self._payload[2] = int(self._speed_right*350) # right
            self._payload[3] = int(self._speed_left*350) # left
            if self._break == 1:
                self._payload[4] = 0
            else:
                self._payload[4] = 100 # break
                
            print("left",self._payload)
            tform.make_message(self._pub_msg, 3,'wheel',4, self._payload)
            
            self.pub.publish(self._pub_msg)
            
            loop_rate.sleep()
if __name__ == "__main__":
    
    # joy
    joys = Joystick()
    
    nh = rospy.init_node("Joystick_ctrl")
    
    joys.start()


# callback joystick, and make message to slave
## the rule needs mapping of joystic x and y at the sametime

# run program