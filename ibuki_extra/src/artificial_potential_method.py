#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Wed Jan 30 21:01:37 2019

@author: ise
"""
device_name = 'wheel'
dir_front = 5000

"ibuki utils"

import transformations as tform
import numpy as np
"ros modules"
import rospy
from silva_beta.msg import Evans
from std_msgs.msg import String, Int32
from sensor_msgs.msg import LaserScan    #changed by ise
from geometry_msgs.msg import Twist

from geometry_msgs.msg import PoseStamped

# interface of Joystick callback
from sensor_msgs.msg import Joy

_CENTER = 5

"UDP modules"
import socket
import math

joint_default = [0,0,0,0,0]

"INITIALIZATION:default must be defined!"
class Joystick(object):
    def __init__(self):
        
        # env var
        self._pub_msg = Evans()
        self.cmd_vel = Twist()
        # payload
        self._payload = [0, 0, 0, 0, 0]
        
        self.maxspeed = 2600
        self.key = None
        self.string = ''
        self.jointnow1, self.jointnow2, self.jointnow3 = 0,0,0   
        self.loop_rate = rospy.Rate(50)
        #Publisher
        
#        self.pub = rospy.Publisher('/silva/auto_local/ch2', Evans, queue_size =25)
        self.vel_pub = rospy.Publisher('/my_robo/diff_drive_controller/cmd_vel', Twist, queue_size=32)
        
        self.movingstate = 0
        self.laststate = 0
        self.deviation = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.direction = 0.0
        self.flag_back = 0
        self.stop_flag = 1
        self.rot_l = 0.0
        self.rot_r = 0.0    
        self.speed_l = 0.0
        self.speed_r = 0.0
        self.goal_x = 0.0
        self.goal_y = 0.0
        self.a1 = _CENTER
        self.a2 = _CENTER
        self.a3 = _CENTER
        self.delta_x = 0.1
        self.delta_y = 0.1
        #Subscriber
#        rospy.Subscriber('/lrf_pub',Int32, self.callback2) #changed by ise
#        rospy.Subscriber('/my_robo/diff_drive_controller/cmd_vel',Twist,self.callback3)
        rospy.Subscriber('/move_base_simple/goal',PoseStamped,self.set_goal)
#        rospy.Subscriber('/angle_info_arml',String,self.forward_or_back)
#        rospy.Subscriber('/scan',LaserScan, self.callback2) #changed by ise
        rospy.Subscriber('/scan',LaserScan, self.callback3)
        rospy.Subscriber('/joy', Joy, self.joy_cb)
        
    def set_goal(self,goal):
        self.goal_x = goal.pose.position.x
        self.goal_y = goal.pose.position.y
        
    def get_pot(self,x,y,goal_x,goal_y,human):
        if human == 1:
            goal_pot1 = np.sqrt((goal_x - x)**2 + ((goal_y - 0.5) - y)**2)
            goal_pot2 = np.sqrt((goal_x - x)**2 + ((goal_y + 0.5) - y)**2)
#            goal_pot1 = np.sqrt(((goal_x - 0.5) - x)**2 + (goal_y - y)**2)
#            goal_pot2 = np.sqrt(((goal_x + 0.5) - x)**2 + (goal_y - y)**2)
#            goal_pot1 = (goal_x - x)**2 + ((goal_y - 1.0) - y)**2
#            goal_pot2 = (goal_x - x)**2 + ((goal_y + 1.0) - y)**2
            enemy_pot = 1/(np.sqrt((goal_x - x)**2 + (goal_y - y)**2) - 0.2)
            potential = self.a1 * enemy_pot + self.a2 * goal_pot1 + self.a3 * goal_pot2
        else:
            goal_pot1 = 0.0
            goal_pot2 = 0.0
            enemy_pot = 10/(np.sqrt((self.goal_x - x)**2 + (self.goal_y - y)**2) + 0.3)
            potential = enemy_pot + self.a2 * goal_pot1 + self.a3 * goal_pot2
        return potential
    
    ### used for joystick callback of setting self.a1~a3
    def joy_cb(self, msg):
        if msg.header.frame_id == "main": # my bad
            self.a1 == msg.axes[8]*4.0 + _CENTER
            self.a2 == msg.axes[9]*4.0 + _CENTER
            self.a3 == msg.axes[10]*4.0 + _CENTER
            
        
    def callback3(self,lrf):
        count = lrf.scan_time / lrf.time_increment
        
        for i in range(int(count)):
            rad = lrf.angle_increment * i
            angle = lrf.angle_increment * i * 180.0 / 3.14159265358979
            pot_map0 = 0.0
            pot_map1 = 0.0
            pot_map2 = 0.0
            if 0.5 < lrf.ranges[i] < 2.0:
                if np.abs(lrf.ranges[i] - self.goal_x) > 0.5:
                    pot_map0 += self.get_pot(0.0,0.0,-lrf.ranges[i]*np.cos(rad),lrf.ranges[i]*np.sin(rad),0)
                    pot_map1 += self.get_pot(self.delta_x,0.0,-lrf.ranges[i]*np.cos(rad),lrf.ranges[i]*np.sin(rad),0)
                    pot_map2 += self.get_pot(0.0,self.delta_y,-lrf.ranges[i]*np.cos(rad),lrf.ranges[i]*np.sin(rad),0)
        pot_map0 += self.get_pot(0.0,0.0,self.goal_x,self.goal_y,1)
        pot_map1 += self.get_pot(self.delta_x,0.0,self.goal_x,self.goal_y,1)
        pot_map2 += self.get_pot(0.0,self.delta_y,self.goal_x,self.goal_y,1)
        V_x = -(pot_map1 - pot_map0)/ self.delta_x
        V_y = -(pot_map2 - pot_map0)/ self.delta_y
        omega = np.arctan2(V_y,V_x)
        
        V = np.sqrt(V_x**2 + V_y**2)/25
        if V > 0.22:
            V = 0.22
            
        if (omega > (np.pi /2.0)) or (omega < -(np.pi /2.0)):
            V_x = -V_x
            V_y = -V_y
            omega = np.arctan2(V_y,V_x)
            if V >0.20:
                V = 0.20
            V *= -1
            
        
        if omega > np.pi / 2.0:
            omega = np.pi / 2.0
        elif omega < -np.pi / 2.0:
            omega = -np.pi / 2.0
        if (self.goal_x == 0.0) and (self.goal_y == 0.0):
            V = 0.0
            omega = 0.0
        self.cmd_vel.linear.x = V
        self.cmd_vel.angular.z = omega
        self.vel_pub.publish(self.cmd_vel)

    def start(self):
        rospy.loginfo("In attesa")
        
        while not rospy.is_shutdown():

            # make message 
            self._payload = [200, 200, self.speed_r, self.speed_l, 600]
            tform.make_message(self._pub_msg, 4, 'wheel', 5, self._payload)
            # publish
#            self.pub.publish(self._pub_msg)
            
            #self.pub.publish(message)
            self.loop_rate.sleep()

    def fake(self,msg):
        print('hello')


if __name__ == "__main__":
    rospy.init_node('Artificial_potential'+device_name) #changed by ise

    joystickA = Joystick()
    joystickA.start()

