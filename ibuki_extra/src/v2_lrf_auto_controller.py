#!/usr/bin/env python
# -*- coding: utf-8 -*-
device_name = 'wheel'
dir_front = 5000

"ibuki utils"

import transformations as tform

"ros modules"
import rospy
from silva_beta.msg import Evans
from std_msgs.msg import String, Int32
from sensor_msgs.msg import LaserScan    #changed by ise
from geometry_msgs.msg import Twist

"UDP modules"
import socket
import math

joint_default = [0,0,0,0,0]

"INITIALIZATION:default must be defined!"
class Joystick(object):
    def __init__(self):
        
        # env var
        self._pub_msg = Evans()
        # payload
        self._payload = [0, 0, 0, 0, 0]
        
        self.maxspeed = 2600
        self.key = None
        self.string = ''
        self.jointnow1, self.jointnow2, self.jointnow3 = 0,0,0   
        self.loop_rate = rospy.Rate(20)
        #Publisher
        
        self.pub = rospy.Publisher('/silva/auto_local/ch2', Evans, queue_size =25)
        self.speed_pub = rospy.Publisher('/speed', Twist, queue_size =25)
        self.speed = Twist()
        
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
        self.human_flag = 1
        #Subscriber
#        rospy.Subscriber('/lrf_pub',Int32, self.callback2) #changed by ise
        rospy.Subscriber('/my_robo/diff_drive_controller/cmd_vel',Twist,self.callback3)
        rospy.Subscriber('/isHuman',Int32,self.human_detect)
#        rospy.Subscriber('/angle_info_arml',String,self.forward_or_back)
        rospy.Subscriber('/scan',LaserScan, self.callback2) #changed by ise
        
    def human_detect(self,human):
        if human.data == 1:
            self.human_flag = 1
        elif (-0.05 < self.speed_l < 0.05) and (-0.05 < self.speed_r < 0.05):
            self.human_flag = 0
            

    def callback3(self,twist):
#        self.human_flag=1
        if (self.stop_flag == 0) and (self.human_flag == 1):
            self.rot_r = (twist.linear.x + 0.19 * twist.angular.z) / 0.15    #wheel-to-wheel=380, radius of wheel=150
            self.rot_l = (twist.linear.x - 0.19 * twist.angular.z) / 0.15
            if self.rot_r > 0:
                self.speed_r = ((self.rot_r / 7.0) + (1.0/48.0)) * 960
            else:
                self.speed_r = ((self.rot_r / 7.0) - (1.0/48.0)) * 960
            if self.rot_l > 0:
                self.speed_l = ((self.rot_l / 7.0) + (1.0/48.0)) * 960
            else:
                self.speed_l = ((self.rot_l / 7.0) - (1.0/48.0)) * 960
        else:
            self.speed_r = 0
            self.speed_l = 0
            
        self.speed.linear.x = 0.0
        self.speed.linear.y = 0.0
        self.speed.linear.z = 0.0
        self.speed.angular.x = self.speed_r
        self.speed.angular.y = self.speed_l
        self.speed.angular.z = 0.0
        self.speed_pub.publish(self.speed)
        #print(self.jointnow1, self.jointnow2,self.jointnow3)
        #finalmessage = self.merge_them()
#        print(finalmessage)
        
        #### SENDING ###
        #wheelsock.sendto(finalmessage, (ibuki.ip(device_name), ibuki.port(device_name)))
                
    def forward_or_back(self,angle_l):
        list_angle = tform.seperate(angle_l.data)
        if 750 < list_angle[2]:
            self.flag_back = 1
#            print ("back")
        else:
            self.flag_back = 0
#            print ("forward")
        
    def callback2(self, lrf):    #made by ise    
#        print("get")
        count = lrf.scan_time / lrf.time_increment
        num = int(0)
        nearPoint = int(0)
        farPoint = int(0)
        total = float(0)
        total_angle = int(0)
#        upper_angle1 = 90
#        upper_angle2 = 360
#        lower_angle1 = 0
#        lower_angle2 = 270
        upper_angle1 = 270
        upper_angle2 = 180
        lower_angle1 = 180
        lower_angle2 = 90
        s_range = 0.50
        l_range = 1.5
        for i in range(int(count)):
            if s_range < lrf.ranges[i] < l_range:
			angle = lrf.angle_increment * i * 180.0 / 3.14159265358979
			if (lower_angle1 < angle < upper_angle1) or (lower_angle2 < angle <= upper_angle2):
				num += 1
				total += lrf.ranges[i]
				total_angle += angle
				if lrf.ranges[i] > 0.6:
					farPoint += 1
            elif lrf.ranges[i] <= s_range:
			angle = lrf.angle_increment * i * 180.0 / 3.14159265358979
			if (lower_angle1 < angle < upper_angle1) or (lower_angle2 < angle <= upper_angle2):
				#rospy.loginfo("near range = %f[m], angle = %f[deg]",lrf.ranges[i],angle)
				nearPoint += 1
				total += lrf.ranges[i]
				total_angle += angle
        print(num,nearPoint)
        if (num > 45)and(nearPoint < 25):    #border
            print('go')
            self.stop_flag = 0
#            #talk(1)
#            print ("go")
#            K = 0.693
#            x = total / (num + nearPoint)
#            if self.movingstate == 0:
#                self.speed = 1.0
##                self.maxspeed = 2800
##                rospy.sleep(1.0)
#                self.laststate = self.movingstate
#                self.movingstate = 1
##                self.direction = 0.0
#            else:
#                t = abs(x - s_range)**(1.0/3.0)
#                self.speed = K * t + 0.3
#                print self.speed
#                self.laststate = self.movingstate
#                self.movingstate = 1

        else:
            print('stop')
            self.stop_flag = 1

    def start(self):
        rospy.loginfo("In attesa")
        
        while not rospy.is_shutdown():

            # make message 
            self._payload = [200, 200, self.speed_r, self.speed_l, 600]
            tform.make_message(self._pub_msg, 4, 'wheel', 5, self._payload)
            # publish
            self.pub.publish(self._pub_msg)
            
            #self.pub.publish(message)
            self.loop_rate.sleep()

    def fake(self,msg):
        print('hello')


if __name__ == "__main__":
    "motor command socket"
    motorsock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    wheelsock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rtclient = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    #motorsock.bind((ip_master, port_drive_headl))
    "Send To Mbed"
    rospy.init_node('lrfAutonomous_'+device_name) #changed by ise

    joystickA = Joystick()
    joystickA.start()

