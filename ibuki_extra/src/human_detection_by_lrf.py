#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
import time
import matplotlib.pyplot as plt
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_euler
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32
import numpy as np
from silva_beta.msg import Evans
import transformations as tform

class LRF_Control(object):
	def __init__(self):
		self.loop_rate = rospy.Rate(10)
		self.pos_r1 = 0
		self.pos_theta1 = 0
		self.pos_x1 = 0
		self.pos_y1 = 0
		self.pos_r2 = 0
		self.pos_theta2 = 0
		self.pos_x2 = 0
		self.pos_y2 = 0
		self.counter = 0
		self.eye_gaze = 0.0
		self._headc = [0.0, 0.0, 0.0, 0.0, 0.0]
		self._payload_headc = [0.0, 0.0, 0.0, 0.0, 0.0]
        
		self._pub_headc = Evans()
        
		self.minimum_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size = 1,latch=True)
		self.direction_pub = rospy.Publisher('/sound_direction', Int32, queue_size = 1)
		self.pub = rospy.Publisher('/silva/auto_local/ch2', Evans, queue_size=10)

		rospy.Subscriber('/scan', LaserScan, self.pub_pos)
		rospy.Subscriber('/silva/auto_local/ch1', Evans, self.gaze)
	
	def gaze(self,msg):
		if msg.name == 'headc':
		    self.eye_gaze = msg.payload[2]
        
	def pub_pos(self, msg):
		self.counter += 1
		if self.counter%10 != 0:
				return
		print(rospy.Time.now())
		count = msg.scan_time / msg.time_increment
		upper_angle1 = 218.5
		upper_angle2 = 180
		upper_angle3 = 270
		upper_angle4 = 135
		lower_angle1 = 180
		lower_angle2 = 141
		lower_angle3 = 225
		lower_angle4 = 90
		s_range = 0.40
		l_range = 1.5
		l_range2 = 0.80
		thr1 = 0.20
		thr2 = 0.20
		thr3 = 0.10
		thr4 = 0.35
		thr5 = 0.35
		ibegin = 0
		iend = 0
		num = int(0)
		total = float(0)
		total_angle = int(0)
		near_point = int(0)
		minimum_list_x = []
		minimum_list_y = []
		pair_minimum_x = []
		pair_minimum_y = []
		for i in range(int(count)):
			if s_range < msg.ranges[i] < l_range2:
				angle = msg.angle_increment * i * 180.0 / 3.14159265358979
				if (lower_angle1 < angle < upper_angle1) or (lower_angle2 < angle <= upper_angle2)or (lower_angle3 < angle <= upper_angle3)or (lower_angle4 < angle <= upper_angle4):
					if (i > 1) and (i < int(count) - 1):
						if (msg.ranges[i - 1] - msg.ranges[i] > thr1):
							ibegin = i
							iend = i
							self.pos_r1 = msg.ranges[i]
							self.pos_theta1 = angle
							self.pos_x1 = -1 * self.pos_r1 * math.cos(self.pos_theta1 * 3.14159265358979 / 180.0)
							self.pos_y1 = -1 * self.pos_r1 * math.sin(self.pos_theta1 * 3.14159265358979 / 180.0)
						if msg.ranges[i + 1] - msg.ranges[i] > thr2:
							iend = i
							self.pos_r2 = msg.ranges[i]
							self.pos_theta2 = angle
							self.pos_x2 = -1 * self.pos_r2 * math.cos(self.pos_theta2 * 3.14159265358979 / 180.0)
							self.pos_y2 = -1 * self.pos_r2 * math.sin(self.pos_theta2 * 3.14159265358979 / 180.0)
							dist = ((self.pos_x2 - self.pos_x1)**2 + (self.pos_y2 - self.pos_y1)**2)**(1.0/2.0)
							if (thr3 < dist < thr4):
								minimum_list_x.append((self.pos_x2 + self.pos_x1) / 2.0)
								minimum_list_y.append((self.pos_y2 + self.pos_y1) / 2.0)
			elif l_range2 < msg.ranges[i] < l_range:
				angle = msg.angle_increment * i * 180.0 / 3.14159265358979
				if (lower_angle1 < angle < upper_angle1) or (lower_angle2 < angle <= upper_angle2):
					if (i > 1) and (i < int(count) - 1):
						if (msg.ranges[i - 1] - msg.ranges[i] > thr1):
							ibegin2 = i
							iend2 = i
							self.pos_r1 = msg.ranges[i]
							self.pos_theta1 = angle
							self.pos_x1 = -1 * self.pos_r1 * math.cos(self.pos_theta1 * 3.14159265358979 / 180.0)
							self.pos_y1 = -1 * self.pos_r1 * math.sin(self.pos_theta1 * 3.14159265358979 / 180.0)
						if msg.ranges[i + 1] - msg.ranges[i] > thr2:
							iend2 = i
							self.pos_r2 = msg.ranges[i]
							self.pos_theta2 = angle
							self.pos_x2 = -1 * self.pos_r2 * math.cos(self.pos_theta2 * 3.14159265358979 / 180.0)
							self.pos_y2 = -1 * self.pos_r2 * math.sin(self.pos_theta2 * 3.14159265358979 / 180.0)
							dist = ((self.pos_x2 - self.pos_x1)**2 + (self.pos_y2 - self.pos_y1)**2)**(1.0/2.0)
							if (thr3 < dist < thr4):
								minimum_list_x.append((self.pos_x2 + self.pos_x1) / 2.0)
								minimum_list_y.append((self.pos_y2 + self.pos_y1) / 2.0)
		if len(minimum_list_x) >= 2:
			for i in range(len(minimum_list_x)-1):
				if ((minimum_list_x[i] - minimum_list_x[i + 1])**2 + (minimum_list_y[i] - minimum_list_y[i + 1])**2)**(1.0/2.0) < thr5:
					pair_minimum_x.append((minimum_list_x[i] + minimum_list_x[i + 1]) / 2.0)
					pair_minimum_y.append((minimum_list_y[i] + minimum_list_y[i + 1]) / 2.0)
					minimum_pos = PoseStamped()
					minimum_pos.header.stamp = rospy.Time.now()
					minimum_pos.header.frame_id = 'laser'
					minimum_pos.pose.position.x = pair_minimum_x[0]
					minimum_pos.pose.position.y = pair_minimum_y[0]
					if pair_minimum_y[0] < 0:
						minimum_pos.pose.position.y = pair_minimum_y[0] + 0.4
					else:
						minimum_pos.pose.position.y = pair_minimum_y[0] - 0.4
					minimum_pos.pose.position.z = 0.0
					q = quaternion_from_euler(0, 0, 0)
					minimum_pos.pose.orientation = Quaternion(*q)
					self.minimum_pub.publish(minimum_pos)
					angle_temp = math.atan2(pair_minimum_y[0],pair_minimum_x[0])
					if angle_temp > 0:
						angle_send = int(((-angle_temp + math.pi)/math.pi)*180)
					else:
						angle_send = int(((-angle_temp - math.pi)/math.pi)*180)

					self.direction_pub.publish(angle_send)
                    
					self._payload_headc[2] = -self.eye_gaze
					self._payload_headc[3] = -self.eye_gaze
					tform.make_message(self._pub_headc, 4, 'headc', 2, self._payload_headc)
					self.pub.publish(self._pub_headc)
                    
#			print 'x:',minimum_list_x
#			print 'y:',minimum_list_y
		elif len(minimum_list_x) > 0:
			minimum_pos = PoseStamped()
			minimum_pos.header.stamp = rospy.Time.now()
			minimum_pos.header.frame_id = 'laser'
			minimum_pos.pose.position.x = minimum_list_x[0]
			minimum_pos.pose.position.y = minimum_list_y[0]
#			if minimum_list_y[0] < 0:
#				minimum_pos.pose.position.y = minimum_list_y[0] + 0.4
#			else:
#				minimum_pos.pose.position.y = minimum_list_y[0] - 0.4
			minimum_pos.pose.position.z = 0.0
			q = quaternion_from_euler(0, 0, 0)
			minimum_pos.pose.orientation = Quaternion(*q)
			self.minimum_pub.publish(minimum_pos)
            
			angle_temp = math.atan2(minimum_list_y[0],minimum_list_x[0])
			angle_send = 180
			if angle_temp > 0:
				angle_send = int(((-angle_temp + math.pi)/math.pi)*180)
			else:
				angle_send = int(((-angle_temp - math.pi)/math.pi)*180)
				self.direction_pub.publish(angle_send)
                
				self._headc[2] = -self.eye_gaze
				self._headc[3] = -self.eye_gaze
				tform.make_message(self._pub_headc, 4, 'headc', 2, self._payload_headc)
				self.pub.publish(self._pub_headc)
                
		else:
			minimum_pos = PoseStamped()
			minimum_pos.header.stamp = rospy.Time.now()
			minimum_pos.header.frame_id = 'laser'
			minimum_pos.pose.position.x = 0
			minimum_pos.pose.position.y = 0
			minimum_pos.pose.position.z = 0.0
			q = quaternion_from_euler(0, 0, 0)
			minimum_pos.pose.orientation = Quaternion(*q)
			self.minimum_pub.publish(minimum_pos)
	
	def start(self):
		rospy.loginfo("In attesa")
		while not rospy.is_shutdown():
			self.loop_rate.sleep()

if __name__ == '__main__':
	rospy.init_node('human_detection_test')
	control = LRF_Control()
	control.start()
