#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import cv2
import rospy
from std_msgs.msg import Int32

rospy.init_node('human_detection_cam')

pub = rospy.Publisher('isHuman', Int32, queue_size = 1)

face_cascade = cv2.CascadeClassifier('/home/nvidia/catkin_ws/src/ibuki_extra/src/haarcascade_frontalface_default.xml')
#body_cascade = cv2.CascadeClassifier('/home/nvidia/catkin_ws/src/ibuki_extra/src/haarcascade_upperbody.xml')
cap = cv2.VideoCapture(0)
while(True):
    flag = 0
    ret, img = cap.read()
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    faces = face_cascade.detectMultiScale(gray, 1.3, 5)
#    body = body_cascade.detectMultiScale(gray)
    for (x,y,w,h) in faces:
        flag = 1
        cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,0),2)
#        roi_gray = gray[y:y+h, x:x+w]
#        roi_color = img[y:y+h, x:x+w]
#        faces = face_cascade.detectMultiScale(roi_gray)
#        for (ex,ey,ew,eh) in faces:
#            cv2.rectangle(roi_color,(ex,ey),(ex+ew,ey+eh),(0,255,0),2)
    if flag == 1:
        pub.publish(1)
    else:
        pub.publish(0)
    cv2.imshow('img', img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
