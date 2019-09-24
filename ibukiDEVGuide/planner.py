#!/usr/bin/env python

import rospy
from darwin_gazebo.darwin import Darwin


if __name__=="__main__":
    rospy.init_node("walker_demo")
    
    rospy.loginfo("Instantiating Darwin Client")
    darwin=Darwin()
    rospy.sleep(1)
 
    rospy.loginfo("Darwin Planner Demo Starting")

    darwin.set_angles({'j_pan':-1})
    rospy.sleep(1)
    darwin.set_angles({'j_tilt':-0})
    rospy.sleep(1)
    darwin.set_angles({'j_shoulder_l':-0})
    rospy.sleep(1)
    darwin.set_angles({'j_low_arm_l':-0})
    rospy.sleep(1)
    darwin.set_angles({'j_wrist_l':-0})
    rospy.sleep(1)
    darwin.set_angles({'j_gripper_l':-0})
    rospy.sleep(1)
    darwin.set_angles({'j_wrist_l':-0})
    rospy.sleep(1)
    darwin.set_angles({'j_shoulder_r':-0})
    rospy.sleep(1)
    darwin.set_angles({'j_low_arm_r':-0})
    rospy.sleep(1)
    darwin.set_angles({'j_wrist_r':-0})
    rospy.sleep(1)
    darwin.set_angles({'j_gripper_r':-0})
    rospy.sleep(1)
    darwin.set_angles({'j_wrist_r':-0})
    rospy.sleep(1)
    darwin.set_angles({'j_pelvis_l':-0})
    rospy.sleep(1)
    darwin.set_angles({'j_thigh1_l':-0})
    rospy.sleep(1)
    darwin.set_angles({'j_thigh1_l':-0})
    rospy.sleep(1)


    
    rospy.loginfo("Darwin Planner Demo Finished")
