#!/usr/bin/env python2

import rospy
import modules.motion as motion

angles = {0:10, 1:-10, 13:10}

if __name__ == "__main__":
    nh = rospy.init_node("setangles")
    s = motion.Joints()
    
    rospy.loginfo("DEMO STARTED")

    s.reset()
    while not rospy.is_shutdown():
        
        s.set_angles_to(angles, 2)
        rospy.sleep(1)
    
    rospy.loginfo("DEMO FINISHED")
    
