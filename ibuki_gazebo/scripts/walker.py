#!/usr/bin/env python

import rospy
from ibuki_gazebo.ibuki import Ibuki


if __name__=="__main__":
    rospy.init_node("walker_demo")
    
    rospy.loginfo("Instantiating ibuki Client")
    ibuki=Ibuki()
    rospy.sleep(1)
 
    rospy.loginfo("ibuki Walker Demo Starting")


    ibuki.set_angles({'arml_r': 0.1})
    rospy.sleep(1)
    # ibuki.set_angles({'hip_y': 0.4})
    # rospy.sleep(1)
    # ibuki.set_angles({'neck_r': 0.4})
    # rospy.sleep(1)
    # ibuki.set_angles({'neck_p': -0.4})
    # rospy.sleep(1)
    # ibuki.set_angles({'neck_y': 0.4})
    # rospy.sleep(1)
    # ibuki.set_angles({'shoulderl_p': 0.4})
    # ibuki.set_angles({'shoulderr_p': 0.4})  
    # rospy.sleep(1)  
    # ibuki.set_angles({'arml_r': 0.4})    
    # ibuki.set_angles({'armr_r': -0.4}) 
    # rospy.sleep(1)
    # ibuki.set_angles({'arml_y': 0.4})    
    # ibuki.set_angles({'armr_y': -0.4})   
    # rospy.sleep(1)
    # ibuki.set_angles({'arml_p': 0.4})    
    # ibuki.set_angles({'armr_p': 0.4})
    # rospy.sleep(1)
    # ibuki.set_angles({'wristl_y': 0.4})    
    # ibuki.set_angles({'wristr_y': 0.4})
    # rospy.sleep(1)
    # ibuki.set_angles({'wristl_r': 0.4})    
    # ibuki.set_angles({'wristr_r': 0.4})

    rospy.loginfo("ibuki Walker Demo Finished")
