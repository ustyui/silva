#!/usr/bin/env python
import rospy
import utils, topics
from silva_core.msg import Evans

class Joints:
    """
    designate a robot as a combination of Joints
    """
    
    def __init__(self,ns="/ibuki/"):
        self.ns = ns
        self.joints = None
        self.angles = None
        self.pub = [[],[],[],[]]
        self.pub_msg = Evans()
        
        rospy.loginfo("[DEBUG]silva motion module is activated.")
        rospy.sleep(0.1)
        robot_name = rospy.get_param('ROBOT_NAME')
        driveunits = rospy.get_param(robot_name+'/DRIVE_UNITS')
        # double check
        
        checksum = utils.read_param(robot_name, robot_name)['Config']['driveunits']

        if driveunits != checksum:
            rospy.logerr("Wrong Relationship! Restart the System!")
        
        # read joint dict and initialize
        self.joints = {}
        self.zeros = {}
        for idx in range(0, driveunits):
            self.joints[idx] = 0
            self.zeros[idx] = 0
             
        rospy.loginfo("Joints populated")
        
        # create publisher
        self.pub[0] = rospy.Publisher(topics.idle[0], Evans, queue_size=3)
        self.pub[1] = rospy.Publisher(topics.reflex[0], Evans, queue_size=3)
        self.pub[2] = rospy.Publisher(topics.slave[0], Evans, queue_size=3)
        self.pub[3] = rospy.Publisher(topics.auto[0], Evans, queue_size=3)
        
#    def _cb_joints(self, msg):
#        return None
    
    def get_angles(self):
        return None    
    def set_angles(self, angles):
        self.angles = self.joints
        for j,v in angles.items():
            if j not in self.joints:
                rospy.logerror("Invalid joint number" +j)
                continue
            # encode joints
            self.angles[j] = v
            
        return self.angles
    def set_angles_to(self, angles, this_topic):
        encoded_angles = self.set_angles(angles).values()
        # change dict to list
        utils.make_message(self.pub_msg,'din',0,0,encoded_angles)
        self.pub[this_topic].publish(self.pub_msg)
        
    def reset(self):
        utils.make_message(self.pub_msg,'din',0,0,self.zeros.values())
        for idx in range(0,3):
            self.pub[idx].publish(self.pub_msg)
        
        