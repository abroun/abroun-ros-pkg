#! /usr/bin/python
# ROS imports
import roslib
roslib.load_manifest( 'lynxmotion_arm' )
import rospy

from arm_driver_msgs.msg import *

import time
import math

from SSC32Driver import SSC32Driver, ServoConfig

#-------------------------------------------------------------------------------
class ROS_SSC32Client():
    
    #---------------------------------------------------------------------------
    def __init__( self, setServoAnglesTopicName = "/setServoAngles" ):
        
        self.setServoAnglesPublisher = rospy.Publisher( setServoAnglesTopicName, 
            arm_driver_msgs.msg.SetServoAngles )

    #---------------------------------------------------------------------------
    def setServoAngles( self, servoAnglesDict, movementSpeed = 0.0 ):
        
        servoAngles = [ arm_driver_msgs.msg.ServoAngle( 
            servoName=key, angle=servoAnglesDict[ key ] ) for key in servoAnglesDict ]

        msg = arm_driver_msgs.msg.SetServoAngles(
            servoAngles = servoAngles, movementSpeed = movementSpeed )
        msg.header.stamp = rospy.Time.now()

        self.setServoAnglesPublisher.publish( msg )
            
