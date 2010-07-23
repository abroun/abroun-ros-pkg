#! /usr/bin/python
# ROS imports
import roslib
roslib.load_manifest( 'lynxmotion_arm' )
import rospy

from lynxmotion_arm.msg import *

import time
import math

from SSC32Driver import SSC32Driver, ServoConfig

#-------------------------------------------------------------------------------
class ROS_SSC32Client():
    
    #---------------------------------------------------------------------------
    def __init__( self, setServoAnglesTopicName = "/setServoAngles" ):
        
        self.setServoAnglesPublisher = rospy.Publisher( setServoAnglesTopicName, 
            lynxmotion_arm.msg.SetServoAngles )

    #---------------------------------------------------------------------------
    def setServoAngles( self, servoAnglesDict, movementSpeed = 0.0 ):
        
        servoAngles = [ { "servoName" : key, "angle" : servoAngles[ key ] } for key in servoAnglesDict ]
        self.setServoAnglesPublisher.publish( lynxmotion_arm.msg.SetServoAngles(
            servoAngles = servoAngles, movementSpeed = movementSpeed ) )
            
