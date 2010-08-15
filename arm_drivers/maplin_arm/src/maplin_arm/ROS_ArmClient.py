#! /usr/bin/python
# ROS imports
import roslib
roslib.load_manifest( 'maplin_arm' )
import rospy

from maplin_arm.msg import *

import time
import math

#-------------------------------------------------------------------------------
class ROS_ArmClient():
    
    #---------------------------------------------------------------------------
    def __init__( self, setArmMotorStatesTopicName = "/setArmMotorStates" ):
        
        self.setArmMotorStatesPublisher = rospy.Publisher( setArmMotorStatesTopicName, 
            maplin_arm.msg.SetMotorStates )

    #---------------------------------------------------------------------------
    def setArmMotorStates( self, motorStatesList ):
        
        motorStates = [ maplin_arm.msg.MotorState( 
            motorIdx=state[ 0 ], motorState=state[ 1 ] ) for state in motorStatesList ]

        msg = maplin_arm.msg.SetMotorStates( motorStates = motorStates )
        msg.header.stamp = rospy.Time.now()

        self.setArmMotorStatesPublisher.publish( msg )
