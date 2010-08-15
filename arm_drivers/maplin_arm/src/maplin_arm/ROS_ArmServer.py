#! /usr/bin/python
# ROS imports
import roslib
roslib.load_manifest( 'maplin_arm' )
import rospy

from maplin_arm.msg import *

import time
import math

from ArmDriver import ArmDriver

#-------------------------------------------------------------------------------
class ROS_ArmServer():
    '''Provides ROS topics for controlling a maplin arm'''
    def __init__( self ):
                
        # Start the node
        rospy.init_node( 'armDriver', anonymous=True )
            
        self.armDriver = ArmDriver()
        
        # Setup topics so that  users can control the motors
        self.setArmMotorStatesTopic = rospy.Subscriber( "setArmMotorStates", 
            maplin_arm.msg.SetMotorStates, self.setArmMotorStatesCallback )
            
    #---------------------------------------------------------------------------
    def update( self ):
        self.armDriver.update()
            
    #---------------------------------------------------------------------------
    def setArmMotorStatesCallback( self, request ):
        
        for stateRequest in request.motorStates:
            if stateRequest.motorIdx >= 0 and stateRequest.motorIdx < ArmDriver.NUM_MOTORS:
                self.armDriver.setMotorState( stateRequest.motorIdx, stateRequest.motorState )
            else:
                rospy.logerr( "Command sent to invalid motorIdx ({0})".format( stateRequest.motorIdx ) )
     
#-------------------------------------------------------------------------------
if __name__ == '__main__':

    UPDATES_PER_SECOND = 60.0

    server = ROS_ArmServer()
    
    try:
        while not rospy.is_shutdown():
            server.update()
            time.sleep( 1.0/UPDATES_PER_SECOND )
    except:
        pass
