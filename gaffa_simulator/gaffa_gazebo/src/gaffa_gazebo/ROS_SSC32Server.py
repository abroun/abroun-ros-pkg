#! /usr/bin/python
# ROS imports
import roslib
roslib.load_manifest( 'gaffa_simulator' )
import rospy

from lynxmotion_arm.msg import *

import time
import math

from SSC32Driver import SSC32Driver, ServoConfig

#-------------------------------------------------------------------------------
class ROS_SSC32Server():
    # This class provides ROS topics that can be used to control servo motors
    # with the SSC32
    def __init__( self ):
                
        # Start the node and get configuration data
        rospy.init_node( 'ssc32', anonymous=True )
        servoConfigData = rospy.get_param( "~servoConfigData" )
    
        # Move the data into servo config objects
        servoConfigDict = {}
        for servoName in servoConfigData:
            servoConfigDict[ servoName ] = ServoConfig( **servoConfigData[ servoName ] )
            
        self.ssc32Driver = SSC32Driver( servoConfigDict )
        
        # Setup topics so that  users can control the servos
        self.setServoAnglesTopic = rospy.Subscriber( "setServoAngles", 
            lynxmotion_arm.msg.SetServoAngles, self.setServoAnglesCallback )
            
    #---------------------------------------------------------------------------
    def setServoAnglesCallback( self, request ):
        
        servoAnglesDict = {}
        for servoAngle in request.servoAngles:
            servoAnglesDict[ servoAngle.servoName ] = servoAngle.angle
            
        self.ssc32Driver.setJointAngles( servoAnglesDict, request.movementSpeed )
     
#-------------------------------------------------------------------------------
if __name__ == '__main__':

    server = ROS_SSC32Server()
    
    try:
        while not rospy.is_shutdown():
            time.sleep( 0.001 )
    except:
        pass
