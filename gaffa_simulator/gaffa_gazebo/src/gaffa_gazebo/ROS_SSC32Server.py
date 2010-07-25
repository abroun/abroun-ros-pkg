#! /usr/bin/python
# ROS imports
import roslib
roslib.load_manifest( 'gaffa_gazebo' )
import rospy

from std_msgs.msg import Float64
from arm_driver_msgs.msg import *

import time
import math

#print dir( lynxmotion_arm )

from lynxmotion_arm.SSC32Driver import ServoConfig

#-------------------------------------------------------------------------------
# Contains the topics and configuration data for a a simulated servo
class SimServoData:
    
    def __init__( self, servoConfig, simControlTopic ):
        
        self.servoConfig = servoConfig
        self.simControlTopic = simControlTopic
        
#-------------------------------------------------------------------------------
class ROS_SSC32Server():
    ''' This class provides ROS topics that emulate the SSC32 ROS server and
        can be used to drive the simulated robot '''
    
    #---------------------------------------------------------------------------
    def __init__( self ):
                
        self.simServoDataDict = {}
                
        # Start the node and configure the simulated servos
        rospy.init_node( 'ssc32', anonymous=True )
        self.configureServos()
        
        # Setup topics so that  users can control the servos
        self.setServoAnglesTopic = rospy.Subscriber( "setServoAngles", 
            arm_driver_msgs.msg.SetServoAngles, self.setServoAnglesCallback )
    
    #---------------------------------------------------------------------------
    def configureServos( self ):
        
        # Clear out old simulated servos
        for servoName in self.simServoDataDict:
            
            simServoData = self.simServoDataDict[ servoName ]
            simServoData.simControlTopic.unregister()
            
        self.simServoDataDict = {}
    
        # Read the new configuration data
        servoConfigData = rospy.get_param( "~servoConfigData" )
    
        # Process the configuration data
        for servoName in servoConfigData:
            
            # Transfer data to ServoConfig class and register with the topic
            # that will control the simulated servo
            servoConfig = ServoConfig( **servoConfigData[ servoName ] )
            simControlTopic = rospy.Publisher( "/{0}_position_controller/command".format( servoName ), 
                Float64 )
            
            self.simServoDataDict[ servoName ] = SimServoData( servoConfig, simControlTopic )
            
    #---------------------------------------------------------------------------
    def setServoAnglesCallback( self, request ):
        
        movementSpeed = request.movementSpeed   # TODO: MAke use of this
        
        # Set each angle in turn
        for servoAngle in request.servoAngles:
            
            if servoAngle.servoName in self.simServoDataDict:
            
                simServoData = self.simServoDataDict[ servoAngle.servoName ]
                servoConfig = simServoData.servoConfig
            
                # Constrain the angle to a valid value
                jointAngle = max( servoConfig.minAngle, min( servoAngle.angle, servoConfig.maxAngle ) )
            
                simServoData.simControlTopic.publish( Float64( jointAngle ) )
            else:
                
                rospy.logerr( "Unrecognised servo name - {0}".format( servoAngle.servoName ) )
     
#-------------------------------------------------------------------------------
if __name__ == '__main__':

    server = ROS_SSC32Server()
    
    try:
        while not rospy.is_shutdown():
            time.sleep( 0.001 )
    except:
        pass
