#! /usr/bin/python
# ROS imports
import roslib
roslib.load_manifest( 'lynxmotion_arm' )
import rospy

#from std_msgs.msg import Float64
#from robotis.srv import None_Float
#from robotis.srv import None_FloatResponse
#from robotis.srv import MoveAng
#from robotis.srv import MoveAngResponse
#from robotis.srv import None_Int32
#from robotis.srv import None_Int32Response

#import robotis.lib_robotis as rs
import time
import math

from SSC32Driver import SSC32Driver, ServoConfig

#-------------------------------------------------------------------------------
class ROS_SSC32Server():
    # This class provides ROS services for a select few lib_robotis functions
    def __init__( self, name = '' ):
        
        self.name = name
        
        try:
            rospy.init_node( 'ssc32_server_' + self.name )
        except rospy.ROSException:
            pass

        #self.servo.disable_torque()

        #rospy.logout( 'ROS_Robotis_Servo: Starting Server /robotis/servo_' + self.name )
        #self.channel = rospy.Publisher('/robotis/servo_' + self.name, Float64)

        #self.__service_ang = rospy.Service('/robotis/servo_' + name + '_readangle',
                                           #None_Float, self.__cb_readangle)

        #self.__service_ismove = rospy.Service('/robotis/servo_' + name + '_ismoving',
                                              #None_Int32, self.__cb_ismoving)

        #self.__service_moveang = rospy.Service('/robotis/servo_' + name + '_moveangle',
                                               #MoveAng, self.__cb_moveangle)

    #def __cb_readangle( self, request ):
        #ang = self.update_server()
        #return None_FloatResponse( ang )

    #def __cb_ismoving( self, request ):
        #status = self.servo.is_moving()
        #return None_Int32Response( int(status) )

    #def __cb_moveangle( self, request ):
        #ang = request.angle
        #angvel = request.angvel
        #blocking = bool( request.blocking )
        #self.servo.move_angle( ang, angvel, blocking )
        #return MoveAngResponse()

    #def update_server(self):
    #    ang = self.servo.read_angle()
    #    self.channel.publish( Float64(ang) )
    #    return ang
     
     
#-------------------------------------------------------------------------------
if __name__ == '__main__':

    rospy.init_node( 'ssc32', anonymous=True )
    try:
        servoConfigData = rospy.get_param( "~servoConfigData" )
    except:
        rospy.logerr( "Unable to find servoConfigData" )
        
    print servoConfigData
    
    print ""
    
    servoConfigDict = {}
    for servoName in servoConfigData:
        servoConfigDict[ servoName ] = ServoConfig( **servoConfigData[ servoName ] )
    print servoConfigDict
    
    try:
        while not rospy.is_shutdown():
            time.sleep( 0.001 )
            #print "ASD", rospy.get_name()
    except:
        pass
