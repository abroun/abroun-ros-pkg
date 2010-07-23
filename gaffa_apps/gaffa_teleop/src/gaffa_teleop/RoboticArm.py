#-------------------------------------------------------------------------------
# ROS imports
import roslib
roslib.load_manifest( 'gaffa_teleop' )
import rospy

import math
import numpy as np
import Homog3D

from lynxmotion_arm.SSC32Driver import ServoConfig
from lynxmotion_arm.ROS_SSC32Client import ROS_SSC32Client

#-------------------------------------------------------------------------------
class DHFrame:

    def __init__( self, d, theta, a, alpha ):
        self.d = d
        self.theta = theta
        self.a = a
        self.alpha = alpha

#-------------------------------------------------------------------------------
class RoboticArm:

    DH_PROXIMAL = "Proximal"
    DH_DISTAL = "Distal"
    
    #---------------------------------------------------------------------------
    def __init__( self, dhFrames, servoConfigs, dhConvention = DH_PROXIMAL ):
        self.numJoints = len( dhFrames )
        self.dhFrames = dhFrames
        self.servoConfigs = servoConfigs
        self.dhConvention = dhConvention
        
        if len( servoConfigs ) != self.numJoints:
            raise Exception( "Number of servos should match the number of joints" )
        
        rospy.init_node( 'roboticArm', anonymous=True )
        self.rosSSC32Client = ROS_SSC32Client()
        #self.armController = ArmController( self.servoConfigs )
        self.jointAngles = [ s.minAngle for s in self.servoConfigs ]
    
    #---------------------------------------------------------------------------
    # Sends the desired joint angles to the arm. The movement speed is defined
    # in terms of radians per second. If the movement speed isn't specified
    # then it isn't sent.
    def setJointAngles( self, jointAnglesDict, movementSpeed = 0.0 ):
            
        self.jointAngles[ 0 ] = jointAnglesDict[ "base" ]
        self.jointAngles[ 1 ] = jointAnglesDict[ "shoulder" ]
        self.jointAngles[ 2 ] = jointAnglesDict[ "elbow" ]
        self.jointAngles[ 3 ] = jointAnglesDict[ "wrist_bend" ]
        self.jointAngles[ 4 ] = jointAnglesDict[ "wrist_rotate" ]
        
        self.rosSSC32Client.setServoAngles( jointAnglesDict, movementSpeed )

    #-------------------------------------------------------------------------------
    def setGripperOpen( self, gripperOpen ):
        raise Exception( "Not implemented" )
        #self.rosSSC32Client.setGripperOpen( gripperOpen )

    #-------------------------------------------------------------------------------
    def CalculateFrameMatrices( self, jointAngles ):
        
        frameMatrices = []
        concatenatedFrameMatrices = []

        if len( jointAngles ) != self.numJoints:
            raise "An angle should be provided for each frame"

        concatentedDHMatrix =  Homog3D.CreateIdentity()
        for frameIdx in range( self.numJoints ):

            dhFrame = self.dhFrames[ frameIdx ]
            
            if self.dhConvention == self.DH_PROXIMAL:
                dhMatrix = Homog3D.CreateProximalDenavitHartenburgMatrix( 
                    dhFrame.a, dhFrame.alpha, dhFrame.d, dhFrame.theta + jointAngles[ frameIdx ] )
            else: # self.dhConvention == self.DH_DISTAL:
                dhMatrix = Homog3D.CreateDistalDenavitHartenburgMatrix( 
                    dhFrame.d, dhFrame.theta + jointAngles[ frameIdx ], dhFrame.a, dhFrame.alpha )

            frameMatrices.append( dhMatrix )

            concatentedDHMatrix = concatentedDHMatrix * dhMatrix
            concatenatedFrameMatrices.append( concatentedDHMatrix )

        return ( frameMatrices, concatenatedFrameMatrices )
        
    #---------------------------------------------------------------------------
    def GetServoConfig( self, jointIdx ):
        #print "Warning: Not fully implemented"
        return self.servoConfigs[ jointIdx ]
        #return self.armController.jointServoConfigs[ jointIdx ]
        
    #---------------------------------------------------------------------------
    # Uses forward kinematics to return a numpy vector that contains the 
    # position of the end effector
    def GetEndEffectorPos( self, angles = None ):
        if angles == None:
            angles = self.jointAngles
        ( frameMatrices, concatenatedFrameMatrices ) = self.CalculateFrameMatrices( angles )
        
        translatedOrigin = concatenatedFrameMatrices[ self.numJoints - 1 ]*Homog3D.CreateVector( 0, 0, 0 )
        return translatedOrigin
        
    #---------------------------------------------------------------------------
    # Uses forward kinematics to return an array of numpy vectors that contain
    # the position of each joint
    def GetJointPositions( self, angles = None ):
        if angles == None:
            angles = self.jointAngles
        ( frameMatrices, concatenatedFrameMatrices ) = self.CalculateFrameMatrices( angles )
        
        origin = Homog3D.CreateVector( 0, 0, 0 )
        jointPositions = [m*origin for m in concatenatedFrameMatrices]
        return jointPositions
    
    #---------------------------------------------------------------------------
    # This IK solver uses a geometric method and assumes the layout of the
    # Lynxmotion arm. More work needs to be done to cope with arbitrary
    # kinematic configurations
    def CalculateJointAnglesIK( self, targetPos, curAngles = None ):
    
        targetPosFeasible = False
        if curAngles == None:
            curAngles = self.jointAngles
            
        # Work out the current angle of the end effector. The wrist bend angle
        # needs to be offset by -pi/2 as it at right angles to the to the
        # previous joint when at 0
        endEffectorAngle = curAngles[ 1 ] + curAngles[ 2 ] + (curAngles[ 3 ] - math.pi/2)
             
        newAngles = self.CalculateConstrainedEndEffectorIK( 
            targetPos, endEffectorAngle, curAngles )
            
        if newAngles == None:
            STEPS_TO_SEARCH = 180
            initialEndEffectorAngle = endEffectorAngle
            
            for curSearchStep in range( STEPS_TO_SEARCH ):
                
                angleOffset = (curSearchStep+1)*math.pi/float(STEPS_TO_SEARCH)
                
                testAngle = initialEndEffectorAngle + angleOffset
                newAngles = self.CalculateConstrainedEndEffectorIK( 
                    targetPos, testAngle, curAngles )
                    
                if newAngles != None:
                    break
                    
                testAngle = initialEndEffectorAngle - angleOffset
                newAngles = self.CalculateConstrainedEndEffectorIK( 
                    targetPos, testAngle, curAngles )
                    
                if newAngles != None:
                    break
            
        return newAngles
            
    #---------------------------------------------------------------------------
    def CalculateConstrainedEndEffectorIK( self, targetPos, endEffectorAngle, curAngles = None ):
        
        def normaliseAngleToJointRange( angle, servoConfig ):
            while angle > servoConfig.maxAngle:
                angle -= 2.0*math.pi
            while angle < servoConfig.minAngle:
                angle += 2.0*math.pi
                
            return angle
        
        targetPosFeasible = False
        if curAngles == None:
            curAngles = self.jointAngles
        
        newAngles = curAngles[:]
        # First calculate base joint angle
        if abs( targetPos[ 0, 0 ] ) < 0.0001 \
            and abs( targetPos[ 1, 0 ] ) < 0.0001:
            
            # Arm is really close to the vertical so just leave the base angle as 
            # it is (it could be anything)
            pass
            
        else:
            newAngles[ 0 ] = math.atan2( targetPos[ 1, 0 ], targetPos[ 0, 0 ] )
            
        newAngles[ 0 ] = normaliseAngleToJointRange( newAngles[ 0 ], self.servoConfigs[ 0 ] )
        if newAngles[ 0 ] < self.servoConfigs[ 0 ].minAngle \
            or newAngles[ 0 ] > self.servoConfigs[ 0 ].maxAngle:
            
            # Try to find a position with the arm bent backwards
            newAngles[ 0 ] += math.pi
            newAngles[ 0 ] = normaliseAngleToJointRange( newAngles[ 0 ], self.servoConfigs[ 0 ] )
            
        # Rotate the targetPos onto the x,z plane so that we can solve for
        # the remaining angles in 2D
        targetPos = Homog3D.CreateRotationZ( -newAngles[ 0 ] )*targetPos
        targetPos[ 2, 0 ] -= self.dhFrames[ 0 ].d   # So we can calculate angles from origin
        
        l2 = self.dhFrames[ 2 ].a
        l3 = self.dhFrames[ 3 ].a
        l4 = self.dhFrames[ 4 ].d
        
        # Keep the orientation of the end effector fixed
        phi = endEffectorAngle
        
        # Use this to work back to the position of joint 4
        x = targetPos[ 0, 0 ] - l4*math.cos( phi )
        z = targetPos[ 2, 0 ] - l4*math.sin( phi )
        
        # Now use the law of cosines to calculate angle 2 and 3
        cosAngle2 = (x*x + z*z - l2*l2 - l3*l3)/(2*l2*l3)
        if cosAngle2 >= -1.0 and cosAngle2 <= 1.0:
            newAngles[ 2 ] = -math.acos( cosAngle2 )    # Use -ve angle to keep elbow up
            
            if newAngles[ 2 ] >= self.servoConfigs[2].minAngle \
                and newAngles[ 2 ] <= self.servoConfigs[2].maxAngle:
            
                newAngles[ 1 ] = math.asin( l3*math.sin( abs( newAngles[ 2 ] ) )/math.sqrt( x*x + z*z ) ) + math.atan2( z, x )
                newAngles[ 3 ] = phi - ( newAngles[ 1 ] + newAngles[ 2 ] )
                newAngles[ 3 ] += math.pi/2.0                
                
                #print "angle1", newAngles[ 1 ]*180.0/math.pi
                #print "angle3", newAngles[ 3 ]*180.0/math.pi
                
                newAngles[ 1 ] = normaliseAngleToJointRange( newAngles[ 1 ], self.servoConfigs[ 1 ] )
                newAngles[ 3 ] = normaliseAngleToJointRange( newAngles[ 3 ], self.servoConfigs[ 3 ] )
                
                # Validate the angles
                if newAngles[ 0 ] >= self.servoConfigs[ 0 ].minAngle \
                    and newAngles[ 0 ] <= self.servoConfigs[ 0 ].maxAngle:
                    
                    
                    if newAngles[ 1 ] >= self.servoConfigs[ 1 ].minAngle \
                        and newAngles[ 1 ] <= self.servoConfigs[ 1 ].maxAngle:
                        if newAngles[ 3 ] >= self.servoConfigs[ 3 ].minAngle \
                            and newAngles[ 3 ] <= self.servoConfigs[ 3 ].maxAngle:
                
                            targetPosFeasible = True
                        #else:
                        #    print "Joint3 constrained"
                    #else:
                    #    print "Joint1 constrained"
                #else:
                #    print "Joint0 constrained, trying to set", newAngles[ 0 ]*180.0/math.pi
            #else:
            #    print "Joint2 constrained"
                
        #else:
        #    print "Not reachable"

        if targetPosFeasible:
            return newAngles
        else:
            return None