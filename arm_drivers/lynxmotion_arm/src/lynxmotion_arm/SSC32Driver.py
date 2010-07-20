#-------------------------------------------------------------------------------
# Talks to the SSC32 control board to control a number of servos
#-------------------------------------------------------------------------------

import math
import yaml
import serial
import time

#-------------------------------------------------------------------------------
class ServoConfig( yaml.YAMLObject ):

    yaml_tag = u'!ServoConfig'
    
    MIN_PULSE_WIDTH = 500
    MAX_PULSE_WIDTH = 2500
 
    #---------------------------------------------------------------------------
    def __init__( self, minAngle, maxAngle, startPulseWidth, endPulseWidth, channelIdx, **args ):
        self.minAngle = minAngle
        self.maxAngle = maxAngle
        self.startPulseWidth = startPulseWidth
        self.endPulseWidth = endPulseWidth
        self.channelIdx = channelIdx
        self.validateServoConfig()

    #---------------------------------------------------------------------------
    # This routine should be called whenever the config variables are changed
    # to ensure that they remain consistent
    def validateServoConfig( self ):

        self.startPulseWidth = int( self.startPulseWidth )
        self.endPulseWidth = int( self.endPulseWidth )

        if self.startPulseWidth < self.MIN_PULSE_WIDTH:
            self.startPulseWidth = self.MIN_PULSE_WIDTH
        if self.startPulseWidth > self.MAX_PULSE_WIDTH:
            self.startPulseWidth = self.MAX_PULSE_WIDTH
        if self.endPulseWidth < self.MIN_PULSE_WIDTH:
            self.endPulseWidth = self.MIN_PULSE_WIDTH
        if self.endPulseWidth > self.MAX_PULSE_WIDTH:
            self.endPulseWidth = self.MAX_PULSE_WIDTH

        if self.endPulseWidth == self.startPulseWidth:
            if self.startPulseWidth < self.MAX_PULSE_WIDTH:
                self.endPulseWidth = self.startPulseWidth + 1      
            else:
                self.endPulseWidth = self.startPulseWidth - 1      
        if self.maxAngle <= self.minAngle:
            ONE_DEGREE = math.radians( 1.0 )
            self.maxAngle = self.minAngle + ONE_DEGREE

    #---------------------------------------------------------------------------
    def getServoRange( self ):
        return self.endPulseWidth - self.startPulseWidth
        
#-------------------------------------------------------------------------------
class SSC32Driver:
    
    DEFAULT_SERIAL_PORT = "/dev/ttyUSB0"
    DEFAULT_BAUDRATE = 115200
    SERIAL_TIMEOUT = 0.5    # The max time to wait for a response

    GRIPPER_SERVO_IDX = 6
    GRIPPER_OPEN_PULSE_WIDTH = 700
    GRIPPER_CLOSED_PULSE_WIDTH = 2300

    #---------------------------------------------------------------------------
    def __init__( self, jointServoConfigDict, serialPort = None, baudRate = None ):
        
        self.jointServoConfigDict = jointServoConfigDict
        
        # Try to connect to the SSC32
        if serialPort == None:
            serialPort = self.DEFAULT_SERIAL_PORT
        if baudRate == None:
            baudRate = self.DEFAULT_BAUDRATE
            
        try:
            self.ssc32Serial = serial.Serial( serialPort, baudRate, timeout=self.SERIAL_TIMEOUT )
        except Exception, e:
            raise Exception( "No arm seems to be attached to %s".format( serialPort ) )

    #---------------------------------------------------------------------------
    # Sends the desired joint angles to the arm. The movement speed is defined
    # in terms of radians per second. If the movement speed isn't specified
    # then it isn't sent.
    def setJointAngles( self, jointAnglesDict, movementSpeed = 0.0 ):
    
        commandStr = ""
        for servoName in jointAnglesDict:
            jointAngle = jointAnglesDict[ servoName ]
            
            if not servoName in self.jointServoConfigDict:
                raise Exception( "Unrecognised servo called ", + servoName )
            
            servoConfig = self.jointServoConfigDict[ servoName ]
            
            # Constrain the angle to a valid value
            jointAngle = max( servoConfig.minAngle, min( jointAngle, servoConfig.maxAngle ) )
            
            # Convert the angle to a pulse width
            normalisedAngle = (jointAngle - servoConfig.minAngle)/(servoConfig.maxAngle - servoConfig.minAngle)
            pulseWidth = int( servoConfig.startPulseWidth 
                + normalisedAngle*(servoConfig.endPulseWidth - servoConfig.startPulseWidth) )
            
            # Add to the command string
            commandStr += "# {0} P {1} ".format( servoConfig.channelIdx, pulseWidth )

            # Add the movement speed in uS per second if needed
            movementSpeedDegrees = math.degrees( movementSpeed )
            degreesToUS = abs(servoConfig.endPulseWidth - servoConfig.startPulseWidth) \
                / abs(servoConfig.maxAngle - servoConfig.minAngle)
            movementSpeedUS = int( movementSpeedDegrees*degreesToUS )

            if movementSpeedUS > 0:
                commandStr += "S {0} ".format( movementSpeedUS )

        commandStr += "\r"
        self.ssc32Serial.write( commandStr + "\r" )

    #-------------------------------------------------------------------------------
    def setGripperOpen( self, gripperOpen ):

        if gripperOpen:
            self.ssc32Serial.write( "# {0} P {1}\r".format( self.GRIPPER_SERVO_IDX, self.GRIPPER_OPEN_PULSE_WIDTH ) )
        else:
            self.ssc32Serial.write( "# {0} P {1}\r".format( self.GRIPPER_SERVO_IDX, self.GRIPPER_CLOSED_PULSE_WIDTH ) )

    #---------------------------------------------------------------------------
    def queryJointPulseWidths( self ):
        
        commandStr = ""
        
        numServos = len( self.jointServoConfigDict )
        numServosQueried = 0
        
        for servoName in self.jointServoConfigDict:
            servoConfig = self.jointServoConfigDict[ servoName ]
            commandStr += "QP {0}".format( servoConfig.channelIdx )

            if numServosQueried < numServos - 1:
                commandStr += " "
            else:
                commandStr += "\r"
            numServosQueried += 1
                
        self.ssc32Serial.write( commandStr )
        rawServoPulseWidths = self.ssc32.receive( self.NUM_SERVOS )
        
        pulseWidths = {}
        if len( rawServoPulseWidths ) != numServos:
            print "Warning: Unable to query all joints. Only got", len( rawServoPulseWidths )
        else:
            for pulseIdx, servoName in enumerate( self.jointServoConfigDict ):
                pulseWidths[ servoName ] = ord( rawServoPulseWidths[ pulseIdx ] ) * 10
        
        return pulseWidths