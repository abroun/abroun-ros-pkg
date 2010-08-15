#-------------------------------------------------------------------------------
# Python libUSB driver for talking to the Maplin USB arm
#
# Based on the C code by Vincent Sanders <vince@kyllikki.org>
#
# Released under the MIT licence
#-------------------------------------------------------------------------------

import usb
import inspect
import time
import struct

#-------------------------------------------------------------------------------
class MotorStates:
    
    OFF = 0
    FORWARD = 1
    BACKWARD = 2

#-------------------------------------------------------------------------------
class ArmDriver:
    
    NUM_MOTORS = 5
    DEFAULT_MOTOR_TIMEOUT = 0.5
    
    # Probably not good to spam the arm so limit the updates
    MAX_UPDATE_RATE = 60.0  # Updates per second
    
    # Valid motor states
    VALID_MOTOR_STATES = [ MotorStates.OFF, MotorStates.FORWARD, MotorStates.BACKWARD ]
    
    # USB constants
    ARM_ID_VENDOR = 0x1267
    ARM_ID_PRODUCT = 0x0000
    CONFIG_IDX = 0
    INTERFACE_IDX = 0
    
    #---------------------------------------------------------------------------
    def __init__( self ):
        
        self.motorTimeouts = [ self.DEFAULT_MOTOR_TIMEOUT for i in range( self.NUM_MOTORS ) ]
        self.motorStates = [ MotorStates.OFF for i in range( self.NUM_MOTORS ) ]
        self.motorStartTimes = [ 0.0 for i in range( self.NUM_MOTORS ) ]
        self.lastUpdateTime = 0.0
        
        # Connect to the arm
        self.armDevice = self.findUSBDevice( 
            idVendor=self.ARM_ID_VENDOR, idProduct=self.ARM_ID_PRODUCT )
        if self.armDevice == None:
            raise Exception( "Unable to find USB arm" )
        
        self.armDeviceHandle = self.armDevice.open()
        self.armDeviceHandle.setConfiguration( self.armDevice.configurations[ self.CONFIG_IDX ] )
        self.armDeviceHandle.claimInterface( self.INTERFACE_IDX )
        self.armDeviceHandle.setAltInterface( self.INTERFACE_IDX )

    #---------------------------------------------------------------------------
    def __del__( self ):
        
        self.armDeviceHandle.releaseInterface()
        del self.armDeviceHandle

    #---------------------------------------------------------------------------
    def findUSBDevice( self, idVendor, idProduct ):
    
        for bus in usb.busses():
            for dev in bus.devices:
                if dev.idVendor == idVendor and dev.idProduct == idProduct:
                    return dev
                
        return None
        
    #---------------------------------------------------------------------------
    def setMotorTimeout( self, motorIdx, timeout ):
        self.motorTimeouts[ motorIdx ] = timeout
        
    #---------------------------------------------------------------------------
    def setMotorState( self, motorIdx, state ):
        if state in self.VALID_MOTOR_STATES:
            self.motorStates[ motorIdx ] = state
            self.motorStartTimes[ motorIdx ] = time.time()
        
    #---------------------------------------------------------------------------
    def update( self ):
        
        curTime = time.time()
        if curTime - self.lastUpdateTime > 1.0/self.MAX_UPDATE_RATE:
            
            # Turn off motors which have run for too long without another command
            for motorIdx in range( self.NUM_MOTORS ):
                if self.motorStates[ motorIdx ] != MotorStates.OFF \
                    and curTime - self.motorStartTimes[ motorIdx ] >= self.motorTimeouts[ motorIdx ]:
                    
                    self.motorStates[ motorIdx ] = MotorStates.OFF
            
            # Send the motor states to the arm
            # TODO: Fix so that this isn't hard coded to 5 motors
            dataBuffer = chr( ((self.motorStates[ 1 ]&0x3) << 6) \
                | ((self.motorStates[ 2 ]&0x3) << 4) \
                | ((self.motorStates[ 3 ]&0x3) << 2) \
                | (self.motorStates[ 4 ]&0x3) ) \
                + chr( (self.motorStates[ 0 ]&0x3) ) + chr( 0x00 )
    
            self.armDeviceHandle.controlMsg( requestType=(usb.TYPE_VENDOR | usb.RECIP_DEVICE), 
                request=6, value=0x100, index=self.INTERFACE_IDX, buffer=dataBuffer, timeout=0 )
            
            # Store the update time
            self.lastUpdateTime = curTime

