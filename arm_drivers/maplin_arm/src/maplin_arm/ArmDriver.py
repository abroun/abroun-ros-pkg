#! /usr/bin/python

import usb
import inspect
import time
import struct

ARM_ID_VENDOR = 0x1267
ARM_ID_PRODUCT = 0x0000
CONFIG_IDX = 0
INTERFACE_IDX = 0

def findUSBDevice( idVendor, idProduct ):
    
    for bus in usb.busses():
        for dev in bus.devices:
            if dev.idVendor == idVendor and dev.idProduct == idProduct:
                return dev
            
    return None


# find our device
device = findUSBDevice( idVendor=ARM_ID_VENDOR, idProduct=ARM_ID_PRODUCT )

# was it found?
if not device is None:
    print "Found arm!"
    #print dir( device )
    
    #for config in device.configurations:
        #print "  Configuration:", config.value
        #print "    Total length:", config.totalLength 
        #print "    selfPowered:", config.selfPowered
        #print "    remoteWakeup:", config.remoteWakeup
        #print "    maxPower:", config.maxPower
        #for intf in config.interfaces:
            #print "    Interface:",intf[0].interfaceNumber
            #for alt in intf:
                #print "    Alternate Setting:",alt.alternateSetting
                #print "      Interface class:",alt.interfaceClass
                #print "      Interface sub class:",alt.interfaceSubClass
                #print "      Interface protocol:",alt.interfaceProtocol
                #for ep in alt.endpoints:
                    #print "      Endpoint:",hex(ep.address)
                    #print "        Type:",ep.type
                    #print "        Max packet size:",ep.maxPacketSize
                    #print "        Interval:",ep.interval
    
    deviceHandle = device.open()
    deviceHandle.setConfiguration( device.configurations[ CONFIG_IDX ] )
    deviceHandle.claimInterface( 0 )
    deviceHandle.setAltInterface( 0 )
    #deviceHandle.claimInterface( device.configurations[ CONFIG_IDX ].interfaces[ INTERFACE_IDX ] )
    #deviceHandle.setAltInterface( device.configurations[ CONFIG_IDX ].interfaces[ INTERFACE_IDX ] )
    
    MOTOR_OFF = 0
    MOTOR_FORWARD = 1
    MOTOR_BACKWARD = 2
    
    shoulderMotor = MOTOR_OFF
    elbowMotor = MOTOR_OFF
    wristMotor = MOTOR_OFF
    gripperMotor = MOTOR_OFF
    baseMotor = MOTOR_OFF
    
    dataBuffer = struct.pack( 'L', (shoulderMotor << 22) )# | b'\x00\x00\x00\x00'
    print "{0:X}".format( shoulderMotor << 22 )
    print len( dataBuffer )
    #dataBuffer = dataBuffer[1:]
    #print dataBuffer == b'\x40\x00\x00'
    dataBuffer = chr( ((shoulderMotor&0x3) << 6) \
        | ((elbowMotor&0x3) << 4) | ((wristMotor&0x3) << 2) | (gripperMotor&0x3) ) \
        + chr( (baseMotor&0x3) ) + chr( 0x00 )
    
    deviceHandle.controlMsg( requestType=(usb.TYPE_VENDOR | usb.RECIP_DEVICE), 
        request=6, value=0x100, index=INTERFACE_IDX, buffer=dataBuffer, timeout=0 );
    
    time.sleep( 0.005 )
    
    dataBuffer = b'\x00\x00\x00'
    deviceHandle.controlMsg( requestType=(usb.TYPE_VENDOR | usb.RECIP_DEVICE), 
        request=6, value=0x100, index=INTERFACE_IDX, buffer=dataBuffer, timeout=0 );
    
    print "Done"
    
    deviceHandle.releaseInterface()
    del deviceHandle
    
else:
    print "Cant find arm"
