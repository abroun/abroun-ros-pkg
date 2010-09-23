#! /usr/bin/env python
# Utility script that cuts out waves from multi wave sequences

import os.path
import subprocess
import sys
from optparse import OptionParser

import roslib
roslib.load_manifest( 'gripper_detector' )
import rosbag
from roslib.rostime import Time

PADDING_TIME = 2.0

#-------------------------------------------------------------------------------
usage = "usage: %prog [options] bagFile"
parser = OptionParser( usage=usage )
parser.add_option( "-p", "--prefix", dest="outputPrefix", default="",
                  help="Prefix to output files", metavar="FILE")

(options, args) = parser.parse_args()

if len( args ) < 1:
    print "Error: Not enough arguments supplied"
    parser.print_help()
    sys.exit( -1 )

# Load in bagfile
bagFilename = args[ 0 ]
bag = rosbag.Bag( bagFilename )

# Extract messages from the bag
servoAngleTimes = []
servoAngleData = []

startTime = None
lastAngleTime = -1.0

for topic, msg, t in bag.read_messages():
    if startTime == None:
        startTime = t
        
    bagTime = t - startTime
    bagTimeSec = bagTime.to_sec()
        
    if msg._type == "arm_driver_msgs/SetServoAngles" \
        and bagTimeSec - lastAngleTime > 0.01:
        
        lastAngleTime = bagTimeSec
        servoAngleTimes.append( bagTimeSec )
        
        servoAngle = msg.servoAngles[ 0 ].angle
        servoAngleData.append( servoAngle )
        
bag.close()

# Work out the number of waves and when they start and end
meanServoAngle = sum( servoAngleData ) / len( servoAngleData )

numCrossings = 0
waveEndTimes = []

for i in range( len( servoAngleData ) - 1 ):
    
    curAngle = servoAngleData[ i ]
    nextAngle = servoAngleData[ i + 1 ]
    
    if ( curAngle > meanServoAngle and nextAngle < meanServoAngle ) \
        or ( curAngle < meanServoAngle and nextAngle > meanServoAngle ):
        
        numCrossings += 1
        
        if numCrossings%2 == 0:
            waveEndTimes.append( servoAngleTimes[ i + 1 ] )
            
waveStartTime = servoAngleTimes[ 0 ]
waveEndTimes.append( servoAngleTimes[ -1 ] )

numWaves = len( waveEndTimes )
print "NumWaves =", numWaves

# Use BagFileSlice to create the new bag file
scriptPath = os.path.dirname( __file__ )

lastWaveIdx = numWaves - 1
sequenceStartTime = waveStartTime - PADDING_TIME
sequenceEndTime = waveEndTimes[ lastWaveIdx ] + PADDING_TIME

for waveIdx in range( numWaves ):
    
    args = [ "rosrun", "abroun_util", "BagFileSlice.py",
        "-o", "{0}_{1:02d}_Waves.bag".format( options.outputPrefix, waveIdx + 1 ) ]
    
    if waveIdx == lastWaveIdx:
        args += [ bagFilename, str( sequenceStartTime ), str( sequenceEndTime ) ]
    else:
        args += [ bagFilename, str( sequenceStartTime ), str( waveEndTimes[ waveIdx ] ) ]
        args += [ bagFilename, str( waveEndTimes[ lastWaveIdx ] ), str( sequenceEndTime ) ]

    print args
    p = subprocess.Popen( args )
    p.wait()
