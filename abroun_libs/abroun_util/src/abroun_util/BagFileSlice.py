#! /usr/bin/env python
# Utility script for cutting up bagfiles and stitching them back together

import os.path
import sys
from optparse import OptionParser

import roslib
roslib.load_manifest( 'abroun_util' )
import rosbag
from roslib.rostime import Time

#-------------------------------------------------------------------------------
parser = OptionParser()
parser.add_option( "-o", "--output", dest="outputFilename", default="out.bag",
                  help="Write output to file", metavar="FILE")

(options, args) = parser.parse_args()

bagFileSlices = []

numArgs = len( args )
argIdx = 0
while argIdx < numArgs:
    # Read in the filename
    bagFilename = args[ argIdx ]
    if not os.path.exists( bagFilename ):
        print "Error: Unable to find bag file -", bagFilename
        sys.exit( -1 )
        
    #Read in the start and end times
    startTime = None
    endTime = None
    
    if argIdx + 1 < numArgs:
        try:
            startTime = float( args[ argIdx + 1 ] )
        except:
            pass    # Catch parse error and ignore
            
    if argIdx + 2 < numArgs:
        try:
            endTime = float( args[ argIdx + 2 ] )
        except:
            pass    # Catch parse error and ignore
    
    # Check that the start and end times are valid
    if startTime == None:
        print "Error: No valid start time found for", bagFilename
        sys.exit( -1 )
    if endTime == None:
        print "Error: No valid end time found for", bagFilename
        sys.exit( -1 )

    if startTime >= endTime:
        print "Error: Start time greater than or equal to end time for", bagFilename
        sys.exit( -1 )
        
    # Add the bagfile to the list to be processed
    bagFileSlices.append( ( bagFilename, startTime, endTime ) )
    
    argIdx += 3
    
# Create the new bag file from the slices
outputBag = rosbag.Bag( options.outputFilename, "w" )

lastTime = 0.0
curSeq = 0
for bagFileSlice in bagFileSlices:
    bag = rosbag.Bag( bagFileSlice[ 0 ] )
    startTime = bagFileSlice[ 1 ]
    endTime = bagFileSlice[ 2 ]
    
    timeOffset = lastTime
    numMsgsCopied = 0
    bagStartTimeSec = None
    for topic, msg, t in bag.read_messages():
        
        timeSec = t.to_sec()
        if bagStartTimeSec == None:
            bagStartTimeSec = timeSec

        timeSec -= bagStartTimeSec
        if timeSec < startTime:
            continue
        elif timeSec > endTime:
            break
        
        curTime = timeOffset + timeSec - startTime
        rosTime = Time.from_sec( curTime )
        msg.header.seq = curSeq
        msg.header.stamp = rosTime
        outputBag.write( topic, msg, rosTime )
        
        numMsgsCopied += 1
        curSeq += 1
        lastTime = curTime

    if numMsgsCopied == 0:
        print "No messages found within given time period for", bagFileSlice[ 0 ]
    else:
        print numMsgsCopied
        
    bag.close()
    
outputBag.close()
        
    
