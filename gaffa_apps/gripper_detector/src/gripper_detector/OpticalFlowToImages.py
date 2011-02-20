#! /usr/bin/env python
# Extracts images from a given bag file and saves them as jpegs

import os.path
import sys
from optparse import OptionParser

import roslib
roslib.load_manifest( 'gripper_detector' )
import rosbag
from roslib.rostime import Time
import cv

from OpticalFlowFilter import OpticalFlowFilter

OPTICAL_FLOW_BLOCK_WIDTH = 8
OPTICAL_FLOW_BLOCK_HEIGHT = 8
OPTICAL_FLOW_RANGE_WIDTH = 8    # Range to look outside of a block for motion
OPTICAL_FLOW_RANGE_HEIGHT = 8

opticalFlowFilter = OpticalFlowFilter(
    OPTICAL_FLOW_BLOCK_WIDTH, OPTICAL_FLOW_BLOCK_HEIGHT,
    OPTICAL_FLOW_RANGE_WIDTH, OPTICAL_FLOW_RANGE_HEIGHT )

#-------------------------------------------------------------------------------
parser = OptionParser()
parser.add_option( "-p", "--prefix", dest="prefix", default="image_",
                  help="Prefix of output images", metavar="FILE" )
parser.add_option( "-i", "--increment", dest="frameIncrement", default="1",
                  help="Number of frames to advance with each step" )
parser.add_option( "-f", "--format", dest="imageFormat", default="png",
                  help="Image format" )

(options, args) = parser.parse_args()

options.frameIncrement = int( options.frameIncrement )
if options.frameIncrement < 1:
    options.frameIncrement = 1

frameIdx = -1
nextFrameIdx = 0

if len( args ) <= 0:
    print "Error: No bag file supplied"
else:
    bag = rosbag.Bag( args[ 0 ] )
    imageIdx = 0

    for topic, msg, t in bag.read_messages():
        
        if msg._type == "sensor_msgs/Image":
        
            frameIdx += 1
            if frameIdx == nextFrameIdx:
                nextFrameIdx += options.frameIncrement
            else:
                continue

            if msg.encoding == "rgb8" or msg.encoding == "bgr8":
            
                if imageIdx > 0:
                    sys.stdout.write( "\r" )
            
                sys.stdout.write( "Processing image " + str( imageIdx ) )
                sys.stdout.flush()
            
                # Extract the image and save using OpenCV
                curImage = cv.CreateImageHeader( ( msg.width, msg.height ), cv.IPL_DEPTH_8U, 3 )
                cv.SetData( curImage, msg.data, msg.step )
                
                # Look for optical flow between this image and the last one
                curImageGray = cv.CreateImage( ( msg.width, msg.height ), cv.IPL_DEPTH_8U, 1 )
                cv.CvtColor( curImage, curImageGray, cv.CV_RGB2GRAY )
                
                opticalFlowX, opticalFlowY = opticalFlowFilter.calcOpticalFlow( curImageGray )
            
                # Draw the optical flow if it's available
                if opticalFlowX != None and opticalFlowY != None:

                    lineColor = cv.CV_RGB( 0, 255, 0 )
            
                    blockCentreY = OPTICAL_FLOW_BLOCK_HEIGHT / 2
                    for y in range( opticalFlowX.shape[ 0 ] ):
                
                        blockCentreX = OPTICAL_FLOW_BLOCK_WIDTH / 2
                        for x in range( opticalFlowX.shape[ 1 ] ):
                        
                            endX = blockCentreX + cv.Get2D( opticalFlowX, y, x )[ 0 ]
                            endY = blockCentreY + cv.Get2D( opticalFlowY, y, x )[ 0 ]
                        
                            cv.Line( curImage, ( int( blockCentreX ), int( blockCentreY ) ),
                                ( int( endX ), int( endY ) ), lineColor )    
                        
                            blockCentreX += OPTICAL_FLOW_BLOCK_WIDTH
                        
                        blockCentreY += OPTICAL_FLOW_BLOCK_HEIGHT  
                
                # Save the image
                cv.CvtColor( curImage, curImage, cv.CV_RGB2BGR )
                
                imageFilename = "{0}{1:08d}.{2}".format( options.prefix, imageIdx, options.imageFormat )
                cv.SaveImage( imageFilename, curImage )
                imageIdx += 1
                
sys.stdout.write( "\n" )
sys.stdout.flush()
