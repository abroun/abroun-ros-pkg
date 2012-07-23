#! /usr/bin/env python
# Extracts images from a given bag file and saves them as jpegs

import os.path
import sys
from optparse import OptionParser

import roslib
roslib.load_manifest( 'abroun_util' )
import rosbag
#from roslib.rostime import Time
import cv

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
                #cv.CvtColor( curImage, curImage, cv.CV_RGB2BGR )
                
                imageFilename = "{0}{1:08d}.{2}".format( options.prefix, imageIdx, options.imageFormat )
                cv.SaveImage( imageFilename, curImage )
                imageIdx += 1
                
sys.stdout.write( "\n" )
sys.stdout.flush()
