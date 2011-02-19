#! /usr/bin/env python
# Average a sequence of images together in order to reduce noise and
# hopefully reveal more detail

import os.path
import sys
from optparse import OptionParser

import roslib
roslib.load_manifest( 'abroun_util' )
import rosbag
from roslib.rostime import Time
import cv
import numpy as np

#-------------------------------------------------------------------------------
parser = OptionParser()
parser.add_option( "-o", "--output", dest="outputFilename", default="out.png",
                  help="Output image filename", metavar="FILE" )

(options, args) = parser.parse_args()

frameAcc = None

if len( args ) <= 0:
    print "Error: No bag file supplied"
else:
    bag = rosbag.Bag( args[ 0 ] )
    imageIdx = 0

    for topic, msg, t in bag.read_messages():
        
        if msg._type == "sensor_msgs/Image":

            if msg.encoding == "rgb8" or msg.encoding == "bgr8":
            
                if imageIdx > 0:
                    sys.stdout.write( "\r" )
            
                sys.stdout.write( "Processing image " + str( imageIdx ) )
                sys.stdout.flush()
            
                # Extract the image using OpenCV
                curImage = cv.CreateMatHeader( msg.height, msg.width, cv.CV_8UC3 )
                cv.SetData( curImage, msg.data, msg.step )
                
                npImage = np.array( curImage, dtype=np.uint8 )
                
                if frameAcc == None:
                    frameAcc = np.zeros( ( msg.height, msg.width ), dtype=np.float32 )
                
                # Convert the image to grayscale and add it to the accumulator
                frameAcc = frameAcc \
                    + 0.299*npImage[ :, :, 0 ].astype( np.float32 ) \
                    + 0.587*npImage[ :, :, 1 ].astype( np.float32 ) \
                    + 0.114*npImage[ :, :, 2 ].astype( np.float32 )
                
                imageIdx += 1
                
    numImages = imageIdx
    
    sys.stdout.write( "\n" )
    sys.stdout.flush()
    
    print "Averaging image"
    
    
    averageImage = frameAcc / numImages
    averageImage[ averageImage > 148.0 ] = 0.0
    #averageImage *= 2.0
    
    
    
    averageImage[ averageImage > 255.0 ] = 255.0
    grayLevels = averageImage.astype( np.uint8 )
    
    colourImage = np.zeros( ( averageImage.shape[ 0 ], averageImage.shape[ 1 ], 3 ), dtype=np.uint8 )
    colourImage[ :, :, 0 ] = grayLevels
    colourImage[ :, :, 1 ] = grayLevels
    colourImage[ :, :, 2 ] = grayLevels
    
    cv.SaveImage( options.outputFilename, colourImage )
    
                

