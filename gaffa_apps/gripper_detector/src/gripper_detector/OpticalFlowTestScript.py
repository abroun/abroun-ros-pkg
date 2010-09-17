#! /usr/bin/python

# ROS imports
import roslib
roslib.load_manifest( 'gripper_detector' )
import rospy
from ros import rosrecord

import sys
import math
import random
import os.path
import time
import copy
from optparse import OptionParser

import numpy as np
import cv

import matplotlib
from matplotlib.figure import Figure
from matplotlib.axes import Subplot

import OpticalFlow.Utils as Utils
from OpticalFlow.InputSequence import InputSequence
from OpticalFlow.RegularisedInputSequence import RegularisedInputSequence
from OpticalFlow.CrossCorrelatedSequence import CrossCorrelatedSequence
from OpticalFlow.ROCCurve import ROCCurve
import OpticalFlow.MarkerBuffer as MarkerBuffer


    
OPTICAL_FLOW_BLOCK_WIDTH = 8
OPTICAL_FLOW_BLOCK_HEIGHT = 8
OPTICAL_FLOW_RANGE_WIDTH = 8    # Range to look outside of a block for motion
OPTICAL_FLOW_RANGE_HEIGHT = 8
OPTICAL_FLOW_METHOD = "BlockMatching"
#OPTICAL_FLOW_METHOD = "LucasKanade"
#OPTICAL_FLOW_METHOD = "HornSchunck"
COMBINATION_METHOD = "NoChange"

#GROUND_TRUTH_FILENAME = "/../../config/TopPosGripper.yaml"
#GROUND_TRUTH_FILENAME = "/../../config/ExperimentPosGripper.yaml"
GROUND_TRUTH_FILENAME = "/../../config/OnTablePosGripper.yaml"

CORRELATION_THRESHOLD = 0.63
MAX_TEST_POINT_X = (320 - OPTICAL_FLOW_BLOCK_WIDTH)/OPTICAL_FLOW_BLOCK_WIDTH - 1
MAX_TEST_POINT_Y = (240 - OPTICAL_FLOW_BLOCK_HEIGHT)/OPTICAL_FLOW_BLOCK_HEIGHT - 1

SAMPLES_PER_SECOND = 30.0
MAX_CORRELATION_LAG = 1.0

#-------------------------------------------------------------------------------
usage = "usage: %prog [options] bagFile maskFile"
parser = OptionParser( usage=usage )
#parser.add_option( "-o", "--output", dest="outputFilename", default="out.bag",
#                  help="Write output to file", metavar="FILE")

(options, args) = parser.parse_args()

if len( args ) < 2:
    print "Error: Not enough arguments supplied"
    parser.print_help()
    sys.exit( -1 )
    
bagFilename = args[ 0 ]
markerFilename = args[ 1 ]

print "Reading in bag file"
inputSequence = InputSequence( bagFilename )
#inputSequence.addDistractorObjects( 4 )
inputSequence.calculateOpticalFlow(
    OPTICAL_FLOW_BLOCK_WIDTH, OPTICAL_FLOW_BLOCK_HEIGHT,
    OPTICAL_FLOW_RANGE_WIDTH, OPTICAL_FLOW_RANGE_HEIGHT,
    OPTICAL_FLOW_METHOD )

print "Resampling data"
regularisedInputSequence = RegularisedInputSequence( 
    inputSequence, SAMPLES_PER_SECOND )

print "Performing cross correlation"
crossCorrelatedSequence = CrossCorrelatedSequence( 
    regularisedInputSequence, 
    MAX_CORRELATION_LAG, COMBINATION_METHOD )
    
print "Building ROC Curve"
markerBuffer = MarkerBuffer.loadMarkerBuffer( markerFilename )
if markerBuffer == None:
    print "Error: Unable to load marker buffer -", markerFilename
    
# Average results

# Plot ROC Curve
# Plot accuracy
        
rocCurve = ROCCurve( crossCorrelatedSequence, markerBuffer )
 

        
        