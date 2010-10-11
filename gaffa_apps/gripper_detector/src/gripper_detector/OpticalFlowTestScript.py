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
from matplotlib.backends.backend_gtkagg import FigureCanvasGTKAgg as FigureCanvas

import OpticalFlow.Utils as Utils
from OpticalFlow.InputSequence import InputSequence
from OpticalFlow.RegularisedInputSequence import RegularisedInputSequence
from OpticalFlow.CrossCorrelatedSequence import CrossCorrelatedSequence
from OpticalFlow.ROCCurve import ROCCurve, GripperDetectorROCCurve
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
NUM_ERROR_BARS = 10

#-------------------------------------------------------------------------------
class WorkingData:
    
    #---------------------------------------------------------------------------
    def __init__( self, bagFilename, inputSequence, 
        regularisedInputSequence, crossCorrelatedSequence, rocCurve ):
            
        self.bagFilename = bagFilename
        self.inputSequence = inputSequence
        self.regularisedInputSequence = regularisedInputSequence
        self.crossCorrelatedSequence = crossCorrelatedSequence
        self.rocCurve = rocCurve

#-------------------------------------------------------------------------------
usage = "usage: %prog [options] maskFile bagFiles"
parser = OptionParser( usage=usage )
parser.add_option( "-p", "--prefix", dest="outputPrefix", default="", 
    help="The prefix to prepend to the output filenames" )
#parser.add_option( "-o", "--output", dest="outputFilename", default="out.bag",
#                  help="Write output to file", metavar="FILE")

(options, args) = parser.parse_args()

if len( args ) < 2:
    print "Error: Not enough arguments supplied"
    parser.print_help()
    sys.exit( -1 )
    
markerFilename = args[ 0 ]
bagFilenames = args[ 1: ]

# Load in marker buffer for evaluating classifier
markerBuffer = MarkerBuffer.loadMarkerBuffer( markerFilename )
if markerBuffer == None:
    print "Error: Unable to load marker buffer -", markerFilename

# Process each bag file in turn
dataList = []
for bagFilename in bagFilenames:
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
    rocCurve = GripperDetectorROCCurve( crossCorrelatedSequence, markerBuffer )
    
    dataList.append( WorkingData( bagFilename, inputSequence, 
        regularisedInputSequence, crossCorrelatedSequence, rocCurve ) )
        
# Average results
if len( dataList ) > 1:
    
    curveList = [ data.rocCurve for data in dataList ]
    averageROCCurve, varianceROCCurve = ROCCurve.averageROCCurves( curveList )

else:
    
    averageROCCurve = dataList[ 0 ].rocCurve
    varianceROCCurve = None
   
# Plot ROC Curve 
figureROC = Figure( figsize=(8,6), dpi=72 )
canvasROC = FigureCanvas( figureROC )
axisROC = figureROC.add_subplot( 111 )

axisROC.plot( averageROCCurve.falsePositiveRates, averageROCCurve.truePositiveRates )

# Add error bars
if varianceROCCurve != None:
    
    #diffBetweenErrorBars = 0.05 #1.0/(NUM_ERROR_BARS)
    #lastFPRValue = averageROCCurve.falsePositiveRates[ 0 ]
    
    #errorFPR = []
    #errorTPR = []
    #errorErrFPR = []
    #errorErrTPR = []
    #for i in range( len( averageROCCurve.falsePositiveRates ) ):
        
        #curFPRValue = averageROCCurve.falsePositiveRates[ i ]
        #if abs( curFPRValue - lastFPRValue ) >= diffBetweenErrorBars:
            #lastFPRValue = curFPRValue
        
            #errorFPR.append( averageROCCurve.falsePositiveRates[ i ] )
            #errorTPR.append( averageROCCurve.truePositiveRates[ i ] )
            #errorErrFPR.append( math.sqrt( varianceROCCurve.falsePositiveRates[ i ] )*2.571 )
            #errorErrTPR.append( math.sqrt( varianceROCCurve.truePositiveRates[ i ] )*2.571 )
            
    #axisROC.errorbar( errorFPR, errorTPR, 
        #xerr=errorErrFPR, yerr=errorErrTPR, linestyle='None' )
        
    errorList = [ math.sqrt( v )*2.571 for v in varianceROCCurve.truePositiveRates ]
        
    axisROC.errorbar( averageROCCurve.falsePositiveRates, averageROCCurve.truePositiveRates, 
        yerr=errorList, linestyle='None' )
        
    #print errorFPR
    #print errorTPR
    #print lastFPRValue, averageROCCurve.falsePositiveRates[ -1 ]

axisROC.set_xlim( 0.0, 1.0 )
axisROC.set_ylim( 0.0, 1.0 )

for i in range( len( averageROCCurve.falsePositiveRates ) ):
    print averageROCCurve.falsePositiveRates[ i ], averageROCCurve.truePositiveRates[ i ], averageROCCurve.scores[ i ]

# Plot accuracy
#figureAccuracy = Figure( figsize=(8,6), dpi=72 )
#canvasAccuracy = FigureCanvas( figureAccuracy )
#axisAccuracy = figureAccuracy.add_subplot( 111 )

#thresholds = averageROCCurve.calculateThresholds()
#axisAccuracy.plot( thresholds, averageROCCurve.accuracy )

##for data in dataList:
##    axisAccuracy.plot( thresholds, data.rocCurve.accuracy )

## Add error bars
#if varianceROCCurve != None:
    
    #diffBetweenErrorBars = 1.0/(NUM_ERROR_BARS)
    #lastThresholdValue = thresholds[ 0 ]
    
    #errorThreshold = []
    #errorAccuracy = []
    #errorErrAccuracy = []
    #for i in range( len( thresholds ) ):
        
        #curThresholdValue = thresholds[ i ]
        #if abs( curThresholdValue - lastThresholdValue ) >= diffBetweenErrorBars:
            #lastThresholdValue = curThresholdValue
        
            #errorThreshold.append( curThresholdValue )
            #errorAccuracy.append( averageROCCurve.accuracy[ i ] )
            #errorErrAccuracy.append( math.sqrt( varianceROCCurve.accuracy[ i ] )*2.571 )
          
    #axisAccuracy.errorbar( errorThreshold, errorAccuracy, yerr=errorErrAccuracy, linestyle='None' )

# Save the graphs
figureROC.savefig( options.outputPrefix + "ROC.png" )
#figureAccuracy.savefig( options.outputPrefix + "Accuracy.png" )
        

 

        
        