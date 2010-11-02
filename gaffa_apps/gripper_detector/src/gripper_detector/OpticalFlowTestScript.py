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
import gc

import numpy as np
import cv

import matplotlib
from matplotlib.figure import Figure
from matplotlib.axes import Subplot
from matplotlib.backends.backend_gtkagg import FigureCanvasGTKAgg as FigureCanvas

import OpticalFlow.Utils as Utils
from OpticalFlow.InputSequence import InputSequence, Distractor
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
ADD_DISTRACTORS = False

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
def evaluateClassifier( markerFilename, bagFilenames, 
                        rocGraphFilename = None, accuracyGraphFilename = None,
                        gaussianStdDev = 0.0 ):
    '''Takes a list of bag files in as data for the classifier and evaluates
       the classifer against a ground truth marker file. Returns the average AUC
       and optimal threshold for the classifier.'''

    # Load in marker buffer for evaluating classifier
    markerBuffer = MarkerBuffer.loadMarkerBuffer( markerFilename )
    if markerBuffer == None:
        print "Error: Unable to load marker buffer -", markerFilename

    # Process each bag file in turn
    dataList = []
    for bagFilename in bagFilenames:
        print "Reading in bag file -", bagFilename
        inputSequence = InputSequence( bagFilename )
        
        if ADD_DISTRACTORS:
            distractors = [
                Distractor( radius=24, startPos=( 25, 35 ), endPos=( 100, 100 ), frequency=2.0 ),
                Distractor( radius=24, startPos=( 200, 200 ), endPos=( 150, 50 ), frequency=0.25 ),
                Distractor( radius=24, startPos=( 188, 130 ), endPos=( 168, 258 ), frequency=0.6 ),
                Distractor( radius=24, startPos=( 63, 94 ), endPos=( 170, 81 ), frequency=1.5 ),
                Distractor( radius=24, startPos=( 40, 287 ), endPos=( 50, 197 ), frequency=3.0 ) ]
            inputSequence.addDistractorObjects( distractors )
        
        inputSequence.calculateOpticalFlow(
            OPTICAL_FLOW_BLOCK_WIDTH, OPTICAL_FLOW_BLOCK_HEIGHT,
            OPTICAL_FLOW_RANGE_WIDTH, OPTICAL_FLOW_RANGE_HEIGHT,
            OPTICAL_FLOW_METHOD )

        #print "Resampling data"
        regularisedInputSequence = RegularisedInputSequence( 
            inputSequence, SAMPLES_PER_SECOND )
        if gaussianStdDev > 0.0:
            regularisedInputSequence.smoothOpticalFlow( gaussianStdDev )

        #print "Performing cross correlation"
        crossCorrelatedSequence = CrossCorrelatedSequence( 
            regularisedInputSequence, 
            MAX_CORRELATION_LAG, COMBINATION_METHOD )
            
        #print "Building ROC Curve"
        rocCurve = GripperDetectorROCCurve( crossCorrelatedSequence, markerBuffer )
        
        print "__AUC =", rocCurve.areaUnderCurve
        
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
    axisROC.set_xlabel( 'False Positive Rate' )
    axisROC.set_ylabel( 'True Positive Rate' )


    # Add error bars
    if varianceROCCurve != None:
        
        diffBetweenErrorBars = 0.025 #1.0/(NUM_ERROR_BARS)
        lastFPRValue = averageROCCurve.falsePositiveRates[ 0 ]
        
        errorFPR = []
        errorTPR = []
        errorErrFPR = []
        errorErrTPR = []
        for i in range( len( averageROCCurve.falsePositiveRates ) ):
            
            curFPRValue = averageROCCurve.falsePositiveRates[ i ]
            if abs( curFPRValue - lastFPRValue ) >= diffBetweenErrorBars:
                lastFPRValue = curFPRValue
            
                errorFPR.append( averageROCCurve.falsePositiveRates[ i ] )
                errorTPR.append( averageROCCurve.truePositiveRates[ i ] )
                errorErrFPR.append( math.sqrt( varianceROCCurve.falsePositiveRates[ i ] )*2.571 )
                errorErrTPR.append( math.sqrt( varianceROCCurve.truePositiveRates[ i ] )*2.571 )
                
        axisROC.errorbar( errorFPR, errorTPR, 
            yerr=errorErrTPR, linestyle='None' )
            #xerr=errorErrFPR, yerr=errorErrTPR, linestyle='None' )
            
        #print errorFPR
        #print errorTPR
        #print lastFPRValue, averageROCCurve.falsePositiveRates[ -1 ]

    axisROC.set_xlim( 0.0, 1.0 )
    axisROC.set_ylim( 0.0, 1.0 )

    # Plot accuracy
    figureAccuracy = Figure( figsize=(8,6), dpi=72 )
    canvasAccuracy = FigureCanvas( figureAccuracy )
    axisAccuracy = figureAccuracy.add_subplot( 111 )

    thresholds = averageROCCurve.calculateThresholds()
    axisAccuracy.plot( thresholds, averageROCCurve.accuracy )

    #for data in dataList:
    #    axisAccuracy.plot( thresholds, data.rocCurve.accuracy )

    # Add error bars
    if varianceROCCurve != None:
        
        diffBetweenErrorBars = 1.0/(NUM_ERROR_BARS)
        lastThresholdValue = thresholds[ 0 ]
        
        errorThreshold = []
        errorAccuracy = []
        errorErrAccuracy = []
        for i in range( len( thresholds ) ):
            
            curThresholdValue = thresholds[ i ]
            if abs( curThresholdValue - lastThresholdValue ) >= diffBetweenErrorBars:
                lastThresholdValue = curThresholdValue
            
                errorThreshold.append( curThresholdValue )
                errorAccuracy.append( averageROCCurve.accuracy[ i ] )
                errorErrAccuracy.append( math.sqrt( varianceROCCurve.accuracy[ i ] )*2.571 )
            
        axisAccuracy.errorbar( errorThreshold, errorAccuracy, yerr=errorErrAccuracy, linestyle='None' )

    # Mark the threshold with the maximum accuracy
    maxAccuracyThreshold = thresholds[ np.argsort( averageROCCurve.accuracy )[ -1 ] ]
    axisAccuracy.axvline( x=float( maxAccuracyThreshold ), color='red' )

    axisAccuracy.set_xlabel( 'Threshold' )
    axisAccuracy.set_ylabel( 'Accuracy' )

    # Save the graphs
    if rocGraphFilename != None:
        figureROC.savefig( rocGraphFilename )
    if accuracyGraphFilename != None:
        figureAccuracy.savefig( accuracyGraphFilename )
        
    return averageROCCurve.areaUnderCurve, maxAccuracyThreshold

#-------------------------------------------------------------------------------
usage = "usage: %prog [options] maskFile bagFiles"
parser = OptionParser( usage=usage )
parser.add_option( "-p", "--prefix", dest="outputPrefix", default="", 
    help="The prefix to prepend to the output filenames" )
parser.add_option( "-e", "--evalVar", 
    action="store_true", dest="evaluateVariableNumWaves", default=False,
    help="Evaluate the effect on AUC of a variable number of waves")


#parser.add_option( "-o", "--output", dest="outputFilename", default="out.bag",
#                  help="Write output to file", metavar="FILE")

(options, args) = parser.parse_args()

if options.evaluateVariableNumWaves:
    if len( args ) < 1:
        print "Error: Not enough arguments supplied"
        parser.print_help()
        sys.exit( -1 )
    
    markerFilename = args[ 0 ]
    
    MAX_NUM_WAVES = 9
    waveNumList = range( 1, MAX_NUM_WAVES + 1 )
    
    #smoothingStdDevs = [ 1.0, 1.5, 2.0, 3.0, 4.0, 5.0 ]
    smoothingStdDevs = [ 0.0 ]
    
    for gaussianStdDev in smoothingStdDevs:
        results = []
        
        for waveNum in waveNumList:
            bagFilenames = [ "VarWave_{0:02}_{1:02}_Waves.bag".format( testIdx, waveNum ) for testIdx in range( 1, 6 ) ]
            
            averageAreaUnderCurve, maxAccuracyThreshold = \
                evaluateClassifier( markerFilename, bagFilenames, 
                    gaussianStdDev=gaussianStdDev )
            
            results.append( ( waveNum, averageAreaUnderCurve, maxAccuracyThreshold ) )
            gc.collect()
        
        print "GaussianStdDev =", gaussianStdDev
        print results
else:
    if len( args ) < 2:
        print "Error: Not enough arguments supplied"
        parser.print_help()
        sys.exit( -1 )

    markerFilename = args[ 0 ]
    bagFilenames = args[ 1: ]

    averageAreaUnderCurve, maxAccuracyThreshold = evaluateClassifier( 
        markerFilename, bagFilenames,
        rocGraphFilename = options.outputPrefix + "ROC.eps",
        accuracyGraphFilename = options.outputPrefix + "Accuracy.eps" )

    # Display AUC
    print "AUC =", averageAreaUnderCurve
    print "Max accuracy at", maxAccuracyThreshold


 

        
        