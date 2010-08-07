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

import numpy as np
import scipy.signal
import scipy.interpolate
import cv

import yaml
import pygtk
pygtk.require('2.0')
import gtk
import gobject

import matplotlib
matplotlib.use('GTK')

from matplotlib.figure import Figure
from matplotlib.axes import Subplot
from matplotlib.backends.backend_gtkagg import FigureCanvasGTKAgg as FigureCanvas
from matplotlib.backends.backend_gtkagg import NavigationToolbar2GTKAgg as NavigationToolbar

from abroun_gtk_gui.widgets import SequenceControl
from SignalDetector import SignalDetector

def printTiming(func):
    def wrapper(*arg):
    
        t1 = time.time()
        res = func(*arg)
        t2 = time.time()
        
        print '%s took %0.3f ms' % (func.func_name, (t2-t1)*1000.0)
        return res

    return wrapper

class SignalDetector:
    
    #---------------------------------------------------------------------------
    def __init__( self ):
        pass
    
    #---------------------------------------------------------------------------
    def convertOpenCVArrayListToPythonArrayofLists( self, openCVArrayList ):
        
        result = []
        
        numOpenCVArrays = len( openCVArrayList )
        if numOpenCVArrays > 0:
            arrayHeight = openCVArrayList[ 0 ].height
            arrayWidth = openCVArrayList[ 0 ].width
            
            for rowIdx in range( arrayHeight ):
                result.append( [] )
                for colIdx in range( arrayWidth ):
                    result[ rowIdx ].append( [] )
                
            for arrayIdx, array in enumerate( openCVArrayList ):
                if array.height != arrayHeight or array.width != arrayWidth:
                    print arrayIdx, array.height, arrayHeight, array.width, arrayWidth
                    raise Exception( "All arrays should have the same width and height" )
                
                for y in range( arrayHeight ):
                    for x in range( arrayWidth ):
                        result[ y ][ x ].append( cv.Get2D( array, y, x )[ 0 ] )
        
        return result
            
    
    #---------------------------------------------------------------------------
    def crossCorrelateArray( self, inputSignal, outputSignalArray, maxLag = None ):
        '''Cross correlates the input signal with an array of output signals
        
        Repeatedly calculates the correlation coefficient between the output 
        signal array and a delayed version of the input signal
        
        Parameters
        ----------
        inputSignal : list
            The input signal samples
        outputSignalArrays : 2D array of lists
            The output signal samples. The length of each list in this array 
            must match the length of inputSignal
        maxLag : int
            The maximum number of samples to lag the input signal by. If this
            is omitted then the lag will range from 0 to the length of the 
            inputSignal list
            
        Returns
        -------
        correlationCoefficients : 2D array of lists
            The correlation coefficients calculated for each of the lagged inputs
            
        Notes
        -----
        This is all horrifically inefficient...
        '''
        result = []
        inputSignalLength = len( inputSignal )
        
        arrayHeight = len( outputSignalArray )
        if arrayHeight > 0:
            
            arrayWidth = len( outputSignalArray[ 0 ] )
            for row in outputSignalArray:
                if len( row ) != arrayWidth:
                    raise Exception( "Input array has rows of variable width" )
                
                resultRow = []
                for outputSignal in row:
                    if len( outputSignal ) != inputSignalLength:
                        raise Exception( "Length of output signal should match length of input signal" )
                    
                    resultRow.append( self.crossCorrelateComplete(
                        outputSignal, inputSignal, maxLag ) )
                
                result.append( resultRow )
        
        return result
    
    #-------------------------------------------------------------------------------
    def covariance( self, x, y ):
        numElements = len( x )
        if numElements > 0:
            eX = sum( x ) / numElements
            eY = sum( y ) / numElements
            eXY = sum( [ x[i]*y[i] for i in range( numElements ) ] ) / numElements
            return eXY - eX*eY
        else:
            return 0.0
    
    #-------------------------------------------------------------------------------
    def crossCorrelate( self, sequence, laggedSequence, lag ):
        
        acc = 0
        sequenceLength = len( sequence )
        laggedSequenceLength = len( laggedSequence )
        
        numSamples = min( sequenceLength - lag, laggedSequenceLength )
        if numSamples > 0:
            
            testSequence = sequence[lag:lag+numSamples]
            testLaggedSequence = laggedSequence[:numSamples]
            
            varX = self.covariance( testSequence, testSequence )
            varY = self.covariance( testLaggedSequence, testLaggedSequence )
            if varX <= 0.0 or varY <= 0.0:
                return 0.0
            else:
                return self.covariance( testSequence, testLaggedSequence ) / math.sqrt( varX*varY )
            
        else:
            return 0.0
            
    #-------------------------------------------------------------------------------
    # Calculates the cross correlation of a sequence for all possible lags
    def crossCorrelateComplete( self, sequence, laggedSequence, maxLag = None ):
        
        if maxLag != None:
            numCorrelations = maxLag + 1
        else:
            numCorrelations = len( sequence )
            
        return [ self.crossCorrelate( sequence, laggedSequence, lag ) \
            for lag in range( numCorrelations ) ]

            

#-------------------------------------------------------------------------------
class OpticalFlowFilter:
    
    #---------------------------------------------------------------------------
    def __init__( self, 
        opticalFlowBlockWidth, opticalFlowBlockHeight, 
        opticalFlowRangeWidth, opticalFlowRangeHeight ):
            
        self.opticalFlowBlockWidth = opticalFlowBlockWidth
        self.opticalFlowBlockHeight = opticalFlowBlockHeight
        self.opticalFlowRangeWidth = opticalFlowRangeWidth
        self.opticalFlowRangeHeight = opticalFlowRangeHeight
        
        self.lastImageGray = None

    #---------------------------------------------------------------------------
    def calcOpticalFlowWidth( self, imageWidth ):
        return (imageWidth - self.opticalFlowBlockWidth)/self.opticalFlowBlockWidth
        
    #---------------------------------------------------------------------------
    def calcOpticalFlowHeight( self, imageHeight ):
        return (imageHeight - self.opticalFlowBlockHeight)/self.opticalFlowBlockHeight
        
    #---------------------------------------------------------------------------
    def calcOpticalFlow( self, curImageGray ):
        
        if curImageGray.channels != 1 or curImageGray.depth != 8:
            raise Exception( "Only able to process gray-scale images" )
        
        if self.lastImageGray == None:
            lastImageGray = curImageGray
        else:
            lastImageGray = self.lastImageGray
        
        # Create storage for the optical flow
        storageWidth = self.calcOpticalFlowWidth( lastImageGray.width )
        storageHeight = self.calcOpticalFlowHeight( lastImageGray.height )
        
        opticalFlowArrayX = cv.CreateMat( storageHeight, storageWidth, cv.CV_32FC1 )
        opticalFlowArrayY = cv.CreateMat( storageHeight, storageWidth, cv.CV_32FC1 )
            
        cv.CalcOpticalFlowBM( lastImageGray, curImageGray, 
            ( self.opticalFlowBlockWidth, self.opticalFlowBlockHeight ),
            ( self.opticalFlowBlockWidth, self.opticalFlowBlockHeight ),
            ( self.opticalFlowRangeWidth, self.opticalFlowRangeHeight ),
            0, opticalFlowArrayX, opticalFlowArrayY )
            
        # Save the current image
        self.lastImageGray = curImageGray
        
        return ( opticalFlowArrayX, opticalFlowArrayY )

#-------------------------------------------------------------------------------
# Normalises a sequence of numbers by subtracting the mean and then dividing
# by the standard deviation. This means that the sequence is centred at 0 and
# 'mostly' within the range [-1,1]
def normaliseSequence( sequence ):
    
    result = []
    
    numElements = len( sequence )
    if numElements > 0:
        
        mean = sum( sequence )/numElements
        
        deviations = [ element - mean for element in sequence ]
        #stdDev = math.sqrt( sum( [ deviation*deviation for deviation in deviations ] ) / numElements )
        maxVal = math.sqrt( max( [ deviation*deviation for deviation in deviations ] ) )
        
        if maxVal == 0:
            result = deviations
        else:
            result = [ deviation / maxVal for deviation in deviations ]
        
    return result
    
#-------------------------------------------------------------------------------
def resampleSequence( sequenceTimes, sequenceData, newSequenceTimes ):
    
    numExistingSamples = len( sequenceData )
    numNewSamples = len( newSequenceTimes )
    if numExistingSamples > 0:
        
        curData = 0.0
        existingSampleIdx = 0
        newSampleIdx = 0
        result = []
        
        while newSampleIdx < numNewSamples:
        
            if existingSampleIdx >= numExistingSamples:
                # We've run out of data so keep on using the last sample we had
                result.append( curData )
                newSampleIdx += 1
            else:
                # Use the current data until we reach a new bit of data
                if newSequenceTimes[ newSampleIdx ] < sequenceTimes[ existingSampleIdx ]:
                    result.append( curData )
                    newSampleIdx += 1
                else:    
                    # Update the current data
                    curData = sequenceData[ existingSampleIdx ]
                    existingSampleIdx += 1
            
        
    else:
        result = [ 0.0 ] * numNewSamples
        
    return result
    

    
    #curIdx = 0
    #laggedIdx = curIdx + lag
    #while curIdx < sequenceLength and laggedIdx < laggedSequenceLength:
        #acc += sequence[ curIdx ]*laggedSequence[ laggedIdx ]
        #curIdx += 1
        #laggedIdx += 1
        
    ## If the lagged sequence ended prematurely we assume that we just added on
    ## zeroes in place of the missing samples
    #numSamplesTested = curIdx
    #return acc / numSamplesTested
    
    
#-------------------------------------------------------------------------------
def calculateEvolvingCrossCorrelation( sequence, laggedSequence, lag ):
    return [ crossCorrelate( sequence[:sliceLength], laggedSequence, lag ) \
        for sliceLength in range( 1, len( sequence ) + 1 ) ]

#-------------------------------------------------------------------------------
class MainWindow:
    
    OPTICAL_FLOW_BLOCK_WIDTH = 8
    OPTICAL_FLOW_BLOCK_HEIGHT = 8
    OPTICAL_FLOW_RANGE_WIDTH = 8    # Range to look outside of a block for motion
    OPTICAL_FLOW_RANGE_HEIGHT = 8
    
    MAX_TEST_POINT_X = (320 - OPTICAL_FLOW_BLOCK_WIDTH)/OPTICAL_FLOW_BLOCK_WIDTH - 1
    MAX_TEST_POINT_Y = (240 - OPTICAL_FLOW_BLOCK_HEIGHT)/OPTICAL_FLOW_BLOCK_HEIGHT - 1
    
    SAMPLES_PER_SECOND = 30.0
    
    GRIPPER_WAVE_FREQUENCY = 1.0    # Waves per second
    GRIPPER_NUM_WAVES = 3.0
    GRIPPER_WAVE_AMPLITUDE = math.radians( 20.0 )
 
    #---------------------------------------------------------------------------
    def __init__( self, bagFilename ):
    
        scriptPath = os.path.dirname( __file__ )
        self.cameraImagePixBuf = None
        self.bagFilename = bagFilename
        self.lastImageGray = None
        
        self.wavingGripper = False
        self.gripperWaveStartTime = None
        
        servoAngleTimes = []
        servoAngleData = []
        self.imageTimes = []
        self.cameraImages = []
        
        self.signalDetector = SignalDetector()
        self.opticalFlowFilter = OpticalFlowFilter(
            self.OPTICAL_FLOW_BLOCK_WIDTH, self.OPTICAL_FLOW_BLOCK_HEIGHT,
            self.OPTICAL_FLOW_RANGE_WIDTH, self.OPTICAL_FLOW_RANGE_HEIGHT )
            
        # Create storage for the optical flow
        numImages = 0
        opticalFlowWidth = 0
        opticalFlowHeight = 0
        for topic, msg, t in rosrecord.logplayer( bagFilename ):
            if msg._type == "sensor_msgs/Image":
                
                if numImages == 0:
                    opticalFlowWidth = self.opticalFlowFilter.calcOpticalFlowWidth( msg.width )
                    opticalFlowHeight = self.opticalFlowFilter.calcOpticalFlowHeight( msg.height )
                    
                numImages += 1
        
        opticalFlowArrayShape = ( opticalFlowHeight, opticalFlowWidth, numImages )
        self.opticalFlowArraysX = np.ndarray(shape=opticalFlowArrayShape, dtype=np.float32)
        self.opticalFlowArraysY = np.ndarray(shape=opticalFlowArrayShape, dtype=np.float32)
        imgIdx = 0
        
        
        # Extract messages from the bag
        startTime = None
        for topic, msg, t in rosrecord.logplayer( bagFilename ):
            if startTime == None:
                startTime = t
                
            bagTime = t - startTime
                
            if msg._type == "arm_driver_msgs/SetServoAngles":
                servoAngleTimes.append( bagTime.to_sec() )
                
                servoAngle = msg.servoAngles[ 0 ].angle
                servoAngleData.append( servoAngle )
                
            elif msg._type == "sensor_msgs/Image":
                
                opticalFlowArrayX, opticalFlowArrayY = self.processCameraImage( msg )

                self.imageTimes.append( bagTime.to_sec() )
                self.cameraImages.append( msg )
                
                #print dir( cv )
                #print dir( opticalFlowArrayX )
                #opNumPy = cvAdaptors.Ipl2NumPy( opticalFlowArrayX )
                cv.Set2D( opticalFlowArrayX, 0, 0, len( self.imageTimes )%8 )
                
                
                t1 = time.time()
                
                #np.put( self.opticalFlowArraysX, [ imgIdx ], np.array( opticalFlowArrayX ) )
                #np.put( self.opticalFlowArraysY, [ imgIdx ], np.array( opticalFlowArrayY ) )
                self.opticalFlowArraysX[ :, :, imgIdx ] = np.array( opticalFlowArrayX )
                self.opticalFlowArraysY[ :, :, imgIdx ] = np.array( opticalFlowArrayY )
                
                imgIdx += 1
                t2 = time.time()
                
                
                #self.opFlowXNP.append( opNumPy )
                #print 'Conversion took %0.3f ms' % ((t2-t1)*1000.0)
                

             

        
        print self.opticalFlowArraysX[ 0, 0 ]
        print self.opticalFlowArraysX[ :, :, 187 ]

         
        #servoAngleVelocity = [ servoAngleData[ i ] - servoAngleData[ i - 1 ] for i in range( 1, len( servoAngleData ) ) ]
        #servoAngleData = [ 0 ] + servoAngleVelocity
        #servoAngleData[ -1 ] = 0

        # Construct regular sampled data from the input servo data
        servoAngleData = normaliseSequence( servoAngleData )
        dataDuration = math.ceil( max( servoAngleTimes[ -1 ], self.imageTimes[ -1 ] ) )
        
        self.regularSampleTimes = [ i*1.0/self.SAMPLES_PER_SECOND for i in range( int( dataDuration*self.SAMPLES_PER_SECOND ) ) ]
        
        servoAngleDataSpline = scipy.signal.cspline1d( np.array( servoAngleData ) )
        self.regularServoAngleData = scipy.signal.cspline1d_eval( 
            servoAngleDataSpline, np.array( self.regularSampleTimes ),
            dx=self.imageTimes[ 1 ]-self.imageTimes[ 0 ], x0=0.0 )
            
        # spline parameters
        s=3.0 # smoothness parameter
        k=5 # spline order
        nest=-1 # estimate of number of knots needed (-1 = maximal)

        # Pad the servo readings with zeros at the start and end
        preTimes = np.arange( 0.0, servoAngleTimes[ 0 ], 0.1 )
        preReadings = np.zeros( len( preTimes ) )
        postTimes = np.arange( servoAngleTimes[ -1 ] + 0.1, dataDuration, 0.1 )
        postReadings = np.zeros( len( postTimes ) )
        
        npAngleTimes = np.concatenate( [ preTimes, np.array( servoAngleTimes ), postTimes ] )
        npAngleData = np.concatenate( [ preReadings, np.array( servoAngleData ), postReadings ] )
        
        self.A = servoAngleTimes
        self.B = servoAngleData
            
        tck = scipy.interpolate.splrep( npAngleTimes, npAngleData )

        # evaluate spline, including interpolated points
        self.regularServoAngleData = scipy.interpolate.splev(
            np.array( self.regularSampleTimes ),tck)

        
        #self.regularServoAngleData = resampleSequence( servoAngleTimes, servoAngleData, self.regularSampleTimes )
        
        op = []
        for a in [ self.opticalFlowArraysX, self.opticalFlowArraysY ]:
            b = []
            for rowIdx in range( a.shape[ 0 ] ):
                row = []
                for colIdx in range( a.shape[ 1 ] ):
                    newArray = resampleSequence( self.imageTimes, a[ rowIdx, colIdx ].tolist(), self.regularSampleTimes )
                    #print newArray
                    row.append( [newArray] )
                b.append( row )
            op.append( b )
        self.regularOpticalFlowArrayX = b[ 0 ]
        self.regularOpticalFlowArrayY = b[ 1 ]
                    
        t1 = time.time()
        self.correlationsX = self.signalDetector.crossCorrelateArray( 
            self.regularServoAngleData, self.regularOpticalFlowArrayX,
            int ( 1.0*self.SAMPLES_PER_SECOND ) )
        self.correlationsY = self.signalDetector.crossCorrelateArray( 
            self.regularServoAngleData, self.regularOpticalFlowArrayY,
            int ( 1.0*self.SAMPLES_PER_SECOND ) )
        t2 = time.time()
        print 'Correlation took %0.3f ms' % ((t2-t1)*1000.0)
        
        # Create the matplotlib graph
        self.figure = Figure( figsize=(8,6), dpi=72 )
        self.axisX = self.figure.add_subplot( 211 )
        self.axisY = self.figure.add_subplot( 212 )
        
        self.canvas = None  # Wait for GUI to be created before creating canvas
        self.navToolbar = None
 
        # Setup the GUI        
        builder = gtk.Builder()
        builder.add_from_file( scriptPath + "/GUI/OpticalFlowExplorer.glade" )
        
        self.dwgCameraImage = builder.get_object( "dwgCameraImage" )
        self.window = builder.get_object( "winMain" )
        self.vboxMain = builder.get_object( "vboxMain" )
        self.hboxWorkArea = builder.get_object( "hboxWorkArea" )
        self.adjTestPointX = builder.get_object( "adjTestPointX" )
        self.adjTestPointY = builder.get_object( "adjTestPointY" )
        self.adjTestPointX.set_upper( self.MAX_TEST_POINT_X )
        self.adjTestPointY.set_upper( self.MAX_TEST_POINT_Y )
        self.sequenceControls = builder.get_object( "sequenceControls" )
        self.sequenceControls.setNumFrames( len( self.cameraImages ) )
        self.sequenceControls.setOnFrameIdxChangedCallback( self.onSequenceControlsFrameIdxChanged )
        self.setFrameIdx( 0 )
        self.processOpticalFlowData()
        
        builder.connect_signals( self )
               
        updateLoop = self.update()
        gobject.idle_add( updateLoop.next )
        
        self.window.show()
        self.window.maximize()
        
    #---------------------------------------------------------------------------
    def onWinMainDestroy( self, widget, data = None ):  
        gtk.main_quit()
        
    #---------------------------------------------------------------------------   
    def main( self ):
        # All PyGTK applications must have a gtk.main(). Control ends here
        # and waits for an event to occur (like a key press or mouse event).
        gtk.main()
        
    #---------------------------------------------------------------------------
    def processOpticalFlowData( self ):

        testX = int( self.adjTestPointX.get_value() )
        testY = int( self.adjTestPointY.get_value() )
        
        # Extract optical flow for the current test point
        #opticalFlowDataX = []
        #opticalFlowDataY = []
        
        #for frameIdx in range( len( self.imageTimes ) ):
            
            #opticalFlowX = self.opticalFlowArraysX[ frameIdx ]
            #opticalFlowY = self.opticalFlowArraysY[ frameIdx ]
            
            #if opticalFlowX == None or opticalFlowY == None:
                #flowX = 0.0
                #flowY = 0.0
            #else:
                #flowX = cv.Get2D( self.opticalFlowArraysX[ frameIdx ], 
                    #testY, testX )[ 0 ] / self.OPTICAL_FLOW_BLOCK_WIDTH
                #flowY = cv.Get2D( self.opticalFlowArraysY[ frameIdx ], 
                    #testY, testX )[ 0 ] / self.OPTICAL_FLOW_BLOCK_HEIGHT
                    
                ##if testX > 12:
                ##    flowX /= 8.0
                #flowY = -flowY
            
            #opticalFlowDataX.append( flowX )
            #opticalFlowDataY.append( flowY )
        
        # Normalise and resample the optical flow data
        #opticalFlowDataX = normaliseSequence( opticalFlowDataX )
        #opticalFlowDataY = normaliseSequence( opticalFlowDataY )
        #print opticalFlowDataY
        
        #regularOpticalFlowDataX = resampleSequence( self.imageTimes, opticalFlowDataX, self.regularSampleTimes )
        #regularOpticalFlowDataY = resampleSequence( self.imageTimes, opticalFlowDataY, self.regularSampleTimes )
        
        #regularOpticalFlowDataX = [ self.regularServoAngleData[ i ]*(1.0+random.gauss( 0, 0.0 )) \
        #    for i in range( len( self.regularServoAngleData ) ) ]
        #regularOpticalFlowDataX = normaliseSequence( regularOpticalFlowDataX )
        
        #for i in range( 58 ):
        #    regularOpticalFlowDataY[ i ] = random.gauss( 0, 0.4 )
        #    regularOpticalFlowDataY[ 240 + i ] = random.gauss( 0, 0.4 )
        
        # Calculate correlation
        #crossCorrelationOpticalFlowX = crossCorrelateComplete( 
        #    regularOpticalFlowDataX, self.regularServoAngleData )
        #evolvingCCOpticalFlowX = calculateEvolvingCrossCorrelation(
        #    regularOpticalFlowDataX, self.regularServoAngleData, int( 0.45*self.SAMPLES_PER_SECOND ) )
        #crossCorrelationOpticalFlowY = crossCorrelateComplete( 
        #    regularOpticalFlowDataY, self.regularServoAngleData )
        #evolvingCCOpticalFlowY = calculateEvolvingCrossCorrelation(
        #    regularOpticalFlowDataY, self.regularServoAngleData, int( 0.45*self.SAMPLES_PER_SECOND ) )
        
        regularOpticalFlowDataX = self.regularOpticalFlowArrayX[ testY ][ testX ]
        regularOpticalFlowDataX = normaliseSequence( regularOpticalFlowDataX )
        regularOpticalFlowDataY = self.regularOpticalFlowArrayY[ testY ][ testX ]
        regularOpticalFlowDataY = normaliseSequence( regularOpticalFlowDataY )
        
        crossCorrelationOpticalFlowX = self.correlationsX[ testY ][ testX ]
        crossCorrelationOpticalFlowY = self.correlationsY[ testY ][ testX ]
        
        # Plot graphs
        self.axisX.clear()
        self.axisX.plot( self.regularSampleTimes, self.regularServoAngleData )
        self.axisX.plot( self.A, self.B )
        self.axisX.plot( self.regularSampleTimes, regularOpticalFlowDataX )
        self.axisX.plot( self.regularSampleTimes[:len(crossCorrelationOpticalFlowX)], crossCorrelationOpticalFlowX )
        #self.axisX.plot( self.regularSampleTimes, evolvingCCOpticalFlowX )
        
        self.axisY.clear()
        self.axisY.plot( self.regularSampleTimes, self.regularServoAngleData )
        self.axisY.plot( self.regularSampleTimes, regularOpticalFlowDataY )
        self.axisY.plot( self.regularSampleTimes[:len(crossCorrelationOpticalFlowY)], crossCorrelationOpticalFlowY )
        #self.axisY.plot( self.regularSampleTimes, evolvingCCOpticalFlowY )
        
        self.refreshGraphDisplay()
    
    #---------------------------------------------------------------------------
    def refreshGraphDisplay( self ):
        
        if self.canvas != None:   
            self.hboxWorkArea.remove( self.canvas )
            self.canvas.destroy()  
            self.canvas = None   
        if self.navToolbar != None:
            self.vboxMain.remove( self.navToolbar )
            self.navToolbar.destroy()  
            self.navToolbar = None   
        
        self.canvas = FigureCanvas( self.figure ) # a gtk.DrawingArea
        self.canvas.show()
        self.hboxWorkArea.pack_start( self.canvas, True, True )
        self.hboxWorkArea.show()
        
        # Navigation toolbar
        self.navToolbar = NavigationToolbar( self.canvas, self.window )
        self.navToolbar.lastDir = '/var/tmp/'
        self.vboxMain.pack_start( self.navToolbar, expand=False, fill=False )
        self.navToolbar.show()
        self.vboxMain.show()
        
    #---------------------------------------------------------------------------
    def processCameraImage( self, rosImage ):
        
        opticalFlowArrayX = None
        opticalFlowArrayY = None
        
        if rosImage.encoding == "rgb8" or rosImage.encoding == "bgr8":
            
            # Create an OpenCV image to process the data
            curImageRGB = cv.CreateImageHeader( ( rosImage.width, rosImage.height ), cv.IPL_DEPTH_8U, 3 )
            cv.SetData( curImageRGB, rosImage.data, rosImage.step )
            curImageGray = cv.CreateImage( ( rosImage.width, rosImage.height ), cv.IPL_DEPTH_8U, 1 )
            cv.CvtColor( curImageRGB, curImageGray, cv.CV_RGB2GRAY )
            
            # Look for optical flow between this image and the last one
            opticalFlowArrayX, opticalFlowArrayY = self.opticalFlowFilter.calcOpticalFlow( curImageGray )
            
        else:
            rospy.logerr( "Unhandled image encoding - " + rosImage.encoding )
            
        return ( opticalFlowArrayX, opticalFlowArrayY )
    
    #---------------------------------------------------------------------------
    def setFrameIdx( self, frameIdx ):
        
        self.frameIdx = frameIdx
        
        # Display the frame
        image = self.cameraImages[ frameIdx ]
        self.cameraImagePixBuf = gtk.gdk.pixbuf_new_from_data( 
            image.data, 
            gtk.gdk.COLORSPACE_RGB,
            False,
            8,
            image.width,
            image.height,
            image.step )

        # Resize the drawing area if necessary
        if self.dwgCameraImage.get_size_request() != ( image.width, image.height ):
            self.dwgCameraImage.set_size_request( image.width, image.height )

        self.dwgCameraImage.queue_draw()
    
    #---------------------------------------------------------------------------
    def onTestPointAdjustmentValueChanged( self, widget ):
        self.processOpticalFlowData()
        self.dwgCameraImage.queue_draw()
    
    #---------------------------------------------------------------------------
    def onSequenceControlsFrameIdxChanged( self, widget ):
        self.setFrameIdx( widget.frameIdx )
    
    #---------------------------------------------------------------------------
    def onDwgCameraImageButtonPressEvent( self, widget, data ):
        
        if self.cameraImagePixBuf != None:
            
            imgRect = self.getImageRectangleInWidget( widget,
                self.cameraImagePixBuf.get_width(), self.cameraImagePixBuf.get_height() )
        
            self.adjTestPointX.set_value( int( ( data.x - imgRect.x )/self.OPTICAL_FLOW_BLOCK_WIDTH ) )
            self.adjTestPointY.set_value( int( ( data.y - imgRect.y )/self.OPTICAL_FLOW_BLOCK_HEIGHT ) )            
        
    #---------------------------------------------------------------------------
    def onDwgCameraImageExposeEvent( self, widget, data = None ):
        
        if self.cameraImagePixBuf != None:
            
            imgRect = self.getImageRectangleInWidget( widget,
                self.cameraImagePixBuf.get_width(), self.cameraImagePixBuf.get_height() )
                
            imgOffsetX = imgRect.x
            imgOffsetY = imgRect.y
                
            # Get the total area that needs to be redrawn
            imgRect = imgRect.intersect( data.area )
        
            srcX = imgRect.x - imgOffsetX
            srcY = imgRect.y - imgOffsetY
           
            widget.window.draw_pixbuf( widget.get_style().fg_gc[ gtk.STATE_NORMAL ],
                self.cameraImagePixBuf, srcX, srcY, 
                imgRect.x, imgRect.y, imgRect.width, imgRect.height )
                
            # Draw the optical flow if it's available
            opticalFlowX = self.opticalFlowArraysX[ :, :, self.frameIdx ]
            opticalFlowY = self.opticalFlowArraysY[ :, :, self.frameIdx ]
            if opticalFlowX != None and opticalFlowY != None:
                
                testX = int( self.adjTestPointX.get_value() )
                testY = int( self.adjTestPointY.get_value() )
            
                graphicsContext = widget.window.new_gc()
                graphicsContext.set_rgb_fg_color( gtk.gdk.Color( 0, 65535, 0 ) )
                
                blockCentreY = imgRect.y + self.OPTICAL_FLOW_BLOCK_HEIGHT / 2
                for y in range( opticalFlowX.shape[ 0 ] ):
                
                    blockCentreX = imgRect.x + self.OPTICAL_FLOW_BLOCK_WIDTH / 2
                    for x in range( opticalFlowX.shape[ 1 ] ):
                        
                        if testX == x and testY == y:
                            # Highlight the current test point
                            radius = 2
                            arcX = int( blockCentreX - radius )
                            arcY = int( blockCentreY - radius )
                            arcWidth = arcHeight = int( radius * 2 )
            
                            drawFilledArc = False
                            graphicsContext.set_rgb_fg_color( gtk.gdk.Color( 65535, 65535, 65535 ) )

                            widget.window.draw_arc( graphicsContext, 
                                drawFilledArc, arcX, arcY, arcWidth, arcHeight, 0, 360 * 64 )
                
                        
                        endX = blockCentreX + opticalFlowX[ y, x ]
                        endY = blockCentreY + opticalFlowY[ y, x ]
                        
                        if endY < blockCentreY:
                            # Up is red
                            graphicsContext.set_rgb_fg_color( gtk.gdk.Color( 65535, 0, 0 ) )
                        elif endY > blockCentreY:
                            # Down is blue
                            graphicsContext.set_rgb_fg_color( gtk.gdk.Color( 0, 0, 65535 ) )
                        else:
                            # Static is green
                            graphicsContext.set_rgb_fg_color( gtk.gdk.Color( 0, 65535, 0 ) )
                            
                        
                        widget.window.draw_line( graphicsContext, 
                            int( blockCentreX ), int( blockCentreY ),
                            int( endX ), int( endY ) )
                        
                        blockCentreX += self.OPTICAL_FLOW_BLOCK_WIDTH
                        
                    blockCentreY += self.OPTICAL_FLOW_BLOCK_HEIGHT            

    #---------------------------------------------------------------------------
    def getImageRectangleInWidget( self, widget, imageWidth, imageHeight ):
        
        # Centre the image inside the widget
        widgetX, widgetY, widgetWidth, widgetHeight = widget.get_allocation()
        
        imgRect = gtk.gdk.Rectangle( 0, 0, widgetWidth, widgetHeight )
        
        if widgetWidth > imageWidth:
            imgRect.x = (widgetWidth - imageWidth) / 2
            imgRect.width = imageWidth
            
        if widgetHeight > imageHeight:
            imgRect.y = (widgetHeight - imageHeight) / 2
            imgRect.height = imageHeight
        
        return imgRect

    #---------------------------------------------------------------------------
    def update( self ):

        UPDATE_FREQUENCY = 30.0    # Updates in Hz

        lastTime = time.clock()

        while 1:
            
            curTime = time.clock()
            
            if curTime - lastTime >= 1.0 / UPDATE_FREQUENCY:
            
                ## Update the wave if active
                #if self.wavingGripper:
                    
                    #waveTime = curTime - self.gripperWaveStartTime
                    #totalWaveTime = self.GRIPPER_NUM_WAVES / self.GRIPPER_WAVE_FREQUENCY
                    #waveFinished = False
                    
                    #print waveTime, totalWaveTime
                    #if waveTime >= totalWaveTime:
                        
                        #waveTime = totalWaveTime
                        #waveFinished = True
                        
                    ## Work out the current displacement from the initial position
                    #displacement = self.GRIPPER_WAVE_AMPLITUDE \
                        #* math.sin( waveTime*self.GRIPPER_WAVE_FREQUENCY*2.0*math.pi )
                    
                    
                    #servoAnglesDict = { "wrist_rotate" : self.wristAngle + displacement }
                    
                    #self.roboticArm.setJointAngles( servoAnglesDict )
                    #if waveFinished:
                        #self.wavingGripper = False

                # Save the time
                lastTime = curTime
                
            yield True
            
        yield False
        
#-------------------------------------------------------------------------------
if __name__ == "__main__":

    if len( sys.argv ) >= 2:
        
        bagFilename = sys.argv[ 1 ]

        mainWindow = MainWindow( bagFilename )
        mainWindow.main()
        
    else:
        print "Please provide a bag file"

