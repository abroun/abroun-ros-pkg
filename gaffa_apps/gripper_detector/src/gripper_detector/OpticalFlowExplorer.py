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
from OpticalFlowFilter import OpticalFlowFilter

def printTiming(func):
    def wrapper(*arg):
    
        t1 = time.time()
        res = func(*arg)
        t2 = time.time()
        
        print '%s took %0.3f ms' % (func.func_name, (t2-t1)*1000.0)
        return res

    return wrapper
        
#-------------------------------------------------------------------------------
# Calculates the cross correlation of a sequence for all possible lags
def crossCorrelateComplete( sequence, laggedSequence, maxLag = None ):

    if maxLag != None:
        numCorrelations = maxLag + 1
    else:
        numCorrelations = len( sequence )

    # x
    x = sequence
    sum_x = np.add.accumulate( x[ ::-1 ] )[ ::-1 ] 
    sum_x2 = np.add.accumulate( np.square( x )[ ::-1 ] )[ ::-1 ] 
    len_x = np.arange( len( x ), 0, -1 )

    #var_x = np.divide( np.subtract( np.multiply( sum_x2, len_x ), np.square( sum_x ) ), np.square( len_x ) )
    var_x = np.subtract( np.multiply( sum_x2, len_x ), np.square( sum_x ) )

    # y
    y = laggedSequence

    sum_y = np.add.accumulate( y )
    sum_y2 = np.add.accumulate( np.square( y ) )
    len_y = np.arange( 1, len( y ) + 1, 1 )

    #var_y = np.divide( np.subtract( np.multiply( sum_y2, len_y ), np.square( sum_y ) ), np.square( len_y ) )
    var_y = np.subtract( np.multiply( sum_y2, len_y ), np.square( sum_y ) )

    # xy
    sum_xy = np.correlate( x, y, mode="full" )[ len( x ) - 1: ]
    #cov_xy = np.divide( np.subtract( np.multiply( sum_xy, len_x ), np.multiply( sum_x, sum_y[::-1] ) ), np.square( len_x ) )
    cov_xy = np.subtract( np.multiply( sum_xy[ : numCorrelations], len_x[ : numCorrelations] ), np.multiply( sum_x[ : maxLag+1], sum_y[::-1][ : numCorrelations] ) )

    corrCoeff = np.divide( cov_xy, np.sqrt( np.multiply( var_x[ : numCorrelations], var_y[ ::-1 ][ : numCorrelations] ) ) )
    corrCoeff[ np.logical_or( np.isnan( corrCoeff ), np.isinf( corrCoeff ) ) ] = 0.0
    
    return corrCoeff
 
#-------------------------------------------------------------------------------   
def isInputSignalPresent( maxCorrCoeffX, maxCorrCoeffY ):
    
    return ( maxCorrCoeffX > 0.75 or maxCorrCoeffY > 0.75 \
        or maxCorrCoeffX + maxCorrCoeffY > 1.2 )
            
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
def resampleSequence( sequenceData, sequenceTimes, newSequenceTimes ):
    
    # Construct a spline to represent the sequence
    tck = scipy.interpolate.splrep( sequenceTimes, sequenceData )

    # Evaluate spline at new sample points
    return scipy.interpolate.splev( newSequenceTimes,tck )
    
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
        self.cameraImages = []
        
        self.opticalFlowFilter = OpticalFlowFilter(
            self.OPTICAL_FLOW_BLOCK_WIDTH, self.OPTICAL_FLOW_BLOCK_HEIGHT,
            self.OPTICAL_FLOW_RANGE_WIDTH, self.OPTICAL_FLOW_RANGE_HEIGHT )
            
        # Create storage for the optical flow
        numImages = 0
        lastCameraTime = None
        opticalFlowWidth = 0
        opticalFlowHeight = 0
        for topic, msg, t in rosrecord.logplayer( bagFilename ):
            if msg._type == "sensor_msgs/Image":
                
                if numImages == 0:
                    opticalFlowWidth = self.opticalFlowFilter.calcOpticalFlowWidth( msg.width )
                    opticalFlowHeight = self.opticalFlowFilter.calcOpticalFlowHeight( msg.height )
                    lastCameraTime = t
                    numImages = 1
                    
                else:
                    if t != lastCameraTime:
                        numImages += 1
                        lastCameraTime = t
        
        self.imageTimes = np.ndarray(shape=( numImages ), dtype=np.float32)
        opticalFlowArrayShape = ( opticalFlowHeight, opticalFlowWidth, numImages )
        self.opticalFlowArraysX = np.ndarray(shape=opticalFlowArrayShape, dtype=np.float32)
        self.opticalFlowArraysY = np.ndarray(shape=opticalFlowArrayShape, dtype=np.float32)
        imgIdx = 0
        
        lastAngleTime = None
        lastCameraTime = None
        
        # Extract messages from the bag
        startTime = None
        for topic, msg, t in rosrecord.logplayer( bagFilename ):
            if startTime == None:
                startTime = t
                
            bagTime = t - startTime
                
            if msg._type == "arm_driver_msgs/SetServoAngles" \
                and bagTime != lastAngleTime:
                
                lastAngleTime = bagTime
                servoAngleTimes.append( bagTime.to_sec() )
                
                servoAngle = msg.servoAngles[ 0 ].angle
                servoAngleData.append( servoAngle )
                
            elif msg._type == "sensor_msgs/Image" \
                and bagTime != lastCameraTime:
                
                opticalFlowArrayX, opticalFlowArrayY = self.processCameraImage( msg )

                lastCameraTime = bagTime
                self.imageTimes[ imgIdx ] = bagTime.to_sec()
                self.cameraImages.append( msg )
                
                self.opticalFlowArraysX[ :, :, imgIdx ] = np.array( opticalFlowArrayX )
                self.opticalFlowArraysY[ :, :, imgIdx ] = np.array( opticalFlowArrayY )
                
                imgIdx += 1

        # Construct regular sampled data from the input servo data
        servoAngleData = normaliseSequence( servoAngleData )
        dataDuration = math.floor( self.imageTimes[ -1 ] )
        
        self.regularSampleTimes = np.arange( 0.0, dataDuration, 1.0/self.SAMPLES_PER_SECOND )
        
        t1 = time.time()
        # Pad the servo readings with zeros at the start and end
        preTimes = np.arange( 0.0, servoAngleTimes[ 0 ], 0.1 )
        preReadings = np.zeros( len( preTimes ) )
        postTimes = np.arange( servoAngleTimes[ -1 ] + 0.1, dataDuration + 1.0, 0.1 )
        postReadings = np.zeros( len( postTimes ) )
        
        npAngleTimes = np.concatenate( [ preTimes, np.array( servoAngleTimes ), postTimes ] )
        npAngleData = np.concatenate( [ preReadings, np.array( servoAngleData ), postReadings ] )
        
        self.regularServoAngleData = resampleSequence( 
            npAngleData, npAngleTimes, self.regularSampleTimes )
                    
        self.regularOpticalFlowArrayX = np.apply_along_axis( resampleSequence, 2, 
            self.opticalFlowArraysX, self.imageTimes, self.regularSampleTimes )
        self.regularOpticalFlowArrayY = np.apply_along_axis( resampleSequence, 2, 
            self.opticalFlowArraysY, self.imageTimes, self.regularSampleTimes )
        
        t2 = time.time()
        print 'Resampling took %0.3f ms' % ((t2-t1)*1000.0)
                    
        t1 = time.time()
        
        maxLag = int( 1.0 * self.SAMPLES_PER_SECOND )
        self.correlationsX = np.apply_along_axis( crossCorrelateComplete, 2, 
            self.regularOpticalFlowArrayX, self.regularServoAngleData, maxLag )
        self.correlationsY = np.apply_along_axis( crossCorrelateComplete, 2, 
            self.regularOpticalFlowArrayY, self.regularServoAngleData, maxLag )
        
        t2 = time.time()
        print 'Correlation took %0.3f ms' % ((t2-t1)*1000.0)
        
        # Detect the input signal based on the correlation in the x and y axis
        maxCorrelationArrayX = np.maximum.reduce( np.absolute( self.correlationsX ), axis=2 )
        maxCorrelationArrayY = np.maximum.reduce( np.absolute( self.correlationsY ), axis=2 )
        self.inputSignalDetectedArray = np.frompyfunc( isInputSignalPresent, 2, 1 )(
            maxCorrelationArrayX, maxCorrelationArrayY )
          
        # Build a histogram for the gripper  
        self.gripperHistogram = cv.CreateHist( 
            [ 256/8, 256/8, 256/8 ], cv.CV_HIST_ARRAY, [ (0,255), (0,255), (0,255) ], 1 )
            
        imageRGB = cv.CreateImageHeader( ( self.cameraImages[ 0 ].width, self.cameraImages[ 0 ].height ), cv.IPL_DEPTH_8U, 3 )
        cv.SetData( imageRGB, self.cameraImages[ 0 ].data, self.cameraImages[ 0 ].step )
            
        r_plane = cv.CreateMat( imageRGB.height, imageRGB.width, cv.CV_8UC1 )
        g_plane = cv.CreateMat( imageRGB.height, imageRGB.width, cv.CV_8UC1 )
        b_plane = cv.CreateMat( imageRGB.height, imageRGB.width, cv.CV_8UC1 )
        cv.Split( imageRGB, r_plane, g_plane, b_plane, None )
        planes = [ r_plane, g_plane, b_plane ]

        maskArray = np.zeros(shape=( imageRGB.height, imageRGB.width ), dtype=np.uint8 )
        for rowIdx in range( self.inputSignalDetectedArray.shape[ 0 ] ):
            for colIdx in range( self.inputSignalDetectedArray.shape[ 1 ] ):
                
                if self.inputSignalDetectedArray[ rowIdx, colIdx ]:
                    rowStartIdx = rowIdx*self.OPTICAL_FLOW_BLOCK_HEIGHT
                    rowEndIdx = rowStartIdx + self.OPTICAL_FLOW_BLOCK_HEIGHT
                    colStartIdx = colIdx*self.OPTICAL_FLOW_BLOCK_WIDTH
                    colEndIdx = colStartIdx + self.OPTICAL_FLOW_BLOCK_WIDTH
                    
                    maskArray[ rowStartIdx:rowEndIdx, colStartIdx:colEndIdx ] = 255

        cv.CalcHist( [ cv.GetImage( i ) for i in planes ], 
            self.gripperHistogram, 0, mask=maskArray )
        
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
        
        regularOpticalFlowDataX = self.regularOpticalFlowArrayX[ testY ][ testX ]
        regularOpticalFlowDataX = normaliseSequence( regularOpticalFlowDataX )
        regularOpticalFlowDataY = self.regularOpticalFlowArrayY[ testY ][ testX ]
        regularOpticalFlowDataY = normaliseSequence( regularOpticalFlowDataY )
        
        crossCorrelationOpticalFlowX = self.correlationsX[ testY ][ testX ]
        crossCorrelationOpticalFlowY = self.correlationsY[ testY ][ testX ]
        
        # Plot graphs
        self.axisX.clear()
        self.axisX.plot( self.regularSampleTimes, self.regularServoAngleData )
        self.axisX.plot( self.regularSampleTimes, regularOpticalFlowDataX )
        self.axisX.plot( self.regularSampleTimes[:len(crossCorrelationOpticalFlowX)], crossCorrelationOpticalFlowX )
        #self.axisX.plot( self.regularSampleTimes, evolvingCCOpticalFlowX )
        
        opticalFlowDataX = self.opticalFlowArraysX[ testY ][ testX ]
        opticalFlowDataX = normaliseSequence( opticalFlowDataX )
        blah = resampleSequence( opticalFlowDataX, self.imageTimes, self.regularSampleTimes )
        blah = normaliseSequence( blah )
        print max( self.imageTimes ), max( self.regularSampleTimes ) 
        self.axisX.plot( self.imageTimes, opticalFlowDataX )
        #self.axisX.plot( self.regularSampleTimes, blah )
        
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
            
        # Track gripper
        imageRGB = cv.CreateImageHeader( ( image.width, image.height ), cv.IPL_DEPTH_8U, 3 )
        cv.SetData( imageRGB, image.data, image.step )
        imageRGB = cv.CloneImage( imageRGB )
            
        r_plane = cv.CreateMat( imageRGB.height, imageRGB.width, cv.CV_8UC1 )
        g_plane = cv.CreateMat( imageRGB.height, imageRGB.width, cv.CV_8UC1 )
        b_plane = cv.CreateMat( imageRGB.height, imageRGB.width, cv.CV_8UC1 )
        cv.Split( imageRGB, r_plane, g_plane, b_plane, None )
        planes = [ r_plane, g_plane, b_plane ]
        
        backproject = cv.CreateImage(cv.GetSize(imageRGB), 8, 1)

        # Run the cam-shift
        cv.CalcArrBackProject( planes, backproject, self.gripperHistogram )
        #cv.Threshold( backproject, backproject, 1, 255, cv.CV_THRESH_BINARY )
        cv.CvtColor( backproject, imageRGB, cv.CV_GRAY2RGB )
        
        #self.cameraImagePixBuf = gtk.gdk.pixbuf_new_from_data( 
            #imageRGB.tostring(), 
            #gtk.gdk.COLORSPACE_RGB,
            #False,
            #8,
            #imageRGB.width,
            #imageRGB.height,
            #imageRGB.width*3 )
        

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
               
            # Draw an overlay to show places where the input motion has been detected
            if self.inputSignalDetectedArray != None:
                
                graphicsContext = widget.window.new_gc()
                graphicsContext.set_rgb_fg_color( gtk.gdk.Color( 65535, 65535, 0 ) )
                
                blockY = imgRect.y
                for y in range( self.inputSignalDetectedArray.shape[ 0 ] ):
                
                    blockX = imgRect.x
                    for x in range( self.inputSignalDetectedArray.shape[ 1 ] ):
                        
                        if self.inputSignalDetectedArray[ y, x ]:
                            points = [ (blockX+int((i*2)%self.OPTICAL_FLOW_BLOCK_WIDTH), blockY+2*int((i*2)/self.OPTICAL_FLOW_BLOCK_WIDTH)) \
                                for i in range( self.OPTICAL_FLOW_BLOCK_WIDTH*self.OPTICAL_FLOW_BLOCK_HEIGHT/4 ) ]
                                
                            widget.window.draw_points( graphicsContext, points )
                            
                        blockX += self.OPTICAL_FLOW_BLOCK_WIDTH
                        
                    blockY += self.OPTICAL_FLOW_BLOCK_HEIGHT
             
            #return
               
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

