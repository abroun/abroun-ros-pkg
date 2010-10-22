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

import OpticalFlow.Utils as Utils
from OpticalFlow.InputSequence import InputSequence, Distractor
from OpticalFlow.RegularisedInputSequence import RegularisedInputSequence
from OpticalFlow.CrossCorrelatedSequence import CrossCorrelatedSequence
from OpticalFlow.ROCCurve import ROCCurve, GripperDetectorROCCurve
import OpticalFlow.MarkerBuffer as MarkerBuffer

def printTiming(func):
    def wrapper(*arg):
    
        t1 = time.time()
        res = func(*arg)
        t2 = time.time()
        
        print '%s took %0.3f ms' % (func.func_name, (t2-t1)*1000.0)
        return res

    return wrapper

#-------------------------------------------------------------------------------
class MainWindow:
    
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
    #GROUND_TRUTH_FILENAME = "/../../config/OnTablePosGripper.yaml"
    GROUND_TRUTH_FILENAME = "/../../config/BasicWave_Gripper.yaml"
    
    CORRELATION_THRESHOLD = 0.6
    MAX_TEST_POINT_X = (320 - OPTICAL_FLOW_BLOCK_WIDTH)/OPTICAL_FLOW_BLOCK_WIDTH - 1
    MAX_TEST_POINT_Y = (240 - OPTICAL_FLOW_BLOCK_HEIGHT)/OPTICAL_FLOW_BLOCK_HEIGHT - 1
    
    SAMPLES_PER_SECOND = 30.0
    MAX_CORRELATION_LAG = 1.0
    
    GRIPPER_WAVE_FREQUENCY = 1.0    # Waves per second
    GRIPPER_NUM_WAVES = 3.0
    GRIPPER_WAVE_AMPLITUDE = math.radians( 20.0 )
 
    #---------------------------------------------------------------------------
    def __init__( self, bagFilename ):
    
        self.scriptPath = os.path.dirname( __file__ )
        self.cameraImagePixBuf = None
        self.bagFilename = bagFilename
        self.lastImageGray = None
        
        # Read in sequence
        t1 = time.time()
        
        self.inputSequence = InputSequence( bagFilename )
        
        distractors = [
            Distractor( radius=24, startPos=( 25, 35 ), endPos=( 100, 100 ), frequency=2.0 ),
            Distractor( radius=24, startPos=( 200, 200 ), endPos=( 150, 50 ), frequency=0.25 ),
            Distractor( radius=24, startPos=( 188, 130 ), endPos=( 168, 258 ), frequency=0.6 ),
            Distractor( radius=24, startPos=( 63, 94 ), endPos=( 170, 81 ), frequency=1.5 ),
            Distractor( radius=24, startPos=( 40, 287 ), endPos=( 50, 197 ), frequency=3.0 ) ]
        self.inputSequence.addDistractorObjects( distractors )
        
        self.inputSequence.calculateOpticalFlow(
            self.OPTICAL_FLOW_BLOCK_WIDTH, self.OPTICAL_FLOW_BLOCK_HEIGHT,
            self.OPTICAL_FLOW_RANGE_WIDTH, self.OPTICAL_FLOW_RANGE_HEIGHT,
            self.OPTICAL_FLOW_METHOD )
            
        t2 = time.time()
        print 'Processing sequence took %0.3f ms' % ((t2-t1)*1000.0)
        
        # Resample sequence
        t1 = time.time()
        
        self.regularisedInputSequence = RegularisedInputSequence( 
            self.inputSequence, self.SAMPLES_PER_SECOND )
            
        t2 = time.time()
        print 'Resampling took %0.3f ms' % ((t2-t1)*1000.0)
        
        
                    
        t1 = time.time()
        
        self.crossCorrelatedSequence = CrossCorrelatedSequence( 
            self.regularisedInputSequence, 
            self.MAX_CORRELATION_LAG, self.COMBINATION_METHOD )
        
        t2 = time.time()
        print 'Correlation took %0.3f ms' % ((t2-t1)*1000.0)
        
        # Detect the input signal based on the correlation in the x and y axis
        self.inputSignalDetectedArray = \
            self.crossCorrelatedSequence.detectInputSequence( self.CORRELATION_THRESHOLD )
          
        # Build a histogram for the gripper  
        self.gripperHistogram = cv.CreateHist( 
            [ 256/8, 256/8, 256/8 ], cv.CV_HIST_ARRAY, [ (0,255), (0,255), (0,255) ], 1 )
            
        firstImage = self.inputSequence.cameraImages[ 0 ]
        imageRGB = cv.CreateImageHeader( ( firstImage.shape[ 1 ], firstImage.shape[ 0 ] ), cv.IPL_DEPTH_8U, 3 )
        cv.SetData( imageRGB, firstImage.data, firstImage.shape[ 1 ]*3 )
            
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
        
        markerBuffer = MarkerBuffer.loadMarkerBuffer( self.scriptPath + self.GROUND_TRUTH_FILENAME )
        if markerBuffer == None:
            raise Exception( "Unable to load marker buffer" )
        
        self.rocCurve = GripperDetectorROCCurve( self.crossCorrelatedSequence, markerBuffer )
        
        # Create the matplotlib graph
        self.figure = Figure( figsize=(8,6), dpi=72 )
        self.axisX = self.figure.add_subplot( 311 )
        self.axisY = self.figure.add_subplot( 312 )
        self.axisROC = self.figure.add_subplot( 313 )
        
        self.canvas = None  # Wait for GUI to be created before creating canvas
        self.navToolbar = None
 
        # Setup the GUI        
        builder = gtk.Builder()
        builder.add_from_file( self.scriptPath + "/GUI/OpticalFlowExplorer.glade" )
        
        self.dwgCameraImage = builder.get_object( "dwgCameraImage" )
        self.window = builder.get_object( "winMain" )
        self.vboxMain = builder.get_object( "vboxMain" )
        self.hboxWorkArea = builder.get_object( "hboxWorkArea" )
        self.adjTestPointX = builder.get_object( "adjTestPointX" )
        self.adjTestPointY = builder.get_object( "adjTestPointY" )
        self.adjTestPointX.set_upper( self.MAX_TEST_POINT_X )
        self.adjTestPointY.set_upper( self.MAX_TEST_POINT_Y )
        self.sequenceControls = builder.get_object( "sequenceControls" )
        self.sequenceControls.setNumFrames( len( self.inputSequence.cameraImages ) )
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
        
        regSeq = self.regularisedInputSequence
        corSeq = self.crossCorrelatedSequence
        
        # Normalise the data ready for display
        normalisedServoAngleData = Utils.normaliseSequence( regSeq.regularServoAngleData )
        normalisedOpticalFlowDataX = Utils.normaliseSequence( regSeq.regularOpticalFlowArrayX[ testY ][ testX ] )
        normalisedOpticalFlowDataY = Utils.normaliseSequence( regSeq.regularOpticalFlowArrayY[ testY ][ testX ] )
        
        numCorrelationChannels = len( corSeq.correlationChannels )
        
        # Plot graphs
        self.axisX.clear()
        self.axisX.plot( regSeq.regularSampleTimes, normalisedServoAngleData )
        self.axisX.plot( regSeq.regularSampleTimes, normalisedOpticalFlowDataX )
        
        if numCorrelationChannels >= 1:
            correlationChannel = corSeq.correlationChannels[ 0 ][ testY ][ testX ]
            self.axisX.plot( regSeq.regularSampleTimes[:len(correlationChannel)], correlationChannel )
        
        self.axisY.clear()
        self.axisY.plot( regSeq.regularSampleTimes, normalisedServoAngleData )
        self.axisY.plot( regSeq.regularSampleTimes, normalisedOpticalFlowDataY )
        
        inpSeq = self.inputSequence
        self.axisY.plot( inpSeq.imageTimes, Utils.normaliseSequence( inpSeq.opticalFlowArraysY[ testY ][ testX ] ) )
        
        if numCorrelationChannels >= 2:
            correlationChannel = corSeq.correlationChannels[ 1 ][ testY ][ testX ]
            self.axisY.plot( regSeq.regularSampleTimes[:len(correlationChannel)], correlationChannel )
        
        self.axisROC.clear()
        self.axisROC.plot( self.rocCurve.falsePositiveRates, self.rocCurve.truePositiveRates )
        #self.axisROC.plot( self.rocCurve.thresholds, self.rocCurve.sensitivity )
        #self.axisROC.plot( self.rocCurve.thresholds, self.rocCurve.specificity )
        
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
    def setFrameIdx( self, frameIdx ):
        
        self.frameIdx = frameIdx
        
        # Display the frame
        image = self.inputSequence.cameraImages[ frameIdx ]
        imageWidth = image.shape[ 1 ]
        imageHeight = image.shape[ 0 ]
        imageStep = imageWidth*3
        
        self.cameraImagePixBuf = gtk.gdk.pixbuf_new_from_data( 
            image.tostring(), 
            gtk.gdk.COLORSPACE_RGB,
            False,
            8,
            imageWidth,
            imageHeight,
            imageStep )
            
        # Track gripper
        imageRGB = cv.CreateImageHeader( ( imageWidth, imageHeight ), cv.IPL_DEPTH_8U, 3 )
        cv.SetData( imageRGB, image.data, imageStep )
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
        if self.dwgCameraImage.get_size_request() != ( imageWidth, imageHeight ):
            self.dwgCameraImage.set_size_request( imageWidth, imageHeight )

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
             
            return
               
            # Draw the optical flow if it's available
            opticalFlowX = self.inputSequence.opticalFlowArraysX[ :, :, self.frameIdx ]
            opticalFlowY = self.inputSequence.opticalFlowArraysY[ :, :, self.frameIdx ]
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

