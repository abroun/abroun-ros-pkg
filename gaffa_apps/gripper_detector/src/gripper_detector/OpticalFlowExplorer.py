#! /usr/bin/python

# ROS imports
import roslib
roslib.load_manifest( 'gripper_detector' )
import rospy
from ros import rosrecord


import sys
import math
import os.path
import time

import numpy as np
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

def printTiming(func):
    def wrapper(*arg):
    
        t1 = time.time()
        res = func(*arg)
        t2 = time.time()
        
        print '%s took %0.3f ms' % (func.func_name, (t2-t1)*1000.0)
        return res

    return wrapper

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
    
#-------------------------------------------------------------------------------
def crossCorrelate( sequence, laggedSequence, lag ):
    
    acc = 0
    sequenceLength = len( sequence )
    laggedSequenceLength = len( laggedSequence )
    
    curIdx = 0
    laggedIdx = curIdx + lag
    while curIdx < sequenceLength and laggedIdx < laggedSequenceLength:
        acc += sequence[ curIdx ]*laggedSequence[ laggedIdx ]
        curIdx += 1
        laggedIdx += 1
        
    # If the lagged sequence ended prematurely we assume that we just added on
    # zeroes in place of the missing samples
    numSamplesTested = curIdx
    return acc / numSamplesTested
    
#-------------------------------------------------------------------------------
# Calculates the cross correlation of a sequence for all possible lags
def crossCorrelateComplete( sequence, laggedSequence ):
    return [ crossCorrelate( sequence, laggedSequence, lag ) for lag in range( len( sequence ) ) ]
    
#-------------------------------------------------------------------------------
def calculateEvolvingCrossCorrelation( sequence, laggedSequence, lag ):
    return [ crossCorrelate( sequence[:sliceLength], laggedSequence, lag ) \
        for sliceLength in range( 1, len( sequence ) + 1 ) ]

#-------------------------------------------------------------------------------
class MainWindow:
    
    OPTICAL_FLOW_BLOCK_WIDTH = 16
    OPTICAL_FLOW_BLOCK_HEIGHT = 16
    OPTICAL_FLOW_RANGE_WIDTH = 8    # Range to look outside of a block for motion
    OPTICAL_FLOW_RANGE_HEIGHT = 8
    
    TEST_X = 17 #17
    TEST_Y = 8 #8
    
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
        self.opticalFlowArraysX = []    #  May contain 'None' elements
        self.opticalFlowArraysY = []    #  May contain 'None' elements
        
        
        # Calculate optical flow for all frames
        
        # When sample point is selected
            # Resample optical flow
            # Update graphs
            
        # Allow user to move back and forwards through the frames
        
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
                self.opticalFlowArraysX.append( opticalFlowArrayX )
                self.opticalFlowArraysY.append( opticalFlowArrayY )
                
                #flowX = cv.Get2D( self.opticalFlowX, self.TEST_Y, self.TEST_X )[ 0 ] \
                    #/ float( self.OPTICAL_FLOW_RANGE_WIDTH )
                #flowY = cv.Get2D( self.opticalFlowY, self.TEST_Y, self.TEST_X )[ 0 ] \
                    #/ float( self.OPTICAL_FLOW_RANGE_HEIGHT )
                ##flowY = -flowY
                
                #opticalFlowDataX.append( flowX )
                #opticalFlowDataY.append( flowY )
                #opticalFlowDataMag.append( flowX + flowY )

        # Construct regular sampled data from the input servo data
        servoAngleData = normaliseSequence( servoAngleData )
        dataDuration = math.ceil( max( servoAngleTimes[ -1 ], self.imageTimes[ -1 ] ) )
        
        regularSampleTimes = [ i*1.0/self.SAMPLES_PER_SECOND for i in range( int( dataDuration*self.SAMPLES_PER_SECOND ) ) ]
        regularServoAngleData = resampleSequence( servoAngleTimes, servoAngleData, regularSampleTimes )
        


        
        #opticalFlowDataX = normaliseSequence( opticalFlowDataX )
        #opticalFlowDataY = normaliseSequence( opticalFlowDataY )
        #opticalFlowDataMag = normaliseSequence( opticalFlowDataMag )
        
        #regularOpticalFlowDataX = resampleSequence( imageTimes, opticalFlowDataX, regularSampleTimes )
        #regularOpticalFlowDataY = resampleSequence( imageTimes, opticalFlowDataY, regularSampleTimes )
        
        #crossCorrelationOpticalFlowX = crossCorrelateComplete( regularServoAngleData, regularOpticalFlowDataX )
        #evolvingCCOpticalFlowX = calculateEvolvingCrossCorrelation(
            #regularServoAngleData, regularOpticalFlowDataX, int( 0.45*self.SAMPLES_PER_SECOND ) )
        #crossCorrelationOpticalFlowY = crossCorrelateComplete( regularServoAngleData, regularOpticalFlowDataY )
        
        # Create the matplotlib graph
        self.figure = Figure( figsize=(8,6), dpi=72 )
        self.axisX = self.figure.add_subplot( 211 )
        self.axisY = self.figure.add_subplot( 212 )
        #self.axisMag = self.figure.add_subplot( 313 )
        
        
        #self.axisX.plot( regularSampleTimes, regularServoAngleData )
        #self.axisX.plot( regularSampleTimes, regularOpticalFlowDataX )
        #self.axisX.plot( regularSampleTimes, crossCorrelationOpticalFlowX )
        #self.axisX.plot( regularSampleTimes, evolvingCCOpticalFlowX )
        

        
        #self.axisY.plot( regularSampleTimes, regularServoAngleData )
        #self.axisY.plot( regularSampleTimes, regularOpticalFlowDataY )
        #self.axisY.plot( regularSampleTimes, crossCorrelationOpticalFlowY )
        
        self.canvas = None  # Wait for GUI to be created before creating canvas
        self.navToolbar = None
 
        # Setup the GUI        
        builder = gtk.Builder()
        builder.add_from_file( scriptPath + "/GUI/OpticalFlowExplorer.glade" )
        
        self.dwgCameraImage = builder.get_object( "dwgCameraImage" )
        self.window = builder.get_object( "winMain" )
        self.vboxMain = builder.get_object( "vboxMain" )
        self.hboxWorkArea = builder.get_object( "hboxWorkArea" )
        self.sequenceControls = builder.get_object( "sequenceControls" )
        self.sequenceControls.setNumFrames( len( self.cameraImages ) )
        self.sequenceControls.setOnFrameIdxChangedCallback( self.onSequenceControlsFrameIdxChanged )
        self.setFrameIdx( 0 )
        
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
    def refreshGraphDisplay( self ):
        
        if self.canvas != None:   
            self.canvas.destroy()  
            self.canvas = None   
        if self.navToolbar != None:
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
    def createOpticalFlowStorage( self, opticalFlowArrayX = None, opticalFlowArrayY = None ):
        
        if self.lastImageGray == None:
            raise Exception( "No image to measure for optical flow" )
        
        storageWidth = (self.lastImageGray.width - self.OPTICAL_FLOW_BLOCK_WIDTH)/self.OPTICAL_FLOW_BLOCK_WIDTH
        storageHeight = (self.lastImageGray.height - self.OPTICAL_FLOW_BLOCK_HEIGHT)/self.OPTICAL_FLOW_BLOCK_HEIGHT
        
        if opticalFlowArrayX == None \
            or storageWidth != opticalFlowArrayX.width \
            or storageHeight != opticalFlowArrayX.height:
                
            opticalFlowArrayX = cv.CreateMat( storageHeight, storageWidth, cv.CV_32FC1 )

        if opticalFlowArrayY == None \
            or storageWidth != opticalFlowArrayY.width \
            or storageHeight != opticalFlowArrayY.height:
                
            opticalFlowArrayY = cv.CreateMat( storageHeight, storageWidth, cv.CV_32FC1 )
            
        return ( opticalFlowArrayX, opticalFlowArrayY )
        
    #---------------------------------------------------------------------------
    #@printTiming
    def calcOpticalFlow( self, curImageGray, opticalFlowArrayX = None, opticalFlowArrayY = None ):
        if self.lastImageGray != None:
                
            opticalFlowArrayX, opticalFlowArrayY = \
                self.createOpticalFlowStorage( opticalFlowArrayX, opticalFlowArrayY )
            
            cv.CalcOpticalFlowBM( self.lastImageGray, curImageGray, 
                ( self.OPTICAL_FLOW_BLOCK_WIDTH, self.OPTICAL_FLOW_BLOCK_HEIGHT ),
                ( self.OPTICAL_FLOW_BLOCK_WIDTH, self.OPTICAL_FLOW_BLOCK_HEIGHT ),
                ( self.OPTICAL_FLOW_RANGE_WIDTH, self.OPTICAL_FLOW_RANGE_HEIGHT ),
                0, opticalFlowArrayX, opticalFlowArrayY )
            
        # Save the current image
        self.lastImageGray = curImageGray
        
        return ( opticalFlowArrayX, opticalFlowArrayY )
        
    #---------------------------------------------------------------------------
    def processCameraImage( self, rosImage ):
        
        opticalFlowArrayX = None
        opticalFlowArrayY = None
        
        if rosImage.encoding == "rgb8":
            
            # Create an OpenCV image to process the data
            curImageRGB = cv.CreateImageHeader( ( rosImage.width, rosImage.height ), cv.IPL_DEPTH_8U, 3 )
            cv.SetData( curImageRGB, rosImage.data, rosImage.step )
            curImageGray = cv.CreateImage( ( rosImage.width, rosImage.height ), cv.IPL_DEPTH_8U, 1 )
            cv.CvtColor( curImageRGB, curImageGray, cv.CV_RGB2GRAY )
            
            # Look for optical flow between this image and the last one
            opticalFlowArrayX, opticalFlowArrayY = self.calcOpticalFlow( curImageGray )
            
            ## Display the image
            #self.cameraImagePixBuf = gtk.gdk.pixbuf_new_from_data( 
                #rosImage.data, 
                #gtk.gdk.COLORSPACE_RGB,
                #False,
                #8,
                #rosImage.width,
                #rosImage.height,
                #rosImage.step )

            ## Resize the drawing area if necessary
            #if self.dwgCameraImage.get_size_request() != ( rosImage.width, rosImage.height ):
                #self.dwgCameraImage.set_size_request( rosImage.width, rosImage.height )

            #self.dwgCameraImage.queue_draw()

        else:
            rospy.logerr( "Unhandled image encoding - " + image.encoding )
            
        return ( opticalFlowArrayX, opticalFlowArrayY )
    
    #---------------------------------------------------------------------------
    def setFrameIdx( self, frameIdx ):
        
        self.frameIdx = frameIdx
        
        # Display the frame
        image = self.cameraImages[ frameIdx ]
        self.cameraImagePixBuf = gtk.gdk.pixbuf_new_from_data( 
            rosImage.data, 
            gtk.gdk.COLORSPACE_RGB,
            False,
            8,
            rosImage.width,
            rosImage.height,
            rosImage.step )

        # Resize the drawing area if necessary
        if self.dwgCameraImage.get_size_request() != ( rosImage.width, rosImage.height ):
            self.dwgCameraImage.set_size_request( rosImage.width, rosImage.height )

        self.dwgCameraImage.queue_draw()
        
    #---------------------------------------------------------------------------
    def onSequenceControlsFrameIdxChanged( self, widget ):
        self.setFrameIdx( widget.frameIdx )
        
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
            if self.opticalFlowX != None and self.opticalFlowY != None:
            
                graphicsContext = widget.window.new_gc()
                graphicsContext.set_rgb_fg_color( gtk.gdk.Color( 0, 65535, 0 ) )
                
                blockCentreY = self.OPTICAL_FLOW_BLOCK_HEIGHT / 2
                for y in range( self.opticalFlowX.height ):
                
                    blockCentreX = self.OPTICAL_FLOW_BLOCK_WIDTH / 2
                    for x in range( self.opticalFlowX.width ):
                        
                        endX = blockCentreX + cv.Get2D( self.opticalFlowX, y, x )[ 0 ]
                        endY = blockCentreY + cv.Get2D( self.opticalFlowY, y, x )[ 0 ]
                        
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

