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
    
    OPTICAL_FLOW_BLOCK_WIDTH = 16
    OPTICAL_FLOW_BLOCK_HEIGHT = 16
    OPTICAL_FLOW_RANGE_WIDTH = 8    # Range to look outside of a block for motion
    OPTICAL_FLOW_RANGE_HEIGHT = 8
    
    TEST_X = 17 #17
    TEST_Y = 8 #8
    
    GRIPPER_WAVE_FREQUENCY = 1.0    # Waves per second
    GRIPPER_NUM_WAVES = 3.0
    GRIPPER_WAVE_AMPLITUDE = math.radians( 20.0 )
 
    #---------------------------------------------------------------------------
    def __init__( self, bagFilename ):
    
        scriptPath = os.path.dirname( __file__ )
        self.cameraImagePixBuf = None
        self.bagFilename = bagFilename
        self.lastImageGray = None
        self.opticalFlowX = None
        self.opticalFlowY = None
        self.wavingGripper = False
        self.gripperWaveStartTime = None
        
        startTime = None
        servoAngleTimes = []
        servoAngleData = []
        imageTimes = []
        opticalFlowDataX = []
        opticalFlowDataY = []
        
        numServoAngleReadings = 0
        servoAngleMean = 0
        
        for topic, msg, t in rosrecord.logplayer( bagFilename ):
            if startTime == None:
                startTime = t
                
            bagTime = t - startTime
                
            if msg._type == "arm_driver_msgs/SetServoAngles":
                servoAngleTimes.append( bagTime.to_seconds() )
                
                servoAngle = msg.servoAngles[ 0 ].angle
                servoAngleData.append( servoAngle )
                
                numServoAngleReadings += 1
                servoAngleMean = servoAngleMean + (servoAngle - servoAngleMean)/(numServoAngleReadings)
                
            elif msg._type == "sensor_msgs/Image":
                
                self.processCameraImage( msg )
                
                if self.opticalFlowX != None and self.opticalFlowY != None:
                
                    imageTimes.append( bagTime.to_seconds() )
                    
                    opticalFlowDataX.append( cv.Get2D( self.opticalFlowX, self.TEST_Y, self.TEST_X )[ 0 ] 
                        / float( self.OPTICAL_FLOW_RANGE_WIDTH ) )
                    opticalFlowDataY.append( cv.Get2D( self.opticalFlowY, self.TEST_Y, self.TEST_X )[ 0 ]
                        / float( self.OPTICAL_FLOW_RANGE_HEIGHT ) )

        servoAngleData = [ angle - servoAngleMean for angle in servoAngleData ]
        
        # Create the matplotlib graph
        self.figure = Figure( figsize=(8,6), dpi=72 )
        self.axisX = self.figure.add_subplot( 211 )
        self.axisY = self.figure.add_subplot( 212 )
        
        print servoAngleTimes
        self.axisX.plot( servoAngleTimes, servoAngleData )
        self.axisX.plot( imageTimes, opticalFlowDataX )
        self.axisY.plot( servoAngleTimes, servoAngleData )
        self.axisY.plot( imageTimes, opticalFlowDataY )

        self.canvas = FigureCanvas( self.figure ) # a gtk.DrawingArea
        self.canvas.show()
                
        # Setup the GUI        
        builder = gtk.Builder()
        builder.add_from_file( scriptPath + "/GUI/OpticalFlowExplorer.glade" )
        
        self.dwgCameraImage = builder.get_object( "dwgCameraImage" )
        self.window = builder.get_object( "winMain" )
        self.vboxMain = builder.get_object( "vboxMain" )
        self.hboxWorkArea = builder.get_object( "hboxWorkArea" )
        
        self.hboxWorkArea.pack_start( self.canvas, True, True )
        self.hboxWorkArea.show()
        
        # Navigation toolbar
        self.navToolbar = NavigationToolbar( self.canvas, self.window )
        self.navToolbar.lastDir = '/var/tmp/'
        self.vboxMain.pack_start( self.navToolbar, expand=False, fill=False )
        self.navToolbar.show()
        self.vboxMain.show()
        
        builder.connect_signals( self )
               
        updateLoop = self.update()
        gobject.idle_add( updateLoop.next )
        
        self.window.show()
        
    #---------------------------------------------------------------------------
    def onWinMainDestroy( self, widget, data = None ):  
        gtk.main_quit()
        
    #---------------------------------------------------------------------------   
    def main( self ):
        # All PyGTK applications must have a gtk.main(). Control ends here
        # and waits for an event to occur (like a key press or mouse event).
        gtk.main()
        
    #---------------------------------------------------------------------------
    def createOpticalFlowStorage( self ):
        
        if self.lastImageGray == None:
            raise Exception( "No image to measure for optical flow" )
        
        storageWidth = (self.lastImageGray.width - self.OPTICAL_FLOW_BLOCK_WIDTH)/self.OPTICAL_FLOW_BLOCK_WIDTH
        storageHeight = (self.lastImageGray.height - self.OPTICAL_FLOW_BLOCK_HEIGHT)/self.OPTICAL_FLOW_BLOCK_HEIGHT
        
        if self.opticalFlowX == None \
            or storageWidth != self.opticalFlowX.width \
            or storageHeight != self.opticalFlowX.height:
                
            self.opticalFlowX = cv.CreateMat( storageHeight, storageWidth, cv.CV_32FC1 )

        if self.opticalFlowY == None \
            or storageWidth != self.opticalFlowY.width \
            or storageHeight != self.opticalFlowY.height:
                
            self.opticalFlowY = cv.CreateMat( storageHeight, storageWidth, cv.CV_32FC1 )
        
    #---------------------------------------------------------------------------
    @printTiming
    def calcOpticalFlow( self, curImageGray ):
        if self.lastImageGray != None:
                
            self.createOpticalFlowStorage()
            
            cv.CalcOpticalFlowBM( self.lastImageGray, curImageGray, 
                ( self.OPTICAL_FLOW_BLOCK_WIDTH, self.OPTICAL_FLOW_BLOCK_HEIGHT ),
                ( self.OPTICAL_FLOW_BLOCK_WIDTH, self.OPTICAL_FLOW_BLOCK_HEIGHT ),
                ( self.OPTICAL_FLOW_RANGE_WIDTH, self.OPTICAL_FLOW_RANGE_HEIGHT ),
                0, self.opticalFlowX, self.opticalFlowY )
            
        # Save the current image
        self.lastImageGray = curImageGray
        
    #---------------------------------------------------------------------------
    def processCameraImage( self, rosImage ):
        
        if rosImage.encoding == "rgb8":
            
            # Create an OpenCV image to process the data
            curImageRGB = cv.CreateImageHeader( ( rosImage.width, rosImage.height ), cv.IPL_DEPTH_8U, 3 )
            cv.SetData( curImageRGB, rosImage.data, rosImage.step )
            curImageGray = cv.CreateImage( ( rosImage.width, rosImage.height ), cv.IPL_DEPTH_8U, 1 )
            cv.CvtColor( curImageRGB, curImageGray, cv.CV_RGB2GRAY )
            
            # Look for optical flow between this image and the last one
            self.calcOpticalFlow( curImageGray )
            
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

