#! /usr/bin/python

# ROS imports
import roslib
roslib.load_manifest( 'object_detector' )
import rospy

import sys
import math
import os.path
import time

import numpy as np
import scipy.signal
import scipy.ndimage.filters as filters
import cv

import yaml
import pygtk
pygtk.require('2.0')
import gtk
import gobject

import sensor_msgs.msg
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
class MainWindow:
    
    DEFAULT_SALIENCY_MAP_WIDTH = 64
    DEFAULT_SALIENCY_MAP_HEIGHT = 64
    
    NUM_GAUSSIAN_MODEL_FRAMES = 10
 
    #---------------------------------------------------------------------------
    def __init__( self ):
    
        self.scriptPath = os.path.dirname( __file__ )
        self.cameraImagePixBuf = None
        self.motionPixBuf = None
        self.curImage = None
        self.filename = None
            
        # Load up the video
        self.videoCapture = cv.CaptureFromFile( self.scriptPath + "/../../test_data/video/MVI_0028.AVI" )
        
        cv.SetCaptureProperty( self.videoCapture, cv.CV_CAP_PROP_POS_FRAMES, 0 )    
        self.pixelModelBuffer = None
        self.numFramesProcessed = 0
            
        # Setup the GUI        
        builder = gtk.Builder()
        builder.add_from_file( self.scriptPath + "/GUI/MotionDetectionExplorer.glade" )
        
        self.window = builder.get_object( "winMain" )   
        self.dwgCameraImage = builder.get_object( "dwgCameraImage" )
        self.dwgMotion = builder.get_object( "dwgMotion" )
        self.sequenceControls = builder.get_object( "sequenceControls" )
        self.sequenceControls.setNumFrames( cv.GetCaptureProperty( self.videoCapture, cv.CV_CAP_PROP_FRAME_COUNT ) )
        self.sequenceControls.setOnFrameIdxChangedCallback( self.onSequenceControlsFrameIdxChanged )
        
        builder.connect_signals( self )
               
        updateLoop = self.update()
        gobject.idle_add( updateLoop.next )
        
        self.window.show()
        self.setFrameIdx( 100 )
        
    #---------------------------------------------------------------------------
    def onWinMainDestroy( self, widget, data = None ):  
        gtk.main_quit()
        
    #---------------------------------------------------------------------------   
    def main( self ):
        # All PyGTK applications must have a gtk.main(). Control ends here
        # and waits for an event to occur (like a key press or mouse event).
        gtk.main()
                
    #---------------------------------------------------------------------------
    def setFrameIdx( self, frameIdx ):
        
        self.frameIdx = frameIdx
        
        # Get the frame from the video
        cv.SetCaptureProperty( self.videoCapture, cv.CV_CAP_PROP_POS_FRAMES, self.frameIdx )
        cvImage = cv.QueryFrame( self.videoCapture )
        if self.pixelModelBuffer == None:
            self.pixelModelBuffer = np.zeros( 
                shape=( cvImage.height, cvImage.width, self.NUM_GAUSSIAN_MODEL_FRAMES ),
                dtype=np.float32 )
        
        # Convert to RGB
        cv.CvtColor( cvImage, cvImage, cv.CV_BGR2RGB )
        
        # Display the frame
        self.cameraImagePixBuf = gtk.gdk.pixbuf_new_from_data( 
            cvImage.tostring(), 
            gtk.gdk.COLORSPACE_RGB,
            False,
            8,
            cvImage.width,
            cvImage.height,
            cvImage.width*3 )
            
        # Convert to gray scale
        imageGray = cv.CreateImage( cv.GetSize( cvImage ), cv.IPL_DEPTH_8U, 1 )
        cv.CvtColor( cvImage, imageGray, cv.CV_RGB2GRAY )
        
        # Detect motion between this frame and the last one
        motionImage = self.calculateMotion( imageGray )
        
        # Display the motion image
        motionImageRGB = cv.CreateImage( cv.GetSize( motionImage ), cv.IPL_DEPTH_8U, 3 )
        cv.CvtColor( motionImage, motionImageRGB, cv.CV_GRAY2RGB )
            
        self.motionPixBuf = gtk.gdk.pixbuf_new_from_data( 
            motionImageRGB.tostring(), 
            gtk.gdk.COLORSPACE_RGB,
            False,
            8,
            motionImageRGB.width,
            motionImageRGB.height,
            motionImageRGB.width*3 )

        # Resize the drawing areas if necessary
        if self.dwgCameraImage.get_size_request() != ( cvImage.width, cvImage.height ):
            self.dwgCameraImage.set_size_request( cvImage.width, cvImage.height )
        if self.dwgMotion.get_size_request() != ( motionImageRGB.width, motionImageRGB.height ):
            self.dwgMotion.set_size_request( motionImageRGB.width, motionImageRGB.height )

        self.dwgCameraImage.queue_draw()
        self.dwgMotion.queue_draw()
        
    #---------------------------------------------------------------------------
    def calculateMotion( self, imageGray ):
        
        imageGray = np.array( cv.GetMat( imageGray ), dtype=np.float32 )
        
        if self.numFramesProcessed < self.pixelModelBuffer.shape[ 2 ]:
            # Add the image to the current Gaussian pixel model
            frameBufferIdx = self.numFramesProcessed%self.pixelModelBuffer.shape[ 2 ]
            self.pixelModelBuffer[ :, :, frameBufferIdx ] = imageGray
            self.numFramesProcessed += 1
            
            if self.numFramesProcessed >= self.pixelModelBuffer.shape[ 2 ]:
                self.pixelDiffThreshold = 15.0*np.std( self.pixelModelBuffer, axis=2 )
                self.pixelLowerThreshold = 5.0*np.std( self.pixelModelBuffer, axis=2 )
                self.pixelUpperThreshold = 25.0*np.std( self.pixelModelBuffer, axis=2 )
                self.pixelThresholdDiff = self.pixelUpperThreshold - self.pixelLowerThreshold
                self.pixelMean = np.mean( self.pixelModelBuffer, axis=2 )
        else:
            self.numFramesProcessed += 1
        
        if self.numFramesProcessed > self.pixelModelBuffer.shape[ 2 ]:
            # We have enough frames to form our pixel models so calculate motion
            t1 = time.time()
            imageAbsDiff = np.abs( imageGray - self.pixelMean )
            prob = ( imageAbsDiff - self.pixelLowerThreshold ) / self.pixelThresholdDiff
            prob *= 255.0
            np.clip( prob, 0.0, 255.0, prob )
            motionImage = np.array( prob, dtype=np.uint8 )
            
            #motionImage = np.array( 
            #    np.where( imageAbsDiff > self.pixelDiffThreshold, 255, 0 ), dtype=np.uint8 )
            t2 = time.time()
            print "Took {0:2.3} ms to calculate motion image".format( ( t2 - t1 )*1000.0 )
            
            self.pixelMean = imageGray
            
        else:
            motionImage = np.zeros( shape=imageGray.shape, dtype=np.uint8 )
        
        return motionImage
    
    #---------------------------------------------------------------------------
    def onSequenceControlsFrameIdxChanged( self, widget ):
        self.setFrameIdx( widget.frameIdx )
    
    #---------------------------------------------------------------------------
    def onMenuItemLoadImageActivate( self, widget ):
        
        pass
    
    #---------------------------------------------------------------------------
    def onMenuItemQuitActivate( self, widget ):
        self.onWinMainDestroy( widget )
        
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
              
    #---------------------------------------------------------------------------
    def onDwgMotionExposeEvent( self, widget, data = None ):
        
        if self.motionPixBuf != None:
            
            imgRect = self.getImageRectangleInWidget( widget,
                self.motionPixBuf.get_width(), self.motionPixBuf.get_height() )
                
            imgOffsetX = imgRect.x
            imgOffsetY = imgRect.y
                
            # Get the total area that needs to be redrawn
            imgRect = imgRect.intersect( data.area )
        
            srcX = imgRect.x - imgOffsetX
            srcY = imgRect.y - imgOffsetY
           
            widget.window.draw_pixbuf( widget.get_style().fg_gc[ gtk.STATE_NORMAL ],
                self.motionPixBuf, srcX, srcY, 
                imgRect.x, imgRect.y, imgRect.width, imgRect.height )

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

        lastTime = time.time()

        while 1:
            
            curTime = time.time()
            
                
            yield True
            
        yield False
        
        
#-------------------------------------------------------------------------------
if __name__ == "__main__":

    mainWindow = MainWindow()
    mainWindow.main()
