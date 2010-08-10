#! /usr/bin/python

# ROS imports
import roslib
roslib.load_manifest( 'gripper_detector' )
import rospy

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

import sensor_msgs.msg

#-------------------------------------------------------------------------------
class MainWindow:
    
    DEFAULT_OPTICAL_FLOW_BLOCK_WIDTH = 8
    DEFAULT_OPTICAL_FLOW_BLOCK_HEIGHT = 8
 
    #---------------------------------------------------------------------------
    def __init__( self ):
    
        self.scriptPath = os.path.dirname( __file__ )
        self.cameraImagePixBuf = None
        self.lastImage = None
        self.markerBuffer = None
        self.filename = None
            
        # Connect to the robot via ROS
        rospy.init_node( 'GripperMarker', anonymous=True )
        
        self.cameraImageTopic = rospy.Subscriber( "/camera/image", 
            sensor_msgs.msg.Image, self.cameraImageCallback )
            
        self.opticalFlowBlockWidth = self.DEFAULT_OPTICAL_FLOW_BLOCK_WIDTH
        self.opticalFlowBlockHeight = self.DEFAULT_OPTICAL_FLOW_BLOCK_HEIGHT
            
        # Setup the GUI        
        builder = gtk.Builder()
        builder.add_from_file( self.scriptPath + "/GUI/GripperMarker.glade" )
        
        self.window = builder.get_object( "winMain" )   
        self.dwgCameraImage = builder.get_object( "dwgCameraImage" )
        self.adjBlockWidth = builder.get_object( "adjBlockWidth" )
        self.adjBlockHeight = builder.get_object( "adjBlockHeight" )
        
        self.adjBlockWidth.set_value( self.opticalFlowBlockWidth )
        self.adjBlockHeight.set_value( self.opticalFlowBlockHeight )
        
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
    def chooseMarkerFile( self ):
        
        result = None
        
        dialog = gtk.FileChooserDialog(
            title="Choose Marker File",
            action=gtk.FILE_CHOOSER_ACTION_SAVE,
            buttons=(gtk.STOCK_CANCEL, gtk.RESPONSE_REJECT,
                      gtk.STOCK_OK, gtk.RESPONSE_ACCEPT) )

        dialog.set_current_folder( self.scriptPath + "/../../config" )
            
        filter = gtk.FileFilter()
        filter.add_pattern( "*.yaml" )
        filter.set_name( "Marker Files" )
        dialog.add_filter( filter )
        dialog.set_filter( filter )
            
        result = dialog.run()

        if result == gtk.RESPONSE_ACCEPT:
            result = dialog.get_filename()
            if os.path.splitext( result )[ 1 ] == "":
                result += ".yaml"

        dialog.destroy()
        
        return result
        
    #---------------------------------------------------------------------------
    def cameraImageCallback( self, rosImage ):
        
        if rosImage.encoding == "rgb8" or rosImage.encoding == "bgr8":
            
            # Display the image
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
            self.lastImage = rosImage
            
            if self.markerBuffer == None:
                self.setupMarkerBuffer()

        else:
            rospy.logerr( "Unhandled image encoding - " + rosImage.encoding )
    
    #---------------------------------------------------------------------------
    def onAdjBlockWidthValueChanged( self, widget, data = None ):
        
        self.opticalFlowBlockWidth = self.adjBlockWidth.get_value()
        if self.lastImage != None:
            self.setupMarkerBuffer()
            
    #---------------------------------------------------------------------------
    def onAdjBlockHeightValueChanged( self, widget, data = None ):
        
        self.opticalFlowBlockHeight = self.adjBlockHeight.get_value()
        if self.lastImage != None:
            self.setupMarkerBuffer()
    
    #---------------------------------------------------------------------------
    def onMenuItemNewActivate( self, widget ):
        self.filename = None
        self.markerBuffer = None
    
    #---------------------------------------------------------------------------
    def onMenuItemOpenActivate( self, widget ):
        
        if self.lastImage == None:
            return  # Can't load until we have an image
        
        filename = self.chooseMarkerFile()
        
        if filename != None:
            markerFile = file( filename, "r" )
            markerBuffer = yaml.load( markerFile )
            
            if type( markerBuffer ) == list:
                markerBuffer = np.array( markerBuffer )
                if len( markerBuffer.shape ) == 2:
                    
                    blockWidth = self.lastImage.width / (markerBuffer.shape[ 1 ] + 1)
                    blockHeight = self.lastImage.height / (markerBuffer.shape[ 0 ] + 1)
                    
                    if blockHeight < self.adjBlockHeight.get_upper() \
                        and blockWidth < self.adjBlockWidth.get_upper():
                        
                        self.adjBlockWidth.set_value( blockWidth )
                        self.adjBlockHeight.set_value( blockHeight )
                        self.markerBuffer = markerBuffer
                        
                        # Successful so remember filename
                        self.filename = filename
                    else:
                        print "Error: Invalid block width or height, please fix with Glade"
            
                else:
                    print "Error: The data is not a 2D list"
            else:
                print "Error: The data is not a list"
    
    #---------------------------------------------------------------------------
    def saveData( self, filename ):
        
        outputFile = file( self.filename, "w" )
        yaml.dump( self.markerBuffer.tolist(), outputFile )
        outputFile.close()
    
    #---------------------------------------------------------------------------
    def onMenuItemSaveActivate( self, widget ):
        if self.filename == None:
            self.onMenuItemSaveAsActivate( widget )
        else:
            self.saveData( self.filename )
    
    #---------------------------------------------------------------------------
    def onMenuItemSaveAsActivate( self, widget ):
    
        filename = self.chooseMarkerFile()
        if filename != None:
        
            self.filename = filename
            self.saveData( self.filename )
    
    #---------------------------------------------------------------------------
    def onMenuItemQuitActivate( self, widget ):
        self.onWinMainDestroy( widget )
    
    #---------------------------------------------------------------------------
    def setMarker( self, widget, data, on ):
        
        if self.markerBuffer != None:
            
            imgRect = self.getImageRectangleInWidget( widget,
                self.cameraImagePixBuf.get_width(), self.cameraImagePixBuf.get_height() )
        
            markerX = int( ( data.x - imgRect.x )/self.opticalFlowBlockWidth )
            markerY = int( ( data.y - imgRect.y )/self.opticalFlowBlockHeight )
            
            if markerY >= 0 and markerY < self.markerBuffer.shape[ 0 ]\
                and markerX >= 0 and markerX < self.markerBuffer.shape[ 1 ]:
                    
                self.markerBuffer[ markerY, markerX ] = on
    
    #---------------------------------------------------------------------------
    def onDwgCameraImageButtonPressEvent( self, widget, data ):
        
        if data.button == 1:
            self.setMarker( widget, data, True )
        else:
            self.setMarker( widget, data, False )
            
    #---------------------------------------------------------------------------
    def onDwgCameraImageMotionNotifyEvent( self, widget, data ):
        
        self.setMarker( widget, data, True )
    
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
              
            # Draw an overlay to show active marker squares
            if self.markerBuffer != None:
                    
                graphicsContext = widget.window.new_gc()
                graphicsContext.set_rgb_fg_color( gtk.gdk.Color( 65535, 65535, 0 ) )
                
                blockY = imgRect.y
                for y in range( self.markerBuffer.shape[ 0 ] ):
                
                    blockX = imgRect.x
                    for x in range( self.markerBuffer.shape[ 1 ] ):
                        
                        if self.markerBuffer[ y, x ]:
                            points = [ (blockX+int((i*2)%self.opticalFlowBlockWidth), blockY+2*int((i*2)/self.opticalFlowBlockWidth)) \
                                for i in range( int(self.opticalFlowBlockWidth*self.opticalFlowBlockHeight/4) ) ]
                                
                            widget.window.draw_points( graphicsContext, points )
                            
                        blockX += self.opticalFlowBlockWidth
                        
                    blockY += self.opticalFlowBlockHeight

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
    def setupMarkerBuffer( self ):
        
        if self.lastImage != None:
            
            opticalFlowBufferWidth = int((self.lastImage.width - self.opticalFlowBlockWidth)/self.opticalFlowBlockWidth)
            opticalFlowBufferHeight = int((self.lastImage.height - self.opticalFlowBlockHeight)/self.opticalFlowBlockHeight)
                        
            if opticalFlowBufferWidth > 0 and opticalFlowBufferHeight > 0:
                
                self.markerBuffer = np.zeros( 
                    shape=(opticalFlowBufferHeight,opticalFlowBufferWidth),
                    dtype=np.bool )

    #---------------------------------------------------------------------------
    def update( self ):

        lastTime = time.clock()

        while 1:
            
            curTime = time.clock()
            
                
            yield True
            
        yield False
        
        
#-------------------------------------------------------------------------------
if __name__ == "__main__":

    mainWindow = MainWindow()
    mainWindow.main()
