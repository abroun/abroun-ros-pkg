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
import cv

import yaml
import pygtk
pygtk.require('2.0')
import gtk
import gobject

#-------------------------------------------------------------------------------
class MainWindow:
 
    #---------------------------------------------------------------------------
    def __init__( self ):
    
        self.scriptPath = os.path.dirname( __file__ )
        self.imagePixBuf = None
        self.segmentationPixBuf = None
        self.lastImage = None
        self.markerBuffer = None
        self.filename = None
            
        # Setup the GUI        
        builder = gtk.Builder()
        builder.add_from_file( self.scriptPath + "/GUI/SegmentationExplorer.glade" )
        
        self.window = builder.get_object( "winMain" )   
        self.dwgImage = builder.get_object( "dwgImage" )
        self.dwgSegmentation = builder.get_object( "dwgSegmentation" )
        self.adjBrushSize = builder.get_object( "adjBrushSize" )
        self.comboBrushType = builder.get_object( "comboBrushType" )
        
        # Set default values
        self.adjBrushSize.set_value( 1 )
        
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
    def chooseImageFile( self ):
        
        result = None
        
        dialog = gtk.FileChooserDialog(
            title="Choose Image File",
            action=gtk.FILE_CHOOSER_ACTION_SAVE,
            buttons=(gtk.STOCK_CANCEL, gtk.RESPONSE_REJECT,
                      gtk.STOCK_OK, gtk.RESPONSE_ACCEPT) )

        dialog.set_current_folder( self.scriptPath + "/../../test_data/saliency" )
            
        filter = gtk.FileFilter()
        filter.add_pattern( "*.png" )
        filter.add_pattern( "*.bmp" )
        filter.add_pattern( "*.jpg" )
        filter.set_name( "Image Files" )
        dialog.add_filter( filter )
        dialog.set_filter( filter )
            
        result = dialog.run()

        if result == gtk.RESPONSE_ACCEPT:
            result = dialog.get_filename()

        dialog.destroy()
        
        return result
    
    #---------------------------------------------------------------------------
    def onMenuItemOpenImageActivate( self, widget ):
                
        filename = self.chooseImageFile()
        
        if filename != None:
            pass
        
            #markerBuffer = MarkerBuffer.loadMarkerBuffer( filename )
            #if markerBuffer != None:
            
                #blockWidth = self.lastImage.width / (markerBuffer.shape[ 1 ] + 1)
                #blockHeight = self.lastImage.height / (markerBuffer.shape[ 0 ] + 1)
                    
                #if blockHeight < self.adjBlockHeight.get_upper() \
                    #and blockWidth < self.adjBlockWidth.get_upper():
                    
                    #self.adjBlockWidth.set_value( blockWidth )
                    #self.adjBlockHeight.set_value( blockHeight )
                    #self.markerBuffer = markerBuffer
                        
                    ## Successful so remember filename
                    #self.filename = filename
                #else:
                    #print "Error: Invalid block width or height, please fix with Glade"            
    
    #---------------------------------------------------------------------------
    def onMenuItemQuitActivate( self, widget ):
        self.onWinMainDestroy( widget )
       
    #---------------------------------------------------------------------------
    def onBtnClearClicked( self, widget ):
        pass
    
    #---------------------------------------------------------------------------
    def onBtnSegmentClicked( self, widget ):
        pass
    
    #---------------------------------------------------------------------------
    def onComboBrushTypeChanged( self, widget ):
        print self.comboBrushType.get_active_text()
    
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
    def onDwgImageExposeEvent( self, widget, data = None ):
        
        if self.imagePixBuf != None:
            
            imgRect = self.getImageRectangleInWidget( widget,
                self.imagePixBuf.get_width(), self.imagePixBuf.get_height() )
                
            imgOffsetX = imgRect.x
            imgOffsetY = imgRect.y
                
            # Get the total area that needs to be redrawn
            imgRect = imgRect.intersect( data.area )
        
            srcX = imgRect.x - imgOffsetX
            srcY = imgRect.y - imgOffsetY
           
            widget.window.draw_pixbuf( widget.get_style().fg_gc[ gtk.STATE_NORMAL ],
                self.imagePixBuf, srcX, srcY, 
                imgRect.x, imgRect.y, imgRect.width, imgRect.height )
              
            # Draw an overlay to show the selected segmentation
            #if self.markerBuffer != None:
                    
                #graphicsContext = widget.window.new_gc()
                #graphicsContext.set_rgb_fg_color( gtk.gdk.Color( 65535, 65535, 0 ) )
                
                #blockY = imgRect.y
                #for y in range( self.markerBuffer.shape[ 0 ] ):
                
                    #blockX = imgRect.x
                    #for x in range( self.markerBuffer.shape[ 1 ] ):
                        
                        #if self.markerBuffer[ y, x ]:
                            #points = [ (blockX+int((i*2)%self.opticalFlowBlockWidth), blockY+2*int((i*2)/self.opticalFlowBlockWidth)) \
                                #for i in range( int(self.opticalFlowBlockWidth*self.opticalFlowBlockHeight/4) ) ]
                                
                            #widget.window.draw_points( graphicsContext, points )
                            
                        #blockX += self.opticalFlowBlockWidth
                        
                    #blockY += self.opticalFlowBlockHeight

    #---------------------------------------------------------------------------
    def onDwgSegmentationExposeEvent( self, widget, data = None ):
        
        if self.segmentationPixBuf != None:
            
            imgRect = self.getImageRectangleInWidget( widget,
                self.segmentationPixBuf.get_width(), self.segmentationPixBuf.get_height() )
                
            imgOffsetX = imgRect.x
            imgOffsetY = imgRect.y
                
            # Get the total area that needs to be redrawn
            imgRect = imgRect.intersect( data.area )
        
            srcX = imgRect.x - imgOffsetX
            srcY = imgRect.y - imgOffsetY
           
            widget.window.draw_pixbuf( widget.get_style().fg_gc[ gtk.STATE_NORMAL ],
                self.segmentationPixBuf, srcX, srcY, 
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

        lastTime = time.clock()

        while 1:
            
            curTime = time.clock()
            
                
            yield True
            
        yield False
        
        
#-------------------------------------------------------------------------------
if __name__ == "__main__":

    mainWindow = MainWindow()
    mainWindow.main()
