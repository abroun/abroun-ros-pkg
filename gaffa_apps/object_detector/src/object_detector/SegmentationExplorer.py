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
class Display:
    '''A combination of a pixBuf and the drawing area that is used to
       to display it'''
    
    #---------------------------------------------------------------------------
    def __init__( self, drawingArea ):
        
        self.drawingArea = drawingArea
        self.pixBuf = None
        
    #---------------------------------------------------------------------------
    def setImageFromOpenCVMatrix( self, cvMtx ):
        
        width = cvMtx.width
        height = cvMtx.height
        data = cvMtx.tostring()
        
        self.setImage( width, height, data )
        
    #---------------------------------------------------------------------------
    def setImageFromNumpyArray( self, imageArray ):
        
        width = imageArray.shape[ 1 ]
        height = imageArray.shape[ 0 ]
        data = imageArray.tostring()
        
        self.setImage( width, height, data )
        
    #---------------------------------------------------------------------------
    def setImage( self, width, height, data ):
        
        # Display the image
        self.pixBuf = gtk.gdk.pixbuf_new_from_data( 
            data, 
            gtk.gdk.COLORSPACE_RGB,
            False,
            8,
            width,
            height,
            width*3 )
            
        # Resize the drawing area if necessary
        if self.drawingArea.get_size_request() != ( width, height ):
            self.drawingArea.set_size_request( width, height )

        self.drawingArea.queue_draw()
        
    #---------------------------------------------------------------------------
    def queueDraw( self ):
        self.drawingArea.queue_draw()
        
    #---------------------------------------------------------------------------
    def drawPixBufToDrawingArea( self, redrawArea ):
        '''Draws the PixBuf to the drawing area. If successful then it returns
           the image rectange that was drawn to so that futher drawing can be
           done. Otherwise it returns None'''
        
        imgRect = self.getImageRectangleInWidget( self.drawingArea )
        if imgRect != None:
            imgOffsetX = imgRect.x
            imgOffsetY = imgRect.y
                
            # Get the total area that needs to be redrawn
            redrawRect = imgRect.intersect( redrawArea )
        
            srcX = redrawRect.x - imgOffsetX
            srcY = redrawRect.y - imgOffsetY
           
            self.drawingArea.window.draw_pixbuf( 
                self.drawingArea.get_style().fg_gc[ gtk.STATE_NORMAL ],
                self.pixBuf, srcX, srcY, 
                redrawRect.x, redrawRect.y, redrawRect.width, redrawRect.height )
        
        return imgRect
         
    #---------------------------------------------------------------------------
    def getImageRectangleInWidget( self, widget ):
        '''Returns a rectangle for drawing the contents of the PixBuf centred
           in the middle of the given widget. If no PixBuf has been set yet then
           this routine returns None'''
        
        if self.pixBuf == None:
            return None
        else:
            imageWidth = self.pixBuf.get_width()
            imageHeight = self.pixBuf.get_height()
        
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

#-------------------------------------------------------------------------------
class MainWindow:
    
    BRUSH_COLOUR = np.array( [ 255, 255, 0 ], dtype=np.uint8 )
 
    #---------------------------------------------------------------------------
    def __init__( self ):
    
        self.scriptPath = os.path.dirname( __file__ )
        self.image = None
        self.maskArray = None
        self.filename = None
            
        # Setup the GUI        
        builder = gtk.Builder()
        builder.add_from_file( self.scriptPath + "/GUI/SegmentationExplorer.glade" )
        
        self.window = builder.get_object( "winMain" )   
        dwgImage = builder.get_object( "dwgImage" )
        dwgSegmentation = builder.get_object( "dwgSegmentation" )
        self.adjBrushSize = builder.get_object( "adjBrushSize" )
        self.comboBrushType = builder.get_object( "comboBrushType" )
        
        self.dwgImageDisplay = Display( dwgImage )
        self.dwgSegmentationDisplay = Display( dwgSegmentation )
        
        # Set default values
        self.adjBrushSize.set_value( 1 )
        self.makeBrush()
        
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
    def makeBrush( self ):
        
        brushSize = self.adjBrushSize.get_value()
        brushShape = ( brushSize, brushSize, 3 )
        #self.brush
        
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
            self.image = cv.LoadImageM( filename )
            cv.CvtColor( self.image, self.image, cv.CV_BGR2RGB )
            
            # Create a mask and blank segmentation that matches the size of the image
            self.maskArray = np.zeros( ( self.image.height, self.image.width, 3 ), np.uint8 )
            self.segmentation = np.zeros( ( self.image.height, self.image.width, 3 ), np.uint8 )
            
            GC_INIT_WITH_RECT = 0
            
            otherMask = cv.CreateMat( self.image.height, self.image.width, cv.CV_8UC1 )
            cv.SetZero( otherMask )
            fgModel = cv.CreateMat( 1, 5*13, cv.CV_32FC1 )
            bgModel = cv.CreateMat( 1, 5*13, cv.CV_32FC1 )
            
            cv.GrabCut( self.image, otherMask, (192,125,120,190), fgModel, bgModel, 4, GC_INIT_WITH_RECT )
            
            npOtherMask = np.array( otherMask )
            self.segmentation[ :, :, 0 ] = otherMask
            self.segmentation[ :, :, 1 ] = otherMask
            self.segmentation[ :, :, 2 ] = otherMask
            #print self.segmentation[ self.segmentation != 0 ].shape
            #self.segmentation[ self.segmentation == 3 ] = 255
            segmentedImage = np.copy( self.image )
            segmentedImage[ self.segmentation != 3 ] = 0
            
            self.dwgImageDisplay.setImageFromOpenCVMatrix( self.image )
            self.dwgSegmentationDisplay.setImageFromNumpyArray( segmentedImage )
        
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
    def onBtnClearMaskClicked( self, widget ):
        if self.image != None:
            self.maskArray = np.zeros( ( self.image.height, self.image.width, 3 ), np.uint8 )
            self.dwgImageDisplay.queueDraw()
    
    #---------------------------------------------------------------------------
    def onBtnSegmentClicked( self, widget ):
        pass
    
    #---------------------------------------------------------------------------
    def onComboBrushTypeChanged( self, widget ):
        print self.comboBrushType.get_active_text()
    
    #---------------------------------------------------------------------------
    def onDwgImageButtonPressEvent( self, widget, data ):
        
        if data.button == 1:
            self.setMarker( widget, data, True )
        else:
            self.setMarker( widget, data, False )
            
    #---------------------------------------------------------------------------
    def onDwgImageMotionNotifyEvent( self, widget, data ):
        
        self.setMarker( widget, data, True )
    
    #---------------------------------------------------------------------------
    def onDwgImageExposeEvent( self, widget, data ):
        
        imgRect = self.dwgImageDisplay.drawPixBufToDrawingArea( data.area )
        
        if imgRect != None:
            
            imgRect = imgRect.intersect( data.area )
              
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
        
        self.dwgSegmentationDisplay.drawPixBufToDrawingArea( data.area )    

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
