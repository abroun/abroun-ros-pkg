import pygtk
pygtk.require('2.0')
import gtk
import gobject

import cv
import numpy as np

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
        
        if cvMtx.channels == 1:
            # Convert grayscale image to RGB
            cvMtxRGB = cv.CreateMat( height, width, cv.CV_8UC3 )
            cv.CvtColor( cvMtx, cvMtxRGB, cv.CV_GRAY2RGB )
            data = cvMtxRGB.tostring()
        else:
            data = cvMtx.tostring()
        
        self.setImage( width, height, data )
        
    #---------------------------------------------------------------------------
    def setImageFromNumpyArray( self, imageArray ):
        
        width = imageArray.shape[ 1 ]
        height = imageArray.shape[ 0 ]
        
        if len( imageArray.shape ) < 3 or imageArray.shape[ 2 ] == 1:
            # Convert grayscale image to RGB
            imageArrayRGB = np.ndarray( ( height, width, 3 ), dtype=np.uint8 )
            imageArrayRGB[ :, :, 0 ] = imageArray
            imageArrayRGB[ :, :, 1 ] = imageArray
            imageArrayRGB[ :, :, 2 ] = imageArray
            data = imageArrayRGB.tostring()
        else:
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
    def clear( self ):
        imgRect = self.getImageRectangleInWidget( self.drawingArea )
        if imgRect != None:
            self.setImageFromNumpyArray( np.zeros( ( imgRect.height, imgRect.width ), dtype=np.uint8 ) )
        
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