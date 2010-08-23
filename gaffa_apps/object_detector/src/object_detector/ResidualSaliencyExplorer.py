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
 
    #---------------------------------------------------------------------------
    def __init__( self ):
    
        self.scriptPath = os.path.dirname( __file__ )
        self.cameraImagePixBuf = None
        self.saliencyMapPixBuf = None
        self.curImage = None
        self.curSaliencyMap = None
        self.filename = None
            
        # Connect to the robot via ROS
        rospy.init_node( 'ResidualSaliencyExplorer', anonymous=True )
        
        self.cameraImageTopic = rospy.Subscriber( "/camera/image", 
            sensor_msgs.msg.Image, self.cameraImageCallback )
            
        self.saliencyMapWidth = self.DEFAULT_SALIENCY_MAP_WIDTH
        self.saliencyMapHeight = self.DEFAULT_SALIENCY_MAP_HEIGHT
            
        # Setup the GUI        
        builder = gtk.Builder()
        builder.add_from_file( self.scriptPath + "/GUI/ResidualSaliencyExplorer.glade" )
        
        self.window = builder.get_object( "winMain" )   
        self.dwgCameraImage = builder.get_object( "dwgCameraImage" )
        self.dwgSaliencyMap = builder.get_object( "dwgSaliencyMap" )
        self.checkGetImageFromCamera = builder.get_object( "checkGetImageFromCamera" )
        self.adjSaliencyMapWidth = builder.get_object( "adjSaliencyMapWidth" )
        self.adjSaliencyMapHeight = builder.get_object( "adjSaliencyMapHeight" )
        
        # Set default values
        self.checkGetImageFromCamera.set_active( True )
        self.adjSaliencyMapWidth.set_value( self.saliencyMapWidth )
        self.adjSaliencyMapHeight.set_value( self.saliencyMapHeight )
        
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
    def cameraImageCallback( self, rosImage ):
        
        if not self.checkGetImageFromCamera.get_active():
            return  # Don't process the image if we're not going to show it
        
        if rosImage.encoding == "rgb8" or rosImage.encoding == "bgr8":
            
            # Create an OpenCV image to process the data
            cvImage = cv.CreateImageHeader( ( rosImage.width, rosImage.height ), cv.IPL_DEPTH_8U, 3 )
            cv.SetData( cvImage, rosImage.data, rosImage.step )
            
            self.processImage( cvImage )
            
        else:
            rospy.logerr( "Unhandled image encoding - " + rosImage.encoding )
            
    #---------------------------------------------------------------------------
    def processImage( self, cvImage ):
        
        # Save the image
        self.curImage = cvImage
        
        # Display the image
        self.cameraImagePixBuf = gtk.gdk.pixbuf_new_from_data( 
            cvImage.tostring(), 
            gtk.gdk.COLORSPACE_RGB,
            False,
            8,
            cvImage.width,
            cvImage.height,
            cvImage.width*3 )
            
        # Resize the drawing area if necessary
        if self.dwgCameraImage.get_size_request() != ( cvImage.width, cvImage.height ):
            self.dwgCameraImage.set_size_request( cvImage.width, cvImage.height )

        self.dwgCameraImage.queue_draw()

        # Now update the saliency map
        self.updateSaliencyMap()
    
    #---------------------------------------------------------------------------
    def numpySaliency( self, scaledImageGray ):
        
        # Convert the image to the complex frequency domain
        npImageGray = np.array( scaledImageGray )
        imageSpectrum = np.fft.fft2( npImageGray )
        
        phaseAngle = np.angle( imageSpectrum )
        
        imageSpectrum.real = np.cos( phaseAngle )
        imageSpectrum.imag = np.sin( phaseAngle )
        
        # Reconstruct the image using just the phase angle
        newImage = np.fft.ifft2( imageSpectrum ).real
        
        # Square the image as we are interpreting it as an error...
        newImage = np.square( newImage )
        
        # Smmoth and normalise the image
        newImage = filters.gaussian_filter( newImage, 4.0, mode='constant' )
        minVal = np.amin( newImage )
        maxVal = np.amax( newImage )
        newImage = np.multiply( np.subtract( newImage, minVal ), 255.0/maxVal )
        
        saliencyMap = np.array( newImage, dtype=np.uint8 )
        
        return saliencyMap
    
    #---------------------------------------------------------------------------
    def opencvSaliency( self, scaledImageGray ):
        
        cvImageGray = cv.CreateMat( scaledImageGray.height, scaledImageGray.width, cv.CV_32FC1 )
        cv.Convert( scaledImageGray, cvImageGray )
        
        src = cvImageGray
        dftWidth = cv.GetOptimalDFTSize( src.width - 1 )
        dftHeight = cv.GetOptimalDFTSize( src.height - 1 )

        real = cv.CreateMat( dftHeight, dftWidth, cv.CV_32FC1 )
        imaginary = cv.CreateMat( dftHeight, dftWidth, cv.CV_32FC1 )
        dft = cv.CreateMat( dftHeight, dftWidth, cv.CV_32FC2 )
    
        tmp = cv.GetSubRect( real, ( 0, 0, src.width, src.height ) )
        cv.Copy( src, tmp )
        cv.Zero( imaginary )
    
        cv.Merge( real, imaginary, None, None, dft )
        # do the fft
        cv.DFT( dft, dft, cv.CV_DXT_FORWARD, src.height )
        cv.Split( dft, real, imaginary, None, None )
    
        cv.CartToPolar( real, imaginary, real, imaginary, 0 )
        cv.Log( real, real )
        filtered = cv.CreateMat( dftHeight, dftWidth, cv.CV_32FC1 )
        cv.Copy( real, filtered )
        cv.Smooth( filtered, filtered, cv.CV_BLUR )
    
        cv.Sub( real, filtered, real, None )
        cv.Exp( real, real )
        cv.PolarToCart( real, imaginary, real, imaginary,0 )
        #cv.PolarToCart( np.ones( shape=(dftHeight,dftWidth), dtype=np.float32 ), imaginary, real, imaginary,0 )
    
        # do inverse fourier transform
        cv.Merge( real, imaginary, None, None, dft )
        cv.DFT( dft, dft, cv.CV_DXT_INV_SCALE, src.height )
        cv.Split( dft, real, imaginary, None, None )

        # get magnitude
        cv.CartToPolar( real, imaginary, real, None, 0 );
        cv.Pow( real, real, 2.0 )
    
        FILTER_RAD = 3
        IPL_BORDER_CONSTANT = 0

        sfiltered = cv.CreateMat( real.height+FILTER_RAD*2, real.width+FILTER_RAD*2, cv.CV_32FC1 )
        cv.CopyMakeBorder( real, sfiltered, ( FILTER_RAD, FILTER_RAD ), IPL_BORDER_CONSTANT )

        cv.Smooth( sfiltered, sfiltered, cv.CV_GAUSSIAN, 2*FILTER_RAD + 1 )
    
        ( min, max, minLoc, maxLoc ) =  cv.MinMaxLoc( sfiltered )
        cv.ConvertScale( sfiltered, sfiltered, 1/(max-min), -min/(max-min) )
    
        # copy result to output image
        tmp = cv.GetSubRect( sfiltered, ( FILTER_RAD, FILTER_RAD, src.width, src.height ) )
        cv.Copy( tmp, cvImageGray )
 
    #cvReleaseMat(&sfiltered);
    #cvReleaseMat(&real);
    #cvReleaseMat(&filtered);
    #cvReleaseMat(&imaginary);
    #cvReleaseMat(&dft);
        
        
        
        saliencyMap = np.array( 255.0*np.array( cvImageGray ), dtype=np.uint8 )
        return saliencyMap
        
    #---------------------------------------------------------------------------
    def tunedSaliency( self, imageGray ):
        
        smoothingFilter = np.array( 
            [[ 0.0, 0.0, 1.0, 0.0, 0.0 ],
            [ 0.0, 1.0, 4.0, 1.0, 0.0 ],
            [ 1.0, 4.0, 6.0, 4.0, 1.0 ],
            [ 0.0, 1.0, 4.0, 1.0, 0.0 ],
            [ 0.0, 0.0, 1.0, 0.0, 0.0 ]] )
        smoothingFilter = np.divide( smoothingFilter, 16.0 )
        
        smoothedImage = filters.convolve( imageGray, smoothingFilter, mode='constant' )
        averageValue = np.mean( imageGray )
        
        saliencyMap = np.abs( np.subtract( averageValue, smoothedImage ) )
        saliencyMap = np.array( saliencyMap, dtype=np.uint8 )
        
        return saliencyMap
        
    #---------------------------------------------------------------------------
    @printTiming
    def updateSaliencyMap( self ):
        
        if self.curImage == None:
            return
            
        # Convert the image to gray scale
        imageGray = cv.CreateMat( self.curImage.height, self.curImage.width, cv.CV_8UC1 )
        cv.CvtColor( self.curImage, imageGray, cv.CV_RGB2GRAY )
        
        # Scale the image, this determines the size of feature which is looked for
        if self.curImage.height < self.curImage.width:
            scale = float( self.saliencyMapWidth ) / float( self.curImage.height )
        else:
            scale = float( self.saliencyMapWidth ) / float( self.curImage.width )
        
        scaledWidth = self.saliencyMapWidth #int( self.curImage.width*scale )
        scaledHeight = self.saliencyMapWidth #int( self.curImage.height*scale )
        scaledImageGray = cv.CreateMat( scaledHeight, scaledWidth, cv.CV_8UC1 )
        cv.Resize( imageGray, scaledImageGray )
        
        # Calculate the saliency map according to Hou and Zhang
        saliencyMap = self.numpySaliency( scaledImageGray )
        #saliencyMap = self.opencvSaliency( scaledImageGray )
        #saliencyMap = self.tunedSaliency( imageGray )
        
        self.curSaliencyMap = saliencyMap
        
        # Display the saliency map - it needs to be converted to RGB for this
        cv.Resize( saliencyMap, imageGray )
        saliencyMapRGB = cv.CreateImage( ( imageGray.width, imageGray.height ), 
            cv.IPL_DEPTH_8U, 3 )
        cv.CvtColor( imageGray, saliencyMapRGB, cv.CV_GRAY2RGB )
                
        self.saliencyMapPixBuf = gtk.gdk.pixbuf_new_from_data( 
            saliencyMapRGB.tostring(), 
            gtk.gdk.COLORSPACE_RGB,
            False,
            8,
            saliencyMapRGB.width,
            saliencyMapRGB.height,
            saliencyMapRGB.width*3 )
            
        # Resize the drawing area if necessary
        if self.dwgSaliencyMap.get_size_request() != ( saliencyMapRGB.width, saliencyMapRGB.height ):
            self.dwgSaliencyMap.set_size_request( saliencyMapRGB.width, saliencyMapRGB.height )

        self.dwgSaliencyMap.queue_draw()

    #---------------------------------------------------------------------------
    def onAdjSaliencyMapWidthValueChanged( self, widget, data = None ):
        
        self.saliencyMapWidth = self.adjSaliencyMapWidth.get_value()
        if not self.checkGetImageFromCamera.get_active():
            self.updateSaliencyMap()
            
    #---------------------------------------------------------------------------
    def onAdjSaliencyMapHeightValueChanged( self, widget, data = None ):
        
        self.saliencyMapHeight = self.adjSaliencyMapHeight.get_value()
        if not self.checkGetImageFromCamera.get_active():
            self.updateSaliencyMap()
    
    #---------------------------------------------------------------------------
    def onMenuItemLoadImageActivate( self, widget ):
        
        filename = self.chooseImageFile()
        
        if filename != None:
            cvImage = cv.LoadImage( filename )
            cv.CvtColor( cvImage, cvImage, cv.CV_BGR2RGB )
            
            self.checkGetImageFromCamera.set_active( False )
            self.processImage( cvImage )
    
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
    def onDwgSaliencyMapExposeEvent( self, widget, data = None ):
        
        if self.saliencyMapPixBuf != None:
            
            imgRect = self.getImageRectangleInWidget( widget,
                self.saliencyMapPixBuf.get_width(), self.saliencyMapPixBuf.get_height() )
                
            imgOffsetX = imgRect.x
            imgOffsetY = imgRect.y
                
            # Get the total area that needs to be redrawn
            imgRect = imgRect.intersect( data.area )
        
            srcX = imgRect.x - imgOffsetX
            srcY = imgRect.y - imgOffsetY
           
            widget.window.draw_pixbuf( widget.get_style().fg_gc[ gtk.STATE_NORMAL ],
                self.saliencyMapPixBuf, srcX, srcY, 
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
