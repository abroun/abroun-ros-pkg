#! /usr/bin/python

# ROS imports
import roslib
roslib.load_manifest( 'object_detector' )
import rospy
import rosbag

import sys
import math
import os.path
import time
import threading

import numpy as np
import cv
import scipy.ndimage

import yaml
import pygtk
pygtk.require('2.0')
import gtk
import gobject

from abroun_gtk_gui.Display import Display
from MotionDetectionFilter import MotionDetectionFilter
from ImageFlowFilter import ImageFlowFilter

#-------------------------------------------------------------------------------
class MainWindow:
 
    #---------------------------------------------------------------------------
    def __init__( self ):
    
        self.scriptPath = os.path.dirname( __file__ )
        self.targetImageGray = None
        self.templateImageGray = None
        self.imageFlowFilter = ImageFlowFilter()
            
        # Setup the GUI        
        builder = gtk.Builder()
        builder.add_from_file( self.scriptPath + "/GUI/AlignmentExplorer.glade" )
        
        self.window = builder.get_object( "winMain" )   
        self.adjDisplacementX = builder.get_object( "adjDisplacementX" )
        self.adjDisplacementY = builder.get_object( "adjDisplacementY" )
        self.lblSSDDisplay = builder.get_object( "lblSSDDisplay" )
        self.lblTargetName = builder.get_object( "lblTargetName" )
        self.lblTemplateName = builder.get_object( "lblTemplateName" )

        dwgTargetImage = builder.get_object( "dwgTargetImage" )
        dwgMergedImage = builder.get_object( "dwgMergedImage" )
        dwgTemplateImage = builder.get_object( "dwgTemplateImage" )
        dwgErrorImage = builder.get_object( "dwgErrorImage" )
        dwgSubtractImage = builder.get_object( "dwgSubtractImage" )
        self.dwgTargetImageDisplay = Display( dwgTargetImage )
        self.dwgMergedImageDisplay = Display( dwgMergedImage )
        self.dwgTemplateImageDisplay = Display( dwgTemplateImage )
        self.dwgErrorImageDisplay = Display( dwgErrorImage )
        self.dwgSubtractImageDisplay = Display( dwgSubtractImage )
        
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
    def mergeImages( self ):
        
        if self.targetImageGray == None:
            # Nothing to do
            return
        
        # Create a transformed version of the template image
        transformedImage = scipy.ndimage.interpolation.shift( 
            self.templateImageGray, 
            ( self.adjDisplacementY.get_value(), self.adjDisplacementX.get_value() ) )
        
        # Create a composite image using the target image for the red channel
        # and the template image for the green channel. This should show 
        # matched pixels as yellow
        width = self.targetImageGray.shape[ 1 ]
        height = self.targetImageGray.shape[ 0 ]
        mergedImage = np.zeros( ( height, width, 3 ), dtype=np.uint8 )
        mergedImage[ :, :, 0 ] = self.targetImageGray
        mergedImage[ :, :, 1 ] = transformedImage
        
        # Display the merged image
        self.dwgMergedImageDisplay.setImageFromNumpyArray( mergedImage )
        
        # Calculate and display the Sum of Squared Differences (SSD) between the 2 images
        SSDValues = np.square( 
            transformedImage.astype( np.int32 ) - self.targetImageGray.astype( np.int32 ) )
        EPSILON = 128
        SSDValues[ SSDValues <= EPSILON*EPSILON ] = 0
            
        transformSSD = np.sum( SSDValues )
        self.lblSSDDisplay.set_text( str( transformSSD ) )
        
        # Display the error as a bitmap
        self.dwgErrorImageDisplay.setImageFromNumpyArray( 
            np.sqrt( SSDValues ).astype( np.uint8 ) )
            
        # Subtract the aligned template image from the target image and display
        #cv.Dilate( transformedImage, transformedImage )
        transformedImage[ transformedImage > 0 ] = 255
        subtractImage = self.targetImageGray.astype( np.int32 ) - transformedImage.astype( np.int32 )
        subtractImage[ subtractImage < 0 ] = 0
        self.dwgSubtractImageDisplay.setImageFromNumpyArray( subtractImage.astype( np.uint8 ) )

    #---------------------------------------------------------------------------
    def chooseImageFile( self ):
        
        result = None
        
        dialog = gtk.FileChooserDialog(
            title="Choose Image File",
            action=gtk.FILE_CHOOSER_ACTION_SAVE,
            buttons=(gtk.STOCK_CANCEL, gtk.RESPONSE_REJECT,
                      gtk.STOCK_OK, gtk.RESPONSE_ACCEPT) )

        dialog.set_current_folder( self.scriptPath + "/../../test_data" )
            
        filter = gtk.FileFilter()
        filter.add_pattern( "*.png" )
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
    def onMenuItemOpenTargetActivate( self, widget ):
                
        filename = self.chooseImageFile()
        
        if filename != None:
            # Load in the target image and convert to grayscale
            imageCV = cv.LoadImageM( filename )
            self.targetImageGray = np.ndarray( ( imageCV.height, imageCV.width ), dtype=np.uint8 )
            cv.CvtColor( imageCV, self.targetImageGray, cv.CV_BGR2GRAY )
            
            # Display the image
            self.dwgTargetImageDisplay.setImageFromNumpyArray( self.targetImageGray )
            self.lblTargetName.set_text( os.path.split( filename )[ 1 ] )
            
            # Clear the template image
            self.templateImageGray = np.zeros( self.targetImageGray.shape, dtype=np.uint8 )
            self.dwgTemplateImageDisplay.setImageFromNumpyArray( self.templateImageGray )
            self.lblTemplateName.set_text( "" )
            
            # Merge the images
            self.mergeImages()
            
    #---------------------------------------------------------------------------
    def onMenuItemOpenTemplateActivate( self, widget ):
        
        if self.targetImageGray == None:
            print "Error: Must load target image first"
            return
        
        filename = self.chooseImageFile()
        
        if filename != None:
            # Load in the template image 
            imageCV = cv.LoadImageM( filename )
            
            # Check that it has the same dimensions as the target image
            if imageCV.width != self.targetImageGray.shape[ 1 ] \
                or imageCV.height != self.targetImageGray.shape[ 0 ]:
                    
                print "Error: The template image must have the same dimensions as the target image"
                return
            
            # Convert to grayscale and display
            self.templateImageGray = np.ndarray( ( imageCV.height, imageCV.width ), dtype=np.uint8 )
            cv.CvtColor( imageCV, self.templateImageGray, cv.CV_BGR2GRAY )
            self.dwgTemplateImageDisplay.setImageFromNumpyArray( self.templateImageGray )
            self.lblTemplateName.set_text( os.path.split( filename )[ 1 ] )

            # Merge the images
            self.mergeImages()
    
    #---------------------------------------------------------------------------
    def onMenuItemQuitActivate( self, widget ):
        self.onWinMainDestroy( widget )
       
    #---------------------------------------------------------------------------
    def onAdjDisplacementXValueChanged( self, widget ):
        self.mergeImages()
    
    #---------------------------------------------------------------------------
    def onAdjDisplacementYValueChanged( self, widget ):
        self.mergeImages()
        
    #---------------------------------------------------------------------------
    def onBtnAutoAlignClicked( self, widget ):
        
        if self.targetImageGray == None or self.templateImageGray == None:
            # Nothing to do
            return
        
        # Align the images
        ( transX, transY, rotationAngle, newImage ) = \
            self.imageFlowFilter.calcImageFlow( self.targetImageGray, self.templateImageGray )
        
        # Display the x and y displacements
        self.adjDisplacementX.set_value( transX )
        self.adjDisplacementY.set_value( transY )
        
        # Merge the images
        #self.mergeImages()
    
    #---------------------------------------------------------------------------
    def onDwgTargetImageExposeEvent( self, widget, data ):
        
        self.dwgTargetImageDisplay.drawPixBufToDrawingArea( data.area )
    
    #---------------------------------------------------------------------------
    def onDwgMergedImageExposeEvent( self, widget, data ):
        
        self.dwgMergedImageDisplay.drawPixBufToDrawingArea( data.area )
    
    #---------------------------------------------------------------------------
    def onDwgTemplateImageExposeEvent( self, widget, data ):
        
        self.dwgTemplateImageDisplay.drawPixBufToDrawingArea( data.area )
    
    #---------------------------------------------------------------------------
    def onDwgErrorImageExposeEvent( self, widget, data ):
        
        self.dwgErrorImageDisplay.drawPixBufToDrawingArea( data.area )
        
    #---------------------------------------------------------------------------
    def onDwgSubtractImageExposeEvent( self, widget, data ):
        
        self.dwgSubtractImageDisplay.drawPixBufToDrawingArea( data.area )
        
    #---------------------------------------------------------------------------
    def update( self ):

        lastTime = time.clock()

        while 1:
            
            curTime = time.clock()
            #print "Processing image", framIdx
                
            yield True
            
        yield False
        
#-------------------------------------------------------------------------------
if __name__ == "__main__":

    mainWindow = MainWindow()
    mainWindow.main()
