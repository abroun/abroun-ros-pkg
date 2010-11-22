#! /usr/bin/python
#-------------------------------------------------------------------------------
# Program for exploring the construction of a segmentation mask from motion
# images around the point of impact between the robot's gripper and an object
# of interest.
#-------------------------------------------------------------------------------

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

from abroun_gtk_gui.widgets import FilePath
from abroun_gtk_gui.Display import Display
from ImageFlowFilter import ImageFlowFilter
import PyBlobLib

#-------------------------------------------------------------------------------
class ImpactImageData:
    
    #---------------------------------------------------------------------------
    def __init__( self, filename = "", 
        displacementX = 0.0, displacementY = 0.0, active = False ):
        
        self.filename = filename
        self.displacementX = displacementX
        self.displacementY = displacementY
        self.active = active

#-------------------------------------------------------------------------------
class ImpactConfig( yaml.YAMLObject ):

    yaml_tag = u'!ImpactConfig'
    
    #---------------------------------------------------------------------------
    def __init__( self ):
        self.imageData = {}

#-------------------------------------------------------------------------------
class MainWindow:

    FOREGROUND_BRUSH_COLOUR = np.array( [ 255, 255, 0 ], dtype=np.uint8 )
    PROBABLY_FOREGROUND_BRUSH_COLOUR = np.array( [ 0, 255, 0 ], dtype=np.uint8 )
    BACKGROUND_BRUSH_COLOUR = np.array( [ 0, 0, 255 ], dtype=np.uint8 )
    
    # Classes of pixel in GrabCut algorithm
    GC_BGD = 0      # background
    GC_FGD = 1      # foreground
    GC_PR_BGD = 2   # most probably background
    GC_PR_FGD = 3   # most probably foreground 

    # GrabCut algorithm flags
    GC_INIT_WITH_RECT = 0
    GC_INIT_WITH_MASK = 1
    GC_EVAL = 2

    #---------------------------------------------------------------------------
    def __init__( self ):
    
        self.scriptPath = os.path.dirname( __file__ )
        
        self.accumulatorImage = None
        self.maskArray = None
        self.fillingImageDataUI = False
        self.handlingFilePath = False

        self.imageFlowFilter = ImageFlowFilter()
            
        # Setup the GUI        
        builder = gtk.Builder()
        builder.add_from_file( self.scriptPath + "/GUI/ImpactExplorer.glade" )
        
        self.window = builder.get_object( "winMain" )   
        self.comboCurImage = builder.get_object( "comboCurImage" )
        self.checkAddToMask = builder.get_object( "checkAddToMask" )
        self.adjDisplacementX = builder.get_object( "adjDisplacementX" )
        self.adjDisplacementY = builder.get_object( "adjDisplacementY" )
        self.filePathImage = builder.get_object( "filePathImage" )
        #self.lblSSDDisplay = builder.get_object( "lblSSDDisplay" )
        #self.lblTargetName = builder.get_object( "lblTargetName" )
        #self.lblTemplateName = builder.get_object( "lblTemplateName" )

        dwgCurImage = builder.get_object( "dwgCurImage" )
        dwgMergedImage = builder.get_object( "dwgMergedImage" )
        dwgImpactMotionImage = builder.get_object( "dwgImpactMotionImage" )
        dwgAccumulatorImage = builder.get_object( "dwgAccumulatorImage" )
        dwgMaskImage = builder.get_object( "dwgMaskImage" )
        dwgSegmentedImage = builder.get_object( "dwgSegmentedImage" )
        self.dwgCurImageDisplay = Display( dwgCurImage )
        self.dwgMergedImageDisplay = Display( dwgMergedImage )
        self.dwgImpactMotionImageDisplay = Display( dwgImpactMotionImage )
        self.dwgAccumulatorImageDisplay = Display( dwgAccumulatorImage )
        self.dwgMaskImageDisplay = Display( dwgMaskImage )
        self.dwgSegmentedImageDisplay = Display( dwgSegmentedImage )
        
        self.filePathImage.setOnFilenameChangedCallback( self.onFilePathImageChanged )
        builder.connect_signals( self )
        self.onMenuItemNewActivate( None ) # Create new config
        
        self.window.show()
        
    #---------------------------------------------------------------------------
    def onWinMainDestroy( self, widget, data = None ):  
        gtk.main_quit()
        
    #---------------------------------------------------------------------------   
    def main( self ):
        # All PyGTK applications must have a gtk.main(). Control ends here
        # and waits for an event to occur (like a key press or mouse event).
        gtk.main()
        
    ##---------------------------------------------------------------------------
    #def mergeImages( self ):
        
        #if self.targetImageGray == None:
            ## Nothing to do
            #return
        
        ## Create a transformed version of the template image
        #transformedImage = scipy.ndimage.interpolation.shift( 
            #self.templateImageGray, 
            #( self.adjDisplacementY.get_value(), self.adjDisplacementX.get_value() ) )
        
        ## Create a composite image using the target image for the red channel
        ## and the template image for the green channel. This should show 
        ## matched pixels as yellow
        #width = self.targetImageGray.shape[ 1 ]
        #height = self.targetImageGray.shape[ 0 ]
        #mergedImage = np.zeros( ( height, width, 3 ), dtype=np.uint8 )
        #mergedImage[ :, :, 0 ] = self.targetImageGray
        #mergedImage[ :, :, 1 ] = transformedImage
        
        ## Display the merged image
        #self.dwgMergedImageDisplay.setImageFromNumpyArray( mergedImage )
        
        ## Calculate and display the Sum of Squared Differences (SSD) between the 2 images
        #SSDValues = np.square( 
            #transformedImage.astype( np.int32 ) - self.targetImageGray.astype( np.int32 ) )
        #EPSILON = 128
        #SSDValues[ SSDValues <= EPSILON*EPSILON ] = 0
            
        #transformSSD = np.sum( SSDValues )
        #self.lblSSDDisplay.set_text( str( transformSSD ) )
        
        ## Display the error as a bitmap
        #self.dwgErrorImageDisplay.setImageFromNumpyArray( 
            #np.sqrt( SSDValues ).astype( np.uint8 ) )
            
        ## Subtract the aligned template image from the target image and display
        ##cv.Dilate( transformedImage, transformedImage )
        #transformedImage[ transformedImage > 0 ] = 255
        #subtractImage = self.targetImageGray.astype( np.int32 ) - transformedImage.astype( np.int32 )
        #subtractImage[ subtractImage < 0 ] = 0
        #self.dwgSubtractImageDisplay.setImageFromNumpyArray( subtractImage.astype( np.uint8 ) )
        
    #---------------------------------------------------------------------------
    def getCurImageName( self ):
         return self.comboCurImage.get_active_text()
    
    #---------------------------------------------------------------------------    
    def isImageGrayscale( self, imageName ):
        
        result = False
        if imageName.find( "PreMotion" ) == 0 \
            or imageName.find( "PostMotion" ) == 0 \
            or imageName == "ImpactMotion":
                
            result = True
        
        return result
        
    #---------------------------------------------------------------------------
    def getDataFromConfig( self ):
        pass

    #---------------------------------------------------------------------------
    def chooseConfigFile( self, action=gtk.FILE_CHOOSER_ACTION_OPEN, startFolder=None ):
        
        if startFolder == None:
            startFolder = self.scriptPath + "/../../test_data/impact_images"
            
        result = None
        
        dialog = gtk.FileChooserDialog(
            title="Choose Config File",
            action=action,
            buttons=(gtk.STOCK_CANCEL, gtk.RESPONSE_REJECT,
                      gtk.STOCK_OK, gtk.RESPONSE_ACCEPT) )

        dialog.set_current_folder( startFolder )
            
        filter = gtk.FileFilter()
        filter.add_pattern( "*.config" )
        filter.set_name( "Config Files" )
        dialog.add_filter( filter )
        dialog.set_filter( filter )
            
        dialogResult = dialog.run()

        if dialogResult == gtk.RESPONSE_ACCEPT:
            result = dialog.get_filename()
            
            # If we're saving make sure that the filename has an extension
            if action == gtk.FILE_CHOOSER_ACTION_SAVE \
                and os.path.splitext( result )[ 1 ] == "":
                
                result += ".config"

        dialog.destroy()
        
        return result
        
    #---------------------------------------------------------------------------
    def onMenuItemNewActivate( self, widget ):
        self.config = ImpactConfig()
        self.configFilename = None
        self.configDir = self.scriptPath + "/../../test_data/impact_images"
        self.filePathImage.setStartingFolder( self.configDir )
        
        self.images = {}    # Dictionary of NumPy arrays
        
        self.onComboCurImageChanged( self.comboCurImage )   # Update combo box
    
    #---------------------------------------------------------------------------
    def onMenuItemOpenConfigActivate( self, widget ):
                
        filename = self.chooseConfigFile()
        
        if filename != None:
            # Load in config file
            configFile = file( filename, "r" )
            self.config = yaml.load( configFile )
            self.configFilename = filename
            self.configDir = os.path.dirname( self.configFilename )
            self.filePathImage.setStartingFolder( self.configDir )
            
            # Load images
            self.images = {}
            for imageName in self.config.imageData.keys():
                image = self.loadImageFromData( imageName )
                self.images[ imageName ] = image
            
            self.onComboCurImageChanged( self.comboCurImage )   # Update combo box
    
    #---------------------------------------------------------------------------
    def onMenuItemSaveConfigActivate( self, widget ):
        if self.configFilename == None:
            self.onMenuItemSaveConfigAsActivate( widget )
        else:
            self.saveConfig( self.configFilename )
        
    #---------------------------------------------------------------------------
    def onMenuItemSaveConfigAsActivate( self, widget ):
        filename = self.chooseConfigFile( action=gtk.FILE_CHOOSER_ACTION_SAVE )
        
        if filename != None:
            self.saveConfig( filename )
    
    #---------------------------------------------------------------------------
    def onMenuItemQuitActivate( self, widget ):
        self.onWinMainDestroy( widget )
        
    #-------------------------------------------------------------------------------
    def saveConfig( self, filename ):

        newConfigDir = os.path.dirname( filename )

        # Make all file paths relative to the new configDir
        for imageName in self.config.imageData.keys():
            imageFilename = self.config.imageData[ imageName ].filename
            if imageFilename != "":
                self.config.imageData[ imageName ].filename = \
                    os.path.relpath( self.configDir + "/" + imageFilename, newConfigDir )

        outputFile = file( filename, "w" )
        yaml.dump( self.config, outputFile )
        outputFile.close()
        self.configFilename = filename
        self.configDir = newConfigDir
        self.filePathImage.setStartingFolder( self.configDir )
        
        self.onComboCurImageChanged( self.comboCurImage )   # Update combo box
       
    #---------------------------------------------------------------------------
    def onAdjDisplacementXValueChanged( self, widget ):
        if not self.fillingImageDataUI:
            curImageName = self.getCurImageName()
            imageData = self.config.imageData[ curImageName ]
            imageData.displacementX = self.adjDisplacementX.get_value()
            
            self.updateMergedImage()
    
    #---------------------------------------------------------------------------
    def onAdjDisplacementYValueChanged( self, widget ):
        if not self.fillingImageDataUI:
            curImageName = self.getCurImageName()
            imageData = self.config.imageData[ curImageName ]
            imageData.displacementY = self.adjDisplacementY.get_value()
            
            self.updateMergedImage()
        
    #---------------------------------------------------------------------------
    def onCheckAddToMaskToggled( self, widget ):
        if not self.fillingImageDataUI:
            curImageName = self.getCurImageName()
            imageData = self.config.imageData[ curImageName ]
            imageData.active = self.checkAddToMask.get_active()
            
            self.updateMergedImage()
        
    ##---------------------------------------------------------------------------
    #def onBtnAutoAlignClicked( self, widget ):
        
        #if self.targetImageGray == None or self.templateImageGray == None:
            ## Nothing to do
            #return
        
        ## Align the images
        #( transX, transY, rotationAngle, newImage ) = \
            #self.imageFlowFilter.calcImageFlow( self.targetImageGray, self.templateImageGray )
        
        ## Display the x and y displacements
        #self.adjDisplacementX.set_value( transX )
        #self.adjDisplacementY.set_value( transY )
        
        ## Merge the images
        ##self.mergeImages()
        
    #---------------------------------------------------------------------------
    def onComboCurImageChanged( self, widget ):
        
        # Make sure that the config has the current image
        curImageName = self.getCurImageName()
        if not curImageName in self.config.imageData:
            self.config.imageData[ curImageName ] = ImpactImageData()
        
        self.fillingImageDataUI = True
        
        # Get data from the current image to populate the data controls
        imageData = self.config.imageData[ curImageName ]
        self.checkAddToMask.set_active( imageData.active )
        self.adjDisplacementX.set_value( imageData.displacementX )
        self.adjDisplacementY.set_value( imageData.displacementY )
        self.filePathImage.setFilename( imageData.filename )
        
        self.fillingImageDataUI = False
        
        self.onCurImageUpdated()
    
    #---------------------------------------------------------------------------
    def onFilePathImageChanged( self, widget, data=None ):
        
        if not self.fillingImageDataUI and not self.handlingFilePath:
            
            self.handlingFilePath = True
            
            curImageName = self.getCurImageName()
            imageData = self.config.imageData[ curImageName ]
            
            # Make path relative to configDir
            filename = self.filePathImage.getFilename()
            filename = os.path.relpath( filename, self.configDir )
            self.filePathImage.setFilename( filename )
            
            imageData.filename = filename

            self.handlingFilePath = False
            
            self.onCurImageUpdated()
    
    #---------------------------------------------------------------------------
    def onBtnUpdateSegmentationClicked( self, widget ):
        
        if self.maskArray != None \
            and "ImpactImage" in self.images.keys() \
            and self.images[ "ImpactImage" ] != None:
            
            workingMask = np.copy( self.maskArray )
            
            fgModel = cv.CreateMat( 1, 5*13, cv.CV_64FC1 )
            cv.Set( fgModel, 0 )
            bgModel = cv.CreateMat( 1, 5*13, cv.CV_64FC1 )
            cv.Set( bgModel, 0 )
            
            workingImage = np.copy( self.images[ "ImpactImage" ] )
            cv.GrabCut( workingImage, workingMask, 
                (0,0,0,0), fgModel, bgModel, 6, self.GC_INIT_WITH_MASK )
                
            cv.Set( fgModel, 0 )
            cv.Set( bgModel, 0 )
            bgdPixels = (workingMask != self.GC_PR_FGD) & (workingMask != self.GC_FGD)
            workingMask[ bgdPixels ] = 0
            workingMask[ bgdPixels == False ] = 255
            cv.Erode( workingMask, workingMask )
            bgdPixels = workingMask == 0
            workingMask[ bgdPixels ] = self.GC_PR_BGD
            workingMask[ bgdPixels == False ] = self.GC_PR_FGD
            workingMask[ self.exclusionMask == 0 ] = self.GC_BGD
            
            cv.GrabCut( workingImage, workingMask, 
                (0,0,0,0), fgModel, bgModel, 6, self.GC_INIT_WITH_MASK )
            
            segmentation = np.copy( self.images[ "ImpactImage" ] )
            segmentation[ (workingMask != self.GC_PR_FGD) & (workingMask != self.GC_FGD) ] = 0
            self.dwgSegmentedImageDisplay.setImageFromNumpyArray( segmentation )
        else:
            self.dwgSegmentedImageDisplay.clear()
    
    #---------------------------------------------------------------------------
    def onDwgCurImageExposeEvent( self, widget, data ):
        
        self.dwgCurImageDisplay.drawPixBufToDrawingArea( data.area )
    
    #---------------------------------------------------------------------------
    def onDwgMergedImageExposeEvent( self, widget, data ):
        
        self.dwgMergedImageDisplay.drawPixBufToDrawingArea( data.area )
    
    #---------------------------------------------------------------------------
    def onDwgImpactMotionImageExposeEvent( self, widget, data ):
        
        self.dwgImpactMotionImageDisplay.drawPixBufToDrawingArea( data.area )
    
    #---------------------------------------------------------------------------
    def onDwgAccumulatorImageExposeEvent( self, widget, data ):
        
        self.dwgAccumulatorImageDisplay.drawPixBufToDrawingArea( data.area )
        
    #---------------------------------------------------------------------------
    def onDwgMaskImageExposeEvent( self, widget, data ):
        
        self.dwgMaskImageDisplay.drawPixBufToDrawingArea( data.area )
    
    #---------------------------------------------------------------------------
    def onDwgSegmentedImageExposeEvent( self, widget, data ):
        
        self.dwgSegmentedImageDisplay.drawPixBufToDrawingArea( data.area )
    
    #---------------------------------------------------------------------------
    def loadImageFromData( self, imageName ):
        
        result = None
        
        if imageName in self.config.imageData:
            imageData = self.config.imageData[ imageName ]
            
            if imageData.filename != "":
                imageFilename = self.configDir + "/" + imageData.filename
                cvImage = cv.LoadImageM( imageFilename )
                if cvImage == None:
                    print "Error: Unable to load", imageFilename
                else:
                    if self.isImageGrayscale( imageName ):
                        result = np.ndarray( ( cvImage.height, cvImage.width ), dtype=np.uint8 )
                        cv.CvtColor( cvImage, result, cv.CV_BGR2GRAY )
                    else:
                        cv.CvtColor( cvImage, cvImage, cv.CV_BGR2RGB )
                        result = np.array( cvImage )
            
        return result
        
    #---------------------------------------------------------------------------
    def onCurImageUpdated( self ):
        
        curImageName = self.getCurImageName()
        image = self.loadImageFromData( curImageName )
        self.images[ curImageName ] = image
        
        if image == None:
            self.dwgCurImageDisplay.clear()
        else:                
            self.dwgCurImageDisplay.setImageFromNumpyArray( image )
            
        self.updateImpactMotionImage()
        self.updateMergedImage()
            
    #---------------------------------------------------------------------------
    def updateImpactMotionImage( self ):
        
        image = self.loadImageFromData( "ImpactMotion" )
        self.images[ "ImpactMotion" ] = image
        
        if image == None:
            self.dwgImpactMotionImageDisplay.clear()
        else:                
            self.dwgImpactMotionImageDisplay.setImageFromNumpyArray( image )
            
    #---------------------------------------------------------------------------
    def updateMergedImage( self ):
        
        mergeCanHappen = False
        
        # First check to see if the conditions are right to do a merge
        if self.checkAddToMask.get_active():
            curImageName = self.getCurImageName()
            if curImageName.find( "PreMotion" ) == 0 \
                or curImageName.find( "PostMotion" ) == 0:
                    
                if "ImpactMotion" in self.images.keys() \
                    and self.images[ "ImpactMotion" ] != None\
                    and curImageName in self.images.keys() \
                    and self.images[ curImageName ] != None:
                    
                    mergeCanHappen = True
                    
        if mergeCanHappen:
            
            targetImageGray = self.images[ "ImpactMotion" ]
            templateImageGray = self.images[ curImageName ]
        
            # Create a transformed version of the template image
            transformedImage = scipy.ndimage.interpolation.shift( 
                templateImageGray, 
                ( self.adjDisplacementY.get_value(), self.adjDisplacementX.get_value() ) )
        
            # Create a composite image using the target image for the red channel
            # and the template image for the green channel. This should show 
            # matched pixels as yellow
            width = targetImageGray.shape[ 1 ]
            height = targetImageGray.shape[ 0 ]
            mergedImage = np.zeros( ( height, width, 3 ), dtype=np.uint8 )
            mergedImage[ :, :, 0 ] = targetImageGray
            mergedImage[ :, :, 1 ] = transformedImage
        
            # Display the merged image
            self.dwgMergedImageDisplay.setImageFromNumpyArray( mergedImage )
        
            ## Calculate and display the Sum of Squared Differences (SSD) between the 2 images
            #SSDValues = np.square( 
                #transformedImage.astype( np.int32 ) - self.targetImageGray.astype( np.int32 ) )
            #EPSILON = 128
            #SSDValues[ SSDValues <= EPSILON*EPSILON ] = 0
                
            #transformSSD = np.sum( SSDValues )
            #self.lblSSDDisplay.set_text( str( transformSSD ) )
        else:
            self.dwgMergedImageDisplay.clear()
            
        self.updateAccumulatorImage()
        
    #---------------------------------------------------------------------------
    def updateAccumulatorImage( self ):
        
        if "ImpactMotion" in self.images.keys() \
            and self.images[ "ImpactMotion" ] != None:
            
            # The accumulator starts with the impact image
            accumulatorArray = np.copy( self.images[ "ImpactMotion" ] ).astype( np.int32 )
            
            # Take maximum values from motion images after the impact but
            # don't add them in to de-emphasise the manipulator
            for imageName in self.config.imageData.keys():
                                
                image = self.images[ imageName ]
                imageData = self.config.imageData[ imageName ]
                
                if image != None \
                    and imageData.active \
                    and imageName.find( "PostMotion" ) == 0:
                
                    transformedImage = scipy.ndimage.interpolation.shift( 
                        image, ( imageData.displacementY, imageData.displacementX ) )
                    accumulatorArray = np.maximum( accumulatorArray, transformedImage )
                    
            # Dilate and subtract motion images from before the impact
            for imageName in self.config.imageData.keys():
                                
                image = self.images[ imageName ]
                imageData = self.config.imageData[ imageName ]
                
                if image != None \
                    and imageData.active \
                    and imageName.find( "PreMotion" ) == 0:
                
                    transformedImage = scipy.ndimage.interpolation.shift( 
                        image, ( imageData.displacementY, imageData.displacementX ) )
                    cv.Dilate( transformedImage, transformedImage )
                    cv.Dilate( transformedImage, transformedImage )
                    cv.Dilate( transformedImage, transformedImage )
                    accumulatorArray = accumulatorArray - transformedImage
            
            self.accumulatorImage = np.clip( accumulatorArray, 0, 255 ).astype( np.uint8 )
            self.dwgAccumulatorImageDisplay.setImageFromNumpyArray( self.accumulatorImage )
        
        else:
            self.accumulatorImage = None
            self.dwgAccumulatorImageDisplay.clear()
            
        self.updateMaskImage()
            
    #---------------------------------------------------------------------------
    def updateMaskImage( self ):
        
        USING_OPTICAL_FLOW = False
        ROI_X = 0
        ROI_Y = 76
        ROI_WIDTH = 230
        ROI_HEIGHT = 100
        
        if self.accumulatorImage != None:
            
            # Create the segmentation mask from the accumulator image
            startMask = np.copy( self.accumulatorImage )
            cv.Dilate( startMask, startMask )
            cv.Erode( startMask, startMask )
            cv.Dilate( startMask, startMask )
            cv.Erode( startMask, startMask )
            startMask = scipy.ndimage.filters.gaussian_filter( 
                startMask, 5.0, mode='constant' )
            
            startMask[ startMask > 0 ] = 255
            #cv.Erode( startMask, startMask )
            #cv.Dilate( startMask, startMask )
            
            #if USING_OPTICAL_FLOW:
            #    cv.Erode( startMask, startMask )
            #    cv.Erode( startMask, startMask )
                
            # Find the larget blob in the ROI
            # Label blobs
            startMask, numBlobs = PyBlobLib.labelBlobs( startMask )
            
            # Find blobs in the region of interest
            testMap = np.copy( startMask )
            testMap[ :ROI_Y, : ] = 0       # Mask out area above the ROI
            testMap[ :, :ROI_X ] = 0       # Mask out area to the left of the ROI
            testMap[ ROI_Y+ROI_HEIGHT: ] = 0   # Mask out area below the ROI
            testMap[ :, ROI_X+ROI_WIDTH: ] = 0   # Mask out area to the right of the ROI
        
            biggestBlobIdx = None
            biggestBlobSize = 0
        
            for blobIdx in range( 1, numBlobs + 1 ):
                if testMap[ testMap == blobIdx ].size > 0:
                    blobSize = startMask[ startMask == blobIdx ].size
                    if blobSize > biggestBlobSize:
                        biggestBlobSize = blobSize
                        biggestBlobIdx = blobIdx
        
            # Isolate the largest blob
            if biggestBlobIdx != None:
                biggestBlobPixels = (startMask == biggestBlobIdx)
                startMask[ biggestBlobPixels ] = 255
                startMask[ biggestBlobPixels == False ] = 0
            else:
                print "No central blob"
                self.maskArray = None
                self.dwgMaskImageDisplay.clear()
                
            # Now expand it to get exclusion mask
            self.exclusionMask = np.copy( startMask )
            for i in range( 10 ):
                cv.Dilate( self.exclusionMask, self.exclusionMask )
            cv.Erode( self.exclusionMask, self.exclusionMask )
            cv.Erode( self.exclusionMask, self.exclusionMask )
            
            #----------------------------------------------------
            
            self.maskArray = np.copy( startMask )
            possiblyForeground = ( self.maskArray > 0 ) & ( self.accumulatorImage > 0 )
            self.maskArray[ possiblyForeground ] = self.GC_PR_FGD
            self.maskArray[ possiblyForeground == False ] = self.GC_PR_BGD
            self.maskArray[ self.exclusionMask == 0 ] = self.GC_BGD
            
            self.definiteMask = np.copy( self.accumulatorImage )
            self.definiteMask[ possiblyForeground ] = 255
            self.definiteMask[ possiblyForeground == False ] = 0
            cv.Erode( self.definiteMask, self.definiteMask )
            cv.Erode( self.definiteMask, self.definiteMask )
            self.maskArray[ self.definiteMask == 255 ] = self.GC_FGD
            
            #if not USING_OPTICAL_FLOW:
            #    smallMask = np.copy( startMask )
            #    smallMask[ smallMask > 0 ] = 255
            #    cv.Erode( smallMask, smallMask )
            #    self.maskArray[ smallMask > 0 ] = self.GC_FGD
            
            # Draw the segmentation mask
            composedImage = None
            if "ImpactImage" in self.images.keys() \
                and self.images[ "ImpactImage" ] != None:
                composedImage = np.copy( self.images[ "ImpactImage" ] )
                
            if composedImage == None:
                height = self.accumulatorImage.shape[ 1 ]
                width = self.accumulatorImage.shape[ 0 ]
                composedImage = np.zeros( ( height, width, 3 ), dtype=np.uint8 )
            
            composedImage[ self.maskArray == self.GC_FGD ] = self.FOREGROUND_BRUSH_COLOUR
            composedImage[ self.maskArray == self.GC_PR_FGD ] = self.PROBABLY_FOREGROUND_BRUSH_COLOUR
            composedImage[ self.maskArray == self.GC_BGD ] = self.BACKGROUND_BRUSH_COLOUR
            
            self.dwgMaskImageDisplay.setImageFromNumpyArray( composedImage )
        else:
            
            self.maskArray = None
            self.dwgMaskImageDisplay.clear()
            
#-------------------------------------------------------------------------------
if __name__ == "__main__":

    mainWindow = MainWindow()
    mainWindow.main()
