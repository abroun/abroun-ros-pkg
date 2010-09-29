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
from gripper_detector.OpticalFlowFilter import OpticalFlowFilter
from MotionDetectionFilter import MotionDetectionFilter
from ImageFlowFilter import ImageFlowFilter
from ResidualSaliencyFilter import ResidualSaliencyFilter

import PyBlobLib

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
class OutputMode:
    OPTICAL_FLOW = "Optical Flow"
    DETECTED_MOTION = "Detected Motion"
    SEGMENTATION = "Segmentation"
    SALIENCY = "Saliency"

#-------------------------------------------------------------------------------
class MainWindow:
 
    OPTICAL_FLOW_BLOCK_WIDTH = 8
    OPTICAL_FLOW_BLOCK_HEIGHT = 8
    OPTICAL_FLOW_RANGE_WIDTH = 8    # Range to look outside of a block for motion
    OPTICAL_FLOW_RANGE_HEIGHT = 8
    
    PROCESSED_FRAME_DIFF = 3
    
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
    def __init__( self, bagFilename = None ):
    
        self.scriptPath = os.path.dirname( __file__ )
        self.image = None
        self.frameIdx = 0
        self.workerThread = None
        self.numFramesProcessed = 0
        self.graphCanvas = None
        self.graphNavToolbar = None
            
        # Setup the GUI        
        builder = gtk.Builder()
        builder.add_from_file( self.scriptPath + "/GUI/ObjectDetectorExplorer.glade" )
        
        self.window = builder.get_object( "winMain" )   
        self.comboOutput_1_Mode = builder.get_object( "comboOutput_1_Mode" )
        self.comboOutput_2_Mode = builder.get_object( "comboOutput_2_Mode" )
        self.vboxGraphs = builder.get_object( "vboxGraphs" )

        dwgInput = builder.get_object( "dwgInput" )
        dwgOutput_1 = builder.get_object( "dwgOutput_1" )
        dwgOutput_2 = builder.get_object( "dwgOutput_2" )
        self.dwgInputDisplay = Display( dwgInput )
        self.dwgOutput_1_Display = Display( dwgOutput_1 )
        self.dwgOutput_2_Display = Display( dwgOutput_2 )
        
        self.sequenceControls = builder.get_object( "sequenceControls" )
        self.sequenceControls.setNumFrames( 1 )
        self.sequenceControls.setOnFrameIdxChangedCallback( self.onSequenceControlsFrameIdxChanged )
        
        builder.connect_signals( self )
               
        updateLoop = self.update()
        gobject.idle_add( updateLoop.next )
        
        self.window.show()
        
        if bagFilename != None:
            self.tryToLoadBagFile( bagFilename )
        
    #---------------------------------------------------------------------------
    def onWinMainDestroy( self, widget, data = None ):  
        gtk.main_quit()
        
    #---------------------------------------------------------------------------   
    def main( self ):
        # All PyGTK applications must have a gtk.main(). Control ends here
        # and waits for an event to occur (like a key press or mouse event).
        gtk.main()
        
    #---------------------------------------------------------------------------
    def isCurFrameReady( self ):
        return self.frameIdx < self.numFramesProcessed
        
    #---------------------------------------------------------------------------
    def setFrameIdx( self, frameIdx ):
        
        frameReady = frameIdx < self.numFramesProcessed
        
        if frameReady or frameIdx == 0:
            self.frameIdx = frameIdx
            self.updateDisplay()
        else:
            # Try to reset to current frame index
            if self.isCurFrameReady():
                self.sequenceControls.setFrameIdx( self.frameIdx )
            else:
                self.sequenceControls.setFrameIdx( 0 )
        
    #---------------------------------------------------------------------------
    def updateDisplay( self ):
        
        if self.isCurFrameReady():
            self.dwgInputDisplay.setImageFromOpenCVMatrix( self.inputImageList[ self.frameIdx ] )
            
            output_1_Mode = self.comboOutput_1_Mode.get_active_text()
            if output_1_Mode == OutputMode.OPTICAL_FLOW:
                self.dwgOutput_1_Display.setImageFromOpenCVMatrix( self.inputImageList[ self.frameIdx ] )
            elif output_1_Mode == OutputMode.DETECTED_MOTION:
                self.dwgOutput_1_Display.setImageFromNumpyArray( self.motionImageList[ self.frameIdx ] )
            elif output_1_Mode == OutputMode.SEGMENTATION:
                self.dwgOutput_1_Display.setImageFromNumpyArray( self.segmentationList[ self.frameIdx ] )
            elif output_1_Mode == OutputMode.SALIENCY:
                self.dwgOutput_1_Display.setImageFromNumpyArray( self.saliencyMapList[ self.frameIdx ] )
                
            output_2_Mode = self.comboOutput_2_Mode.get_active_text()
            if output_2_Mode == OutputMode.OPTICAL_FLOW:
                self.dwgOutput_2_Display.setImageFromOpenCVMatrix( self.inputImageList[ self.frameIdx ] )
            elif output_2_Mode == OutputMode.DETECTED_MOTION:
                self.dwgOutput_2_Display.setImageFromNumpyArray( self.motionImageList[ self.frameIdx ] )
            elif output_2_Mode == OutputMode.SEGMENTATION:
                
                diffImage = np.array( self.motionImageList[ self.frameIdx ], dtype=np.int32 ) \
                     - np.array( self.imageFlowList[ self.frameIdx ][ 3 ], dtype=np.int32 )
                diffImage = np.array( np.maximum( diffImage, 0 ), dtype=np.uint8 )
                
                #self.dwgOutput_2_Display.setImageFromNumpyArray( diffImage )
                self.dwgOutput_2_Display.setImageFromNumpyArray( self.segmentationList[ self.frameIdx ] )
            elif output_2_Mode == OutputMode.SALIENCY:
                self.dwgOutput_2_Display.setImageFromNumpyArray( self.saliencyMapList[ self.frameIdx ] )
        
    #---------------------------------------------------------------------------
    def chooseBagFile( self ):
        
        result = None
        
        dialog = gtk.FileChooserDialog(
            title="Choose Bag File",
            action=gtk.FILE_CHOOSER_ACTION_SAVE,
            buttons=(gtk.STOCK_CANCEL, gtk.RESPONSE_REJECT,
                      gtk.STOCK_OK, gtk.RESPONSE_ACCEPT) )

        dialog.set_current_folder( self.scriptPath + "/../../test_data/bags" )
            
        filter = gtk.FileFilter()
        filter.add_pattern( "*.bag" )
        filter.set_name( "Bag Files" )
        dialog.add_filter( filter )
        dialog.set_filter( filter )
            
        result = dialog.run()

        if result == gtk.RESPONSE_ACCEPT:
            result = dialog.get_filename()

        dialog.destroy()
        
        return result
    
    #---------------------------------------------------------------------------
    def onMenuItemOpenBagActivate( self, widget ):
                
        bagFilename = self.chooseBagFile()
        
        if bagFilename != None:
            self.tryToLoadBagFile( bagFilename )          
    
    #---------------------------------------------------------------------------
    def onMenuItemQuitActivate( self, widget ):
        self.onWinMainDestroy( widget )
       
    #---------------------------------------------------------------------------
    def onSequenceControlsFrameIdxChanged( self, widget ):
        self.setFrameIdx( widget.frameIdx )
    
    #---------------------------------------------------------------------------
    def onComboOutput_1_ModeChanged( self, widget ):
        self.updateDisplay()
        
    #---------------------------------------------------------------------------
    def onComboOutput_2_ModeChanged( self, widget ):
        self.updateDisplay()
       
    #---------------------------------------------------------------------------
    def onDwgInputExposeEvent( self, widget, data ):
        
        self.dwgInputDisplay.drawPixBufToDrawingArea( data.area )
        
    #---------------------------------------------------------------------------
    def onDwgOutput_1_ExposeEvent( self, widget, data = None ):
        
        imgRect = self.dwgOutput_1_Display.drawPixBufToDrawingArea( data.area ) 
        
        if imgRect != None:
            imgRect = imgRect.intersect( data.area )
            
            outputMode = self.comboOutput_1_Mode.get_active_text()
            self.drawOutputOverlay( widget, imgRect, outputMode )
        
    #---------------------------------------------------------------------------
    def onDwgOutput_2_ExposeEvent( self, widget, data = None ):
        
        imgRect = self.dwgOutput_2_Display.drawPixBufToDrawingArea( data.area )
        
        if imgRect != None:
            imgRect = imgRect.intersect( data.area )
            
            outputMode = self.comboOutput_2_Mode.get_active_text()
            self.drawOutputOverlay( widget, imgRect, outputMode )
        
    #---------------------------------------------------------------------------
    def drawOutputOverlay( self, widget, imgRect, outputMode ):
        
        if outputMode == OutputMode.OPTICAL_FLOW:
                
            # Draw the optical flow if it's available
            opticalFlowX = self.opticalFlowListX[ self.frameIdx ]
            opticalFlowY = self.opticalFlowListY[ self.frameIdx ]
            if opticalFlowX != None and opticalFlowY != None:
            
                graphicsContext = widget.window.new_gc()
                graphicsContext.set_rgb_fg_color( gtk.gdk.Color( 0, 65535, 0 ) )
                
                blockCentreY = imgRect.y + self.OPTICAL_FLOW_BLOCK_HEIGHT / 2
                for y in range( opticalFlowX.shape[ 0 ] ):
                
                    blockCentreX = imgRect.x + self.OPTICAL_FLOW_BLOCK_WIDTH / 2
                    for x in range( opticalFlowX.shape[ 1 ] ):
                            
                        endX = blockCentreX + opticalFlowX[ y, x ]
                        endY = blockCentreY + opticalFlowY[ y, x ]
                        
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
                    
        elif outputMode == OutputMode.SALIENCY:
             
            graphicsContext = widget.window.new_gc()
            graphicsContext.set_rgb_fg_color( gtk.gdk.Color( 0, 65535, 0 ) )
                    
            for cluster in self.saliencyClusterList[ self.frameIdx ]:
                
                mean = cluster[ 0 ]
                stdDev = cluster[ 1 ]
                arcX = int( imgRect.x + mean[ 0 ] )
                arcY = int( imgRect.y + mean[ 1 ] )
                
                # Draw a circle to represent the cluster
                arcWidth = arcHeight = int( stdDev * 2 )
            
                drawFilledArc = False
                            
                widget.window.draw_arc( graphicsContext, 
                    drawFilledArc, arcX, arcY, arcWidth, arcHeight, 0, 360 * 64 )

    #---------------------------------------------------------------------------
    def tryToLoadBagFile( self, bagFilename ):
        
        # Locate and open file
        try:
            bag = rosbag.Bag( bagFilename )
        except:
            print "Error: Unable to load", bagFilename
            return
        
        # Count the number of frames in the bag file
        numFrames = 0
        for topic, msg, t in bag.read_messages():
            if msg._type == "sensor_msgs/Image":
                numFrames += 1
        
        if numFrames == 0:
            print "Error: No frames in bag file"
            return
            
        numFrames = int( math.ceil( float( numFrames )/int( self.PROCESSED_FRAME_DIFF ) ) )
        
        # Throw away existing data and prepare to process the bag file
        if self.workerThread != None and self.workerThread.is_alive():
            self.workCancelled = True
            self.workerThread.join()
            
        self.inputImageList = [ None for i in range( numFrames ) ]
        self.grayScaleImageList = [ None for i in range( numFrames ) ]
        self.motionImageList = [ None for i in range( numFrames ) ]
        self.opticalFlowListX = [ None for i in range( numFrames ) ]
        self.opticalFlowListY = [ None for i in range( numFrames ) ]
        self.segmentationList = [ None for i in range( numFrames ) ]
        self.imageFlowList = [ ( 0, 0, 0, None ) for i in range( numFrames ) ]
        self.maxMotionCounts = [ 0 for i in range( numFrames ) ]
        self.saliencyMapList = [ None for i in range( numFrames ) ]
        self.saliencyClusterList = [ [] for i in range( numFrames ) ]
        self.numFramesProcessed = 0
        
        # Kick off worker thread to process the bag file
        self.workCancelled = False
        self.workerThread = threading.Thread( target=self.processBag, args=( bag, ) )
        self.workerThread.daemon = True
        self.workerThread.start()
        
        self.sequenceControls.setNumFrames( numFrames )

    #---------------------------------------------------------------------------
    def processBag( self, bag ):
    
        bagFrameIdx = 0
        frameIdx = 0
        
        # Setup filters
        opticalFlowFilter = OpticalFlowFilter(
            self.OPTICAL_FLOW_BLOCK_WIDTH, self.OPTICAL_FLOW_BLOCK_HEIGHT, 
            self.OPTICAL_FLOW_RANGE_WIDTH, self.OPTICAL_FLOW_RANGE_HEIGHT )
            
        motionDetectionFilter = MotionDetectionFilter()
        imageFlowFilter = ImageFlowFilter()
        residualSaliencyFilter = ResidualSaliencyFilter()
            
        # Process bag file
        for topic, msg, t in bag.read_messages():
            
            if self.workCancelled:
                # We've been given the signal to quit
                break
            
            if msg._type == "sensor_msgs/Image":
                
                bagFrameIdx += 1
                if (bagFrameIdx-1)%self.PROCESSED_FRAME_DIFF != 0:
                    continue
                
                print "Processing image", frameIdx
                
                # Get input image
                image = cv.CreateMatHeader( msg.height, msg.width, cv.CV_8UC3 )
                cv.SetData( image, msg.data, msg.step )
                
                # Convert to grayscale
                grayImage = cv.CreateMat( msg.height, msg.width, cv.CV_8UC1 )
                cv.CvtColor( image, grayImage, cv.CV_BGR2GRAY )
                grayImageNumpPy = np.array( grayImage )
                
                # Calculate optical flow
                opticalFlowArrayX, opticalFlowArrayY = \
                    opticalFlowFilter.calcOpticalFlow( grayImage )
                    
                # Detect motion
                motionImage = motionDetectionFilter.calcMotion( grayImage )
                
                # Calculate image flow
                imageFlow = imageFlowFilter.calcImageFlow( motionImage )
                
                # Calculate saliency map
                saliencyMap, largeSaliencyMap = residualSaliencyFilter.calcSaliencyMap( grayImageNumpPy )
                
                largeSaliencyMap = PyBlobLib.labelBlobs( largeSaliencyMap )
                
                # Threshold the saliency map
                #largeSaliencyMap = (largeSaliencyMap > 128).astype(np.uint8) * 255
                #cv.AdaptiveThreshold( largeSaliencyMap, largeSaliencyMap, 255 )
                
                # Detect clusters within the saliency map
                #NUM_CLUSTERS = 5
                
                #numSamples = np.sum( saliencyMap )
                #sampleList = np.ndarray( ( numSamples, 2 ), dtype=np.float32 )
                
                #sampleListIdx = 0
                #for y in range( saliencyMap.shape[ 0 ] ):
                    #for x in range( saliencyMap.shape[ 1 ] ):
                        
                        #numNewSamples = saliencyMap[ y, x ]
                        #if numNewSamples > 0:
                            #sampleList[ sampleListIdx:sampleListIdx+numNewSamples, 0 ] = x
                            #sampleList[ sampleListIdx:sampleListIdx+numNewSamples, 1 ] = y
                            #sampleListIdx += numNewSamples
                            
                #sampleList[ 0:numSamples/2 ] = ( 20, 20 )
                #sampleList[ numSamples/2: ] = ( 200, 200 )
                
                #labelList = np.ndarray( ( numSamples, 1 ), dtype=np.int32 )
                #cv.KMeans2( sampleList, NUM_CLUSTERS, labelList, 
                    #(cv.CV_TERMCRIT_ITER | cv.CV_TERMCRIT_EPS, 10, 0.01) )
                    
                #saliencyScaleX = float( largeSaliencyMap.shape[ 1 ] ) / saliencyMap.shape[ 1 ]
                #saliencyScaleY = float( largeSaliencyMap.shape[ 0 ] ) / saliencyMap.shape[ 0 ]
                clusterList = []
                #for clusterIdx in range( NUM_CLUSTERS ):
                    
                    #clusterSamples = sampleList[ 
                        #np.where( labelList == clusterIdx )[ 0 ], : ]

                    #if clusterSamples.size <= 0:
                        #mean = ( 0.0, 0.0 )
                        #stdDev = 0.0
                    #else:
                        #mean = clusterSamples.mean( axis=0 )
                        #mean = ( mean[ 0 ]*saliencyScaleX, mean[ 1 ]*saliencyScaleY )
                        #stdDev = clusterSamples.std()*saliencyScaleX
                    
                    #clusterList.append( ( mean, stdDev ) )
                
                # Work out the maximum amount of motion we've seen in a single frame so far
                motionCount = motionImage[ motionImage > 0 ].size
                
                if frameIdx == 0:
                    lastMotionCount = 0
                else:
                    lastMotionCount = self.maxMotionCounts[ frameIdx - 1 ]
                    
                if motionCount < lastMotionCount:
                    motionCount = lastMotionCount
                
                # Work out diffImage    
                diffImage = np.array( motionImage, dtype=np.int32 ) \
                     - np.array( imageFlow[ 3 ], dtype=np.int32 )
                diffImage = np.array( np.maximum( diffImage, 0 ), dtype=np.uint8 )
                
                
                # Segment the image
                #workingMask = np.copy( motionImage )
                workingMask = np.copy( diffImage )
                kernel = cv.CreateStructuringElementEx( 
                    cols=3, rows=3, 
                    anchorX=1, anchorY=1, shape=cv.CV_SHAPE_CROSS )
                cv.Erode( workingMask, workingMask, kernel )
                cv.Dilate( workingMask, workingMask )
                
                extraExtraMask = np.copy( workingMask )
                cv.Dilate( extraExtraMask, extraExtraMask )
                cv.Dilate( extraExtraMask, extraExtraMask )
                cv.Dilate( extraExtraMask, extraExtraMask )
                cv.Dilate( extraExtraMask, extraExtraMask )
                cv.Dilate( extraExtraMask, extraExtraMask )
                cv.Dilate( extraExtraMask, extraExtraMask )
                
                allMask = np.copy( extraExtraMask )
                cv.Dilate( allMask, allMask )
                cv.Dilate( allMask, allMask )
                cv.Dilate( allMask, allMask )
                cv.Dilate( allMask, allMask )
                cv.Dilate( allMask, allMask )
                cv.Dilate( allMask, allMask )
                
                possibleForeground = workingMask > 0
            
                if workingMask[ possibleForeground ].size >= 100 \
                    and frameIdx >= 16:
                        
                    print "Msk size", workingMask[ possibleForeground ].size
                    print workingMask[ 0, 0:10 ]
                    
                    fgModel = cv.CreateMat( 1, 5*13, cv.CV_32FC1 )
                    bgModel = cv.CreateMat( 1, 5*13, cv.CV_32FC1 )
                    #workingMask[ possibleForeground ] = self.GC_FGD
                    #workingMask[ possibleForeground == False ] = self.GC_PR_BGD
                    
                    #workingMask[ : ] = self.GC_PR_BGD
                    #workingMask[ possibleForeground ] = self.GC_FGD
                    
                    workingMask[ : ] = self.GC_BGD
                    workingMask[ allMask > 0 ] = self.GC_PR_BGD
                    workingMask[ extraExtraMask > 0 ] = self.GC_PR_FGD
                    workingMask[ possibleForeground ] = self.GC_FGD
                    
                    
                    if frameIdx == 16:
                        # Save mask
                        maskCopy = np.copy( workingMask )
                        maskCopy[ maskCopy == self.GC_BGD ] = 0
                        maskCopy[ maskCopy == self.GC_PR_BGD ] = 64
                        maskCopy[ maskCopy == self.GC_PR_FGD ] = 128
                        maskCopy[ maskCopy == self.GC_FGD ] = 255
                        print "Unused pixels", \
                            maskCopy[ (maskCopy != 255) & (maskCopy != 0) ].size
                          
                        outputImage = cv.CreateMat( msg.height, msg.width, cv.CV_8UC3 )
                        cv.CvtColor( maskCopy, outputImage, cv.CV_GRAY2BGR )
                        
                        cv.SaveImage( "output.png", image );
                        cv.SaveImage( "outputMask.png", outputImage ); 
                        
                        print "Saved images"
                        #return 
                        
                    
                    #print "Set Msk size", workingMask[ workingMask == self.GC_PR_FGD ].size
                
                    imageCopy = np.copy( image )
                    cv.CvtColor( imageCopy, imageCopy, cv.CV_BGR2RGB )
                
                    print "Start seg"
                    cv.GrabCut( imageCopy, workingMask, 
                        (0,0,0,0), fgModel, bgModel, 6, self.GC_INIT_WITH_MASK )
                    print "Finish seg"
                
                    segmentation = np.copy( image )
                    segmentation[ (workingMask != self.GC_PR_FGD) & (workingMask != self.GC_FGD) ] = 0
                
                else:
                    segmentation = np.zeros( ( image.height, image.width ), dtype=np.uint8 )
                
                
                # Save output data
                self.inputImageList[ frameIdx ] = image
                self.grayScaleImageList[ frameIdx ] = grayImage
                self.opticalFlowListX[ frameIdx ] = opticalFlowArrayX
                self.opticalFlowListY[ frameIdx ] = opticalFlowArrayY
                self.motionImageList[ frameIdx ] = motionImage
                self.segmentationList[ frameIdx ] = segmentation
                self.maxMotionCounts[ frameIdx ] = motionCount
                self.imageFlowList[ frameIdx ] = imageFlow
                self.saliencyMapList[ frameIdx ] = largeSaliencyMap
                self.saliencyClusterList[ frameIdx ] = clusterList
                
                frameIdx += 1
                self.numFramesProcessed += 1
                
        if not self.workCancelled:
            self.refreshGraphDisplay()
            
        print "Finished processing bag file"
        
    #---------------------------------------------------------------------------
    def refreshGraphDisplay( self ):
        
        # Remove existing graph items
        if self.graphCanvas != None:   
            self.vboxGraphs.remove( self.graphCanvas )
            self.graphCanvas.destroy()  
            self.graphCanvas = None   
        if self.graphNavToolbar != None:
            self.vboxGraphs.remove( self.graphNavToolbar )
            self.graphNavToolbar.destroy()  
            self.graphNavToolbar = None   
            
        # Draw the graphs
        self.graphFigure = Figure( figsize=(8,6), dpi=72 )
        self.graphAxis = self.graphFigure.add_subplot( 111 )
        self.graphAxis.plot( range( 1, len( self.maxMotionCounts )+1 ), self.maxMotionCounts )
        
        # Build the new graph display
        self.graphCanvas = FigureCanvas( self.graphFigure ) # a gtk.DrawingArea
        self.graphCanvas.show()
        self.graphNavToolbar = NavigationToolbar( self.graphCanvas, self.window )
        self.graphNavToolbar.lastDir = '/var/tmp/'
        self.graphNavToolbar.show()
        
        # Show the graph
        self.vboxGraphs.pack_start( self.graphNavToolbar, expand=False, fill=False )
        self.vboxGraphs.pack_start( self.graphCanvas, True, True )
        self.vboxGraphs.show()
        self.vboxGraphs.show()

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

    bagFilename = None
    if len( sys.argv ) >= 2:
        bagFilename = sys.argv[ 1 ]

    mainWindow = MainWindow( bagFilename )
    mainWindow.main()
