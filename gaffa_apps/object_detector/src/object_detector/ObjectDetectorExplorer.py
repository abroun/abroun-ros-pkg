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
from abroun_gtk_gui.Display import Display
from gripper_detector.OpticalFlowFilter import OpticalFlowFilter
from MotionDetectionFilter import MotionDetectionFilter
from ImageFlowFilter import ImageFlowFilter
from ResidualSaliencyFilter import ResidualSaliencyFilter

import PyBlobLib
import PyVarFlowLib

#-------------------------------------------------------------------------------
class OutputMode:
    OPTICAL_FLOW = "Optical Flow"
    DETECTED_MOTION = "Detected Motion"
    SEGMENTATION = "Segmentation"
    SALIENCY = "Saliency"
    SEGMENTATION_MASK = "Segmentation Mask"

#-------------------------------------------------------------------------------
class MainWindow:
 
    OPTICAL_FLOW_BLOCK_WIDTH = 8
    OPTICAL_FLOW_BLOCK_HEIGHT = 8
    OPTICAL_FLOW_RANGE_WIDTH = 8    # Range to look outside of a block for motion
    OPTICAL_FLOW_RANGE_HEIGHT = 8
    
    PROCESSED_FRAME_DIFF = 2
    
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
               
        #updateLoop = self.update()
        #gobject.idle_add( updateLoop.next )
        
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
        gtk.gdk.threads_init()
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
            elif output_1_Mode == OutputMode.SEGMENTATION_MASK:
                self.dwgOutput_1_Display.setImageFromNumpyArray( self.segmentationMaskList[ self.frameIdx ] )
                
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
            elif output_2_Mode == OutputMode.SEGMENTATION_MASK:
                self.dwgOutput_2_Display.setImageFromNumpyArray( self.segmentationMaskList[ self.frameIdx ] )
        
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
        self.segmentationMaskList = [ None for i in range( numFrames ) ]
        self.imageFlowList = [ ( 0, 0, 0, None ) for i in range( numFrames ) ]
        self.maxMotionCounts = [ 0 for i in range( numFrames ) ]
        self.leftMostMotionList = [ 0 for i in range( numFrames ) ]
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
    
        FLIP_IMAGE = False
        USING_OPTICAL_FLOW_FOR_MOTION = False
    
        bagFrameIdx = 0
        frameIdx = 0
        impactFrameIdx = None
        
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
                
                if FLIP_IMAGE:
                    cv.Flip( image, None, 1 )
                
                # Convert to grayscale
                grayImage = cv.CreateMat( msg.height, msg.width, cv.CV_8UC1 )
                cv.CvtColor( image, grayImage, cv.CV_BGR2GRAY )
                grayImageNumpPy = np.array( grayImage )
                
                # Calculate optical flow
                opticalFlowArrayX, opticalFlowArrayY = \
                    opticalFlowFilter.calcOpticalFlow( grayImage )
                    
                # Detect motion
                if USING_OPTICAL_FLOW_FOR_MOTION:
                    if frameIdx == 0:
                        motionImage = PyVarFlowLib.createMotionMask( 
                            grayImageNumpPy, grayImageNumpPy )
                    else:
                        motionImage = PyVarFlowLib.createMotionMask( 
                            np.array( self.grayScaleImageList[ frameIdx - 1 ] ), 
                            grayImageNumpPy )
                else:
                    motionImage = motionDetectionFilter.calcMotion( grayImage )
                
                
                # Work out the left most point in the image where motion appears
                motionTest = np.copy( motionImage )
                
                cv.Erode( motionTest, motionTest )
                if frameIdx == 0:
                    leftMostMotion = motionImage.shape[ 1 ]
                else:
                    leftMostMotion = self.leftMostMotionList[ frameIdx - 1 ]
                
                leftMostMotionDiff = 0
                for i in range( leftMostMotion ):
                    if motionTest[ :, i ].max() > 0:
                        leftMostMotionDiff = abs( leftMostMotion - i )
                        leftMostMotion = i
                        break
                
                segmentationMask = np.zeros( ( msg.height, msg.width ), dtype=np.uint8 )
                
                FRAMES_BACK = 3
                
                if impactFrameIdx == None:        
                    if leftMostMotionDiff > 18 and leftMostMotion < 0.75*msg.width:
                        
                        # Found impact frame
                        impactFrameIdx = frameIdx
                    
                else:
                    PROCESS_IMPACT = False
                    if PROCESS_IMPACT and frameIdx - impactFrameIdx == FRAMES_BACK:
                        
                        # Should now have enough info to segment object
                        impactMotionImage = self.motionImageList[ impactFrameIdx ]
                        
                        print "Aligning"
                        postImpactRealFarFlow = imageFlowFilter.calcImageFlow( impactMotionImage, motionImage )
                        print "Aligning"
                        postImpactFarFlow = imageFlowFilter.calcImageFlow( impactMotionImage, self.motionImageList[ impactFrameIdx + 2 ] )
                        print "Aligning"
                        postImpactNearFlow = imageFlowFilter.calcImageFlow( impactMotionImage, self.motionImageList[ impactFrameIdx + 1 ] )
                        
                        segmentationMask = np.maximum( np.maximum( np.maximum( 
                            impactMotionImage, postImpactNearFlow[ 3 ] ), postImpactFarFlow[ 3 ] ), postImpactRealFarFlow[ 3 ] )
                        cv.Dilate( segmentationMask, segmentationMask )
                        
                        print "Aligning"
                        preImpactRealFarFlow = imageFlowFilter.calcImageFlow( impactMotionImage, self.motionImageList[ impactFrameIdx - 8 ] )
                        print "Aligning"
                        preImpactFarFlow = imageFlowFilter.calcImageFlow( impactMotionImage, self.motionImageList[ impactFrameIdx - 6 ] )
                        print "Aligning"
                        preImpactNearFlow = imageFlowFilter.calcImageFlow( impactMotionImage, self.motionImageList[ impactFrameIdx - 4 ] )
                        
                        subMask = np.maximum( np.maximum( 
                            preImpactRealFarFlow[ 3 ], preImpactFarFlow[ 3 ] ), preImpactNearFlow[ 3 ] )
                        cv.Erode( subMask, subMask )
                        cv.Dilate( subMask, subMask )
                        cv.Dilate( subMask, subMask )
                        cv.Dilate( subMask, subMask )
                        
                        subMask[ subMask > 0 ] = 255
                        diffImage = segmentationMask.astype( np.int32 ) - subMask.astype( np.int32 )
                        diffImage[ diffImage < 0 ] = 0
                        diffImage = diffImage.astype( np.uint8 )
                        cv.Erode( diffImage, diffImage )
                        #diffImage[ diffImage > 0 ] = 255

                        #segmentationMask = subMask
                        segmentationMask = diffImage
                        #segmentationMask = np.where( diffImage > 128, 255, 0 ).astype( np.uint8 )
                
                # Calculate image flow
                #imageFlow = imageFlowFilter.calcImageFlow( motionImage )
                
                ## Calculate saliency map
                #saliencyMap, largeSaliencyMap = residualSaliencyFilter.calcSaliencyMap( grayImageNumpPy )
                
                #blobMap = np.where( largeSaliencyMap > 128, 255, 0 ).astype( np.uint8 )
                
                #blobMap, numBlobs = PyBlobLib.labelBlobs( blobMap )
                #print "found", numBlobs, "blobs"
                
                #largeSaliencyMap = np.where( largeSaliencyMap > 128, 255, 0 ).astype( np.uint8 )
                
                
                
                
                
                
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
                #motionCount = motionImage[ motionImage > 0 ].size
                
                #if frameIdx == 0:
                    #lastMotionCount = 0
                #else:
                    #lastMotionCount = self.maxMotionCounts[ frameIdx - 1 ]
                    
                #if motionCount < lastMotionCount:
                    #motionCount = lastMotionCount
                
                ## Work out diffImage    
                #diffImage = np.array( motionImage, dtype=np.int32 ) \
                     #- np.array( imageFlow[ 3 ], dtype=np.int32 )
                #diffImage = np.array( np.maximum( diffImage, 0 ), dtype=np.uint8 )
                
                
                
                
                
                # Segment the image
                #workingMask = np.copy( motionImage )
                #workingMask = np.copy( diffImage )
                workingMask = np.copy( segmentationMask )
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
                    
                    fgModel = cv.CreateMat( 1, 5*13, cv.CV_64FC1 )
                    bgModel = cv.CreateMat( 1, 5*13, cv.CV_64FC1 )
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
                
                    imageToSegment = image #self.inputImageList[ frameIdx - FRAMES_BACK ]
                
                    imageCopy = np.copy( imageToSegment )
                    cv.CvtColor( imageCopy, imageCopy, cv.CV_BGR2RGB )
                
                    print "Start seg"
                    cv.GrabCut( imageCopy, workingMask, 
                        (0,0,0,0), fgModel, bgModel, 12, self.GC_INIT_WITH_MASK )
                    print "Finish seg"
                
                    segmentation = np.copy( imageToSegment )
                    segmentation[ (workingMask != self.GC_PR_FGD) & (workingMask != self.GC_FGD) ] = 0
                
                    
                    black = (workingMask != self.GC_PR_FGD) & (workingMask != self.GC_FGD)
                    #motionImage = np.where( black, 0, 255 ).astype( np.uint8 )
                    
                    # Refine the segmentation
                    REFINE_SEG = False
                    if REFINE_SEG:
                        motionImageCopy = np.copy( motionImage )
                        cv.Erode( motionImageCopy, motionImageCopy )
                        #cv.Erode( motionImageCopy, motionImageCopy )
                        #cv.Erode( motionImageCopy, motionImageCopy )
                        
                        workingMask[ motionImageCopy > 0 ] = self.GC_PR_FGD
                        workingMask[ motionImageCopy == 0 ] = self.GC_PR_BGD
                        
                        cv.Dilate( motionImageCopy, motionImageCopy )
                        cv.Dilate( motionImageCopy, motionImageCopy )
                        cv.Dilate( motionImageCopy, motionImageCopy )
                        cv.Dilate( motionImageCopy, motionImageCopy )
                        workingMask[ motionImageCopy == 0 ] = self.GC_BGD
                        
                        print "Other seg"
                        cv.GrabCut( imageCopy, workingMask, 
                            (0,0,0,0), fgModel, bgModel, 12, self.GC_INIT_WITH_MASK )
                        print "Other seg done"
                            
                        segmentation = np.copy( imageToSegment )
                        segmentation[ (workingMask != self.GC_PR_FGD) & (workingMask != self.GC_FGD) ] = 0
                    
                        
                        black = (workingMask != self.GC_PR_FGD) & (workingMask != self.GC_FGD)
                        motionImage = np.where( black, 0, 255 ).astype( np.uint8 )
                    
                
                else:
                    segmentation = np.zeros( ( image.height, image.width ), dtype=np.uint8 )
                
                
                # Save output data
                self.inputImageList[ frameIdx ] = image
                self.grayScaleImageList[ frameIdx ] = grayImage
                self.opticalFlowListX[ frameIdx ] = opticalFlowArrayX
                self.opticalFlowListY[ frameIdx ] = opticalFlowArrayY
                self.motionImageList[ frameIdx ] = motionImage
                self.segmentationList[ frameIdx ] = segmentation
                self.segmentationMaskList[ frameIdx ] = segmentationMask
                #self.maxMotionCounts[ frameIdx ] = motionCount
                #self.imageFlowList[ frameIdx ] = imageFlow
                #self.saliencyMapList[ frameIdx ] = largeSaliencyMap
                #self.saliencyClusterList[ frameIdx ] = clusterList
                self.leftMostMotionList[ frameIdx ] = leftMostMotion
                
                frameIdx += 1
                self.numFramesProcessed += 1
                
        if not self.workCancelled:
            
            
            SAVE_MOTION_IMAGES = True
            BASE_MOTION_IMAGE_NAME = self.scriptPath + "/../../test_data/motion_images/motion_{0:03}.png"
            
            if SAVE_MOTION_IMAGES and len( self.motionImageList ) > 0:
                
                width = self.motionImageList[ 0 ].shape[ 1 ]
                height = self.motionImageList[ 0 ].shape[ 0 ]
                colourImage = np.zeros( ( height, width, 3 ), dtype=np.uint8 )
                
                for frameIdx, motionImage in enumerate( self.motionImageList ):
                    
                    colourImage[ :, :, 0 ] = motionImage
                    colourImage[ :, :, 1 ] = motionImage
                    colourImage[ :, :, 2 ] = motionImage
                    
                    outputName = BASE_MOTION_IMAGE_NAME.format( frameIdx + 1 )
                    cv.SaveImage( outputName, colourImage )
            
            # Recalculate impactFrameIdx
            width = self.motionImageList[ 0 ].shape[ 1 ]
            
            totalMotionDiff = 0
            maxMotionDiff = 0
            impactFrameIdx = None
            for motionIdx in range( 1, len( self.leftMostMotionList ) ):
            
                motionDiff = abs( self.leftMostMotionList[ motionIdx ] \
                    - self.leftMostMotionList[ motionIdx - 1 ] )
                totalMotionDiff += motionDiff
                    
                if motionDiff > maxMotionDiff and totalMotionDiff > 0.5*width:
                    maxMotionDiff = motionDiff
                    impactFrameIdx = motionIdx
            
            if maxMotionDiff <= 18:
                impactFrameIdx = None
                    
            
            if impactFrameIdx != None:
                
                NUM_FRAMES_BEFORE = 8
                BASE_MOTION_IMAGE_NAME = self.scriptPath + "/../../test_data/impact_images/motion_{0:03}.png"
                START_MOTION_IMAGE_NAME = self.scriptPath + "/../../test_data/impact_images/start_motion.png"
                START_IMAGE_NAME = self.scriptPath + "/../../test_data/impact_images/start.png"
                IMPACT_IMAGE_NAME = self.scriptPath + "/../../test_data/impact_images/impact.png"
                NUM_FRAMES_AFTER = 3
                
                width = self.motionImageList[ 0 ].shape[ 1 ]
                height = self.motionImageList[ 0 ].shape[ 0 ]
                colourImage = np.zeros( ( height, width, 3 ), dtype=np.uint8 )
                
                for frameIdx in range( impactFrameIdx - NUM_FRAMES_BEFORE,
                    impactFrameIdx + NUM_FRAMES_AFTER + 1 ):
                    
                    motionImage = self.motionImageList[ frameIdx ]  
                    colourImage[ :, :, 0 ] = motionImage
                    colourImage[ :, :, 1 ] = motionImage
                    colourImage[ :, :, 2 ] = motionImage
                    
                    outputName = BASE_MOTION_IMAGE_NAME.format( frameIdx - impactFrameIdx )
                    cv.SaveImage( outputName, colourImage )
                
                motionDetectionFilter.calcMotion( self.grayScaleImageList[ 0 ] )
                startMotionImage = motionDetectionFilter.calcMotion( 
                    self.grayScaleImageList[ impactFrameIdx ] )
                colourImage[ :, :, 0 ] = startMotionImage
                colourImage[ :, :, 1 ] = startMotionImage
                colourImage[ :, :, 2 ] = startMotionImage  
                cv.SaveImage( START_MOTION_IMAGE_NAME, colourImage )
                
                cv.CvtColor( self.inputImageList[ 0 ], colourImage, cv.CV_RGB2BGR )    
                cv.SaveImage( START_IMAGE_NAME, colourImage )
                cv.CvtColor( self.inputImageList[ impactFrameIdx ], colourImage, cv.CV_RGB2BGR )    
                cv.SaveImage( IMPACT_IMAGE_NAME, colourImage )
                    
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
        #self.graphAxis.plot( range( 1, len( self.maxMotionCounts )+1 ), self.maxMotionCounts )
        diffs = [ 0 ] + [ self.leftMostMotionList[ i+1 ] - self.leftMostMotionList[ i ] for i in range( len( self.leftMostMotionList ) - 1 ) ]
        #self.graphAxis.plot( range( 1, len( self.leftMostMotionList )+1 ), self.leftMostMotionList )
        self.graphAxis.plot( range( 1, len( self.leftMostMotionList )+1 ), diffs )
        
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
