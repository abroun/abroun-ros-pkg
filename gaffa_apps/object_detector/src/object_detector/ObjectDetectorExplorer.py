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
                
            output_2_Mode = self.comboOutput_2_Mode.get_active_text()
            if output_2_Mode == OutputMode.OPTICAL_FLOW:
                self.dwgOutput_2_Display.setImageFromOpenCVMatrix( self.inputImageList[ self.frameIdx ] )
            elif output_2_Mode == OutputMode.DETECTED_MOTION:
                self.dwgOutput_2_Display.setImageFromNumpyArray( self.motionImageList[ self.frameIdx ] )
            elif output_2_Mode == OutputMode.SEGMENTATION:
                self.dwgOutput_2_Display.setImageFromNumpyArray( self.segmentationList[ self.frameIdx ] )
        
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
        self.maxMotionCounts = [ 0 for i in range( numFrames ) ]
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
                
                # Calculate optical flow
                opticalFlowArrayX, opticalFlowArrayY = \
                    opticalFlowFilter.calcOpticalFlow( grayImage )
                    
                # Detect motion
                motionImage = motionDetectionFilter.calcMotion( grayImage )
                
                # Work out the maximum amount of motion we've seen in a single frame so far
                motionCount = motionImage[ motionImage > 0 ].size
                
                if frameIdx == 0:
                    lastMotionCount = 0
                else:
                    lastMotionCount = self.maxMotionCounts[ frameIdx - 1 ]
                    
                if motionCount < lastMotionCount:
                    motionCount = lastMotionCount
                
                # Segment the image
                workingMask = np.copy( motionImage )
                kernel = cv.CreateStructuringElementEx( 
                    cols=3, rows=3, 
                    anchorX=1, anchorY=1, shape=cv.CV_SHAPE_CROSS )
                cv.Erode( workingMask, workingMask, kernel )
                cv.Dilate( workingMask, workingMask )
                possibleForeground = workingMask > 0
            
                if workingMask[ possibleForeground ].size >= 875676567800:
                    
                    fgModel = cv.CreateMat( 1, 5*13, cv.CV_32FC1 )
                    bgModel = cv.CreateMat( 1, 5*13, cv.CV_32FC1 )
                    workingMask[ possibleForeground ] = self.GC_PR_FGD
                    workingMask[ possibleForeground == False ] = self.GC_PR_BGD
                
                    print "Start seg"
                    cv.GrabCut( image, workingMask, 
                        (0,0,0,0), fgModel, bgModel, 1, self.GC_INIT_WITH_MASK )
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
        self.vboxGraphs.pack_start( self.graphCanvas, True, True )
        self.vboxGraphs.pack_start( self.graphNavToolbar, expand=False, fill=False )
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
