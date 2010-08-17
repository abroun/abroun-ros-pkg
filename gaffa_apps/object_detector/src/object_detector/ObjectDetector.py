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
import scipy.cluster.vq as vq
import cv

import yaml
import pygtk
pygtk.require('2.0')
import gtk
import gobject

import sensor_msgs.msg
from gripper_detector.OpticalFlowFilter import OpticalFlowFilter

#-------------------------------------------------------------------------------
class MainWindow:
    
    OPTICAL_FLOW_BLOCK_WIDTH = 8
    OPTICAL_FLOW_BLOCK_HEIGHT = 8
    OPTICAL_FLOW_RANGE_WIDTH = 8    # Range to look outside of a block for motion
    OPTICAL_FLOW_RANGE_HEIGHT = 8
 
    #---------------------------------------------------------------------------
    def __init__( self ):
    
        self.scriptPath = os.path.dirname( __file__ )
        self.cameraImagePixBuf = None
        self.lastImageGray = None
        self.opticalFlowX = None
        self.opticalFlowY = None
        self.oldColours = None
        self.dataBuffersSetup = False
        self.npAcc = None
            
        # Connect to the robot via ROS
        rospy.init_node( 'ObjectDetector', anonymous=True )
        
        self.cameraImageTopic = rospy.Subscriber( "/camera/image", 
            sensor_msgs.msg.Image, self.cameraImageCallback )
            
        self.opticalFlowFilter = OpticalFlowFilter(
            self.OPTICAL_FLOW_BLOCK_WIDTH, self.OPTICAL_FLOW_BLOCK_HEIGHT,
            self.OPTICAL_FLOW_RANGE_WIDTH, self.OPTICAL_FLOW_RANGE_HEIGHT )   
            
        # Setup the GUI        
        builder = gtk.Builder()
        builder.add_from_file( self.scriptPath + "/GUI/ObjectDetector.glade" )
        
        self.window = builder.get_object( "winMain" )   
        self.dwgCameraImage = builder.get_object( "dwgCameraImage" )
        
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
    def cameraImageCallback( self, rosImage ):
        
        if rosImage.encoding == "rgb8" or rosImage.encoding == "bgr8":
            
            # Display the image
            #self.cameraImagePixBuf = gtk.gdk.pixbuf_new_from_data( 
                #rosImage.data, 
                #gtk.gdk.COLORSPACE_RGB,
                #False,
                #8,
                #rosImage.width,
                #rosImage.height,
                #rosImage.step )
                
            # Create an OpenCV image to process the data
            curImage = cv.CreateImageHeader( ( rosImage.width, rosImage.height ), cv.IPL_DEPTH_8U, 3 )
            cv.SetData( curImage, rosImage.data, rosImage.step )
                
            curImageGray = cv.CreateImage( ( rosImage.width, rosImage.height ), cv.IPL_DEPTH_8U, 1 )
            cv.CvtColor( curImage, curImageGray, cv.CV_RGB2GRAY )
                
            # Look for optical flow between this image and the last one
            self.opticalFlowX, self.opticalFlowY = self.opticalFlowFilter.calcOpticalFlow( curImageGray )
             
            if not self.dataBuffersSetup:
                self.attemptToSetupDataBuffers()
                
            self.opticalFlowBufferX[ :, :, self.curSampleIdx ] = self.opticalFlowX
            self.opticalFlowBufferY[ :, :, self.curSampleIdx ] = self.opticalFlowY
            self.curSampleIdx = (self.curSampleIdx + 1)%self.numBufferSamples
            
            #t1 = time.time()
            
            ## Check for correlated movement in the X direction
            #groups = np.multiply( np.ones( 
                #shape=(self.opticalFlowBufferX.shape[0],self.opticalFlowBufferX.shape[1]), 
                #dtype=np.int32 ), -1 )
                
            #NO_MOVEMENT_GROUP = 0
            #nextGroupIdx = 1
            #numBlocks = self.opticalFlowBufferX.shape[0]*self.opticalFlowBufferX.shape[1]
            
            #numSamples = self.opticalFlowBufferX.shape[ 2 ]
            #squareSumArray = np.add.reduce( np.square( self.opticalFlowBufferX ), axis=2 )
            #sumArray = np.add.reduce( self.opticalFlowBufferX, axis=2 )
            #meanArray = np.divide( sumArray, numSamples )
            #varArray = np.subtract( 
                #np.divide( squareSumArray, numSamples ),
                #np.square( meanArray ) )
            
            ##print np.max( varArray )
            ##print self.opticalFlowBufferX[ varArray > 15.0 ]
            #for blockIdx in range( numBlocks ):
                
                #rowIdx = int( blockIdx / self.opticalFlowBufferX.shape[1] )
                #colIdx = blockIdx%self.opticalFlowBufferX.shape[1]
                    
                #if groups[ rowIdx, colIdx ] == -1:
                    
                    
                    #flow = self.opticalFlowBufferX[ rowIdx, colIdx, : ]
                    
                    ##mean = np.sum( flow )/len( flow )
                    #var = varArray[ rowIdx, colIdx ]
                    
                    #if var < 10.0:
                        #groups[ rowIdx, colIdx ] = NO_MOVEMENT_GROUP
                    #else:  
                        #curGroupIdx = nextGroupIdx
                        #groups[ rowIdx, colIdx ] = curGroupIdx
                        #nextGroupIdx += 1
                        
                        #mulArray = np.add.reduce( np.multiply( self.opticalFlowBufferX, flow ), axis=2 )

                        #corCoeffArray = np.subtract( 
                            #np.divide( mulArray, numSamples ), np.multiply( meanArray, meanArray[ rowIdx, colIdx ] ) ) 
                        #corCoeffArray = np.divide( corCoeffArray,
                            #np.sqrt( np.multiply( varArray, var ) ) )

                        #absCoeff = np.abs( corCoeffArray )
                        #groups[ np.logical_and( varArray > 10.0, 
                            #np.logical_and( corCoeffArray > 0.01, groups == -1 ) ) ] = curGroupIdx

                        ## Find all blocks with correlated movement
                        ##for otherBlockIdx in range( blockIdx + 1, numBlocks ):
                            ##otherRowIdx = int( otherBlockIdx / self.opticalFlowBufferX.shape[1] )
                            ##otherColIdx = otherBlockIdx%self.opticalFlowBufferX.shape[1]
                            
                            ### Only look at blocks which aren't already part of a group
                            ##if groups[ otherRowIdx, otherColIdx ] == -1:
                                
                                ##if corCoeffArray[ otherRowIdx, otherColIdx ] > 0.95:
                                    ##groups[ otherRowIdx, otherColIdx ] = curGroupIdx
                                ##otherFlow = self.opticalFlowBufferX[ otherRowIdx, otherColIdx, : ]
                                ##if np.corrcoef( flow, otherFlow )[ 0, 1 ] > 0.95:
                                    ##groups[ otherRowIdx, otherColIdx ] = curGroupIdx
              
            #t2 = time.time()
            #print "FindCorr", t2 - t1
              
            #def colourBlock( width, height, r, g, b ):
                #block = np.tile( np.array( [ r, g, b ], dtype=np.uint8 ), ( width, height ) )
                #block.shape = ( width, height, 3 )
                #return block
            
            #t1 = time.time()
              
            ## Now translate groups back into an image
            #availableColours = [
                #colourBlock( self.OPTICAL_FLOW_BLOCK_WIDTH, self.OPTICAL_FLOW_BLOCK_HEIGHT, 255, 0, 0 ),
                #colourBlock( self.OPTICAL_FLOW_BLOCK_WIDTH, self.OPTICAL_FLOW_BLOCK_HEIGHT, 0, 255, 0 ),
                #colourBlock( self.OPTICAL_FLOW_BLOCK_WIDTH, self.OPTICAL_FLOW_BLOCK_HEIGHT, 0, 0, 255 ) ]
            #BLACK = colourBlock( self.OPTICAL_FLOW_BLOCK_WIDTH, self.OPTICAL_FLOW_BLOCK_HEIGHT, 0, 0, 0 )
            
            npImage = np.fromstring( rosImage.data, dtype=np.uint8 )
            npImage.shape = ( rosImage.height, rosImage.width, 3 )
            
            #numGroups = nextGroupIdx
            #groupColours = [ i - 1 for i in range( numGroups ) ]
            #neededColours = numGroups - 1
            #numAvailableColours = len( availableColours )
            #if neededColours > numAvailableColours:
                #print "Needed {0} extra colours".format( neededColours - numAvailableColours )
            
            #for rowIdx in range( self.opticalFlowBufferX.shape[0] ):
                #for colIdx in range( self.opticalFlowBufferX.shape[1] ):
                    
                    #groupIdx = groups[ rowIdx, colIdx ]
                    #if groupIdx > 0:
                        #if groupIdx >= numGroups: #len( groupColours ):
                            #print rowIdx, colIdx, numGroups, groupIdx
                        #colourIdx = groupColours[ groupIdx ]
                        #if colourIdx < numAvailableColours:
                    
                            #leftX = colIdx*self.OPTICAL_FLOW_BLOCK_WIDTH
                            #rightX = leftX + self.OPTICAL_FLOW_BLOCK_WIDTH
                            #topY = rowIdx*self.OPTICAL_FLOW_BLOCK_HEIGHT
                            #bottomY = topY + self.OPTICAL_FLOW_BLOCK_HEIGHT
                            
                            ##print npImage[ topY:bottomY, leftX:rightX, : ].shape, availableColours[ colourIdx ].shape
                            #npImage[ topY:bottomY, leftX:rightX, : ] = availableColours[ colourIdx ]
            
            #t2 = time.time()
            #print "Colour", t2 - t1
            
            
            
            #if self.oldColours == None:
                ##(self.oldColours, label) = vq.kmeans2( npImage, k=10 )
                #print "Ra"
                #self.oldColours = np.array(
                    #[ [ i*64, i*64, i*64 ] for i in range( 255/64 ) ], dtype=np.uint8 ) 
            
            #( code, dist ) = vq.vq( npImage, self.oldColours )
            
            #newImage = np.ndarray( shape = npImage.shape, dtype=np.uint8 )
            
            #newColours = np.array( self.oldColours, dtype=np.uint8 )
            #for i in range( newImage.shape[ 0 ] ):
                #newImage[ i ] = newColours[ code[ i ] ]
            #np.choose( label, np.array( centroid, dtype=np.uint8 ) )
            #print np.array( centroid, dtype=np.uint8 )
            
            if self.npAcc == None:
                self.npAcc = np.ndarray( shape=(10,npImage.shape[0],npImage.shape[1],npImage.shape[2]), dtype=np.uint8 )
                self.accIdx = 0
                
            self.npAcc[ self.accIdx, :, :, : ] = npImage
            self.accIdx = (self.accIdx + 1)%self.npAcc.shape[0]
            npImage = np.array( np.divide( np.add.reduce( self.npAcc ), self.npAcc.shape[0] ), dtype=np.uint8 )
            
            self.cameraImagePixBuf = gtk.gdk.pixbuf_new_from_data( 
                npImage.tostring(), 
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
            self.lastImageGray = curImageGray

        else:
            rospy.logerr( "Unhandled image encoding - " + rosImage.encoding )
    
    #---------------------------------------------------------------------------
    def attemptToSetupDataBuffers( self ):
        
        if self.opticalFlowX != None and self.opticalFlowY != None:
            
            # Optical flow has started so we have enough data to setup our buffers
            self.numBufferSamples = 5
            
            opticalFlowBufferShape = ( self.opticalFlowX.height, self.opticalFlowX.width, self.numBufferSamples )
            self.opticalFlowBufferX = np.zeros(shape=opticalFlowBufferShape, dtype=np.float32)
            self.opticalFlowBufferY = np.zeros(shape=opticalFlowBufferShape, dtype=np.float32)
        
            self.curSampleIdx = 0
            self.dataBuffersSetup = True
    
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
