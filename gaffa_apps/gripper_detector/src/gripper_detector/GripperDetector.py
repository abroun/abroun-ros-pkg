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
import gaffa_teleop.Homog3D
import cv

import yaml
import pygtk
pygtk.require('2.0')
import gtk
import gobject

import gaffa_teleop.LynxmotionArmDescription
from lynxmotion_arm.SSC32Driver import ServoConfig
from gaffa_teleop.RoboticArm import DHFrame, RoboticArm
import sensor_msgs.msg

from OpticalFlowFilter import OpticalFlowFilter

def printTiming(func):
    def wrapper(*arg):
    
        t1 = time.time()
        res = func(*arg)
        t2 = time.time()
        
        print '%s took %0.3f ms' % (func.func_name, (t2-t1)*1000.0)
        return res

    return wrapper
    
#-------------------------------------------------------------------------------
# Calculates the cross correlation of a sequence for all possible lags
def crossCorrelateComplete( sequence, laggedSequence, maxLag = None ):

    if maxLag != None:
        numCorrelations = maxLag + 1
    else:
        numCorrelations = len( sequence )

    # x
    x = sequence
    sum_x = np.add.accumulate( x[ ::-1 ] )[ ::-1 ] 
    sum_x2 = np.add.accumulate( np.square( x )[ ::-1 ] )[ ::-1 ] 
    len_x = np.arange( len( x ), 0, -1 )

    #var_x = np.divide( np.subtract( np.multiply( sum_x2, len_x ), np.square( sum_x ) ), np.square( len_x ) )
    var_x = np.subtract( np.multiply( sum_x2, len_x ), np.square( sum_x ) )

    # y
    y = laggedSequence

    sum_y = np.add.accumulate( y )
    sum_y2 = np.add.accumulate( np.square( y ) )
    len_y = np.arange( 1, len( y ) + 1, 1 )

    #var_y = np.divide( np.subtract( np.multiply( sum_y2, len_y ), np.square( sum_y ) ), np.square( len_y ) )
    var_y = np.subtract( np.multiply( sum_y2, len_y ), np.square( sum_y ) )

    # xy
    sum_xy = np.correlate( x, y, mode="full" )[ len( x ) - 1: ]
    #cov_xy = np.divide( np.subtract( np.multiply( sum_xy, len_x ), np.multiply( sum_x, sum_y[::-1] ) ), np.square( len_x ) )
    cov_xy = np.subtract( np.multiply( sum_xy[ : numCorrelations], len_x[ : numCorrelations] ), np.multiply( sum_x[ : maxLag+1], sum_y[::-1][ : numCorrelations] ) )

    corrCoeff = np.divide( cov_xy, np.sqrt( np.multiply( var_x[ : numCorrelations], var_y[ ::-1 ][ : numCorrelations] ) ) )
    corrCoeff[ np.logical_or( np.isnan( corrCoeff ), np.isinf( corrCoeff ) ) ] = 0.0
    
    return corrCoeff
 
#-------------------------------------------------------------------------------   
def isInputSignalPresent( maxCorrCoeffX, maxCorrCoeffY ):
    
    return ( maxCorrCoeffX > 0.5 or maxCorrCoeffY > 0.5 \
        or maxCorrCoeffX + maxCorrCoeffY > 1.2 )

#-------------------------------------------------------------------------------
def getArmServoConfig():
    
    # Pulls the servo configuration for the arm from the parameter server
    # and returns it as a dictionary of ServoConfigs
    servoConfigData = rospy.get_param( "/ssc32_server/servoConfigData" )
    
    # Move the data into servo config objects
    servoConfigDict = {}
    for servoName in servoConfigData:
        servoConfigDict[ servoName ] = ServoConfig( **servoConfigData[ servoName ] )
        
    return servoConfigDict

#-------------------------------------------------------------------------------
class ArmPos:
    def __init__( self, servoAnglesDict, screenPos ):
        self.servoAnglesDict = servoAnglesDict
        self.screenPos = screenPos

#-------------------------------------------------------------------------------
class MainWindow:
    
    OPTICAL_FLOW_BLOCK_WIDTH = 8
    OPTICAL_FLOW_BLOCK_HEIGHT = 8
    OPTICAL_FLOW_RANGE_WIDTH = 8    # Range to look outside of a block for motion
    OPTICAL_FLOW_RANGE_HEIGHT = 8
    
    GRIPPER_WAVE_FREQUENCY = 1.0    # Waves per second
    GRIPPER_NUM_WAVES = 3.0
    GRIPPER_WAVE_AMPLITUDE = math.radians( 20.0 )/4.0
    
    BUFFER_TIME_LENGTH = 10.0
    SAMPLES_PER_SECOND = 15.0
    MAX_CORRELATION_LAG = 1.0
    GRIPPER_DETECTION_TIME = 9.0    # Should be less than BUFFER_TIME_LENGTH
    GRIPPER_DETECTION_WAVE_START_TIME = 3.0
    
    TEST_ARM_POSITIONS = [
        ArmPos( {
            "base_rotate" : math.radians( 72.65 ), 
            "shoulder_rotate" : math.radians( 75.99 ), 
            "elbow_rotate" : math.radians( -97.34 ), 
            "wrist_rotate" : math.radians( 80.05 ),
            "gripper_rotate" : math.radians( -0.03 ) },
            ( 100, 85 ) ),
        ArmPos( {
            "base_rotate" : math.radians( 85.0 ), 
            "shoulder_rotate" : math.radians( 72.59 ), 
            "elbow_rotate" : math.radians( -107.21 ), 
            "wrist_rotate" : math.radians( 87.14 ),
            "gripper_rotate" : math.radians( -67.23 ) },
            ( 97, 170 ) ),
        ArmPos( {
            "base_rotate" : math.radians( 0.0 ), 
            "shoulder_rotate" : math.radians( 57.32 ), 
            "elbow_rotate" : math.radians( -116.48 ), 
            "wrist_rotate" : math.radians( 130.12 ),
            "gripper_rotate" : math.radians( 0.00 ) },
            ( 247, 165 ) ) ]
     
    # Locating Positions states
    LPS_MOVING_TO_TEST_POS = "MovingToTestPos"
    LPS_DETECTING_GRIPPER = "DetectingGripper"
 
    #---------------------------------------------------------------------------
    def __init__( self ):
    
        scriptPath = os.path.dirname( __file__ )
        self.cameraImagePixBuf = None
        self.servoConfigDict = getArmServoConfig()
        self.lastImageGray = None
        self.opticalFlowX = None
        self.opticalFlowY = None
        self.wavingGripper = False
        self.tryingToDetectGripper = False
        self.gripperWaveStartTime = None
        self.lastImage = None
        self.curTestPosIdx = 0
        self.locatingPositions = False
        
        self.wristAngle = 0.0
            
        # Connect to the robot via ROS
        #rospy.init_node( 'GripperDetector', anonymous=True )
        
        # TODO: Move arm to all dictionary
        servoConfigList = [ self.servoConfigDict[ servoName ] for servoName in self.servoConfigDict ]
        self.roboticArm = RoboticArm( gaffa_teleop.LynxmotionArmDescription.ARM_DH_PROXIMAL, servoConfigList )
        
        self.cameraImageTopic = rospy.Subscriber( "/camera/image", 
            sensor_msgs.msg.Image, self.cameraImageCallback )
            
        self.opticalFlowFilter = OpticalFlowFilter(
            self.OPTICAL_FLOW_BLOCK_WIDTH, self.OPTICAL_FLOW_BLOCK_HEIGHT,
            self.OPTICAL_FLOW_RANGE_WIDTH, self.OPTICAL_FLOW_RANGE_HEIGHT )
            
        self.dataBuffersSetup = False
        self.opticalFlowSampleIdx = None
        self.inputSignalDetectedArray = None
        self.gripperHistogram = None
        self.gripperTrackWindow = None
        
        # Setup the GUI        
        builder = gtk.Builder()
        builder.add_from_file( scriptPath + "/GUI/GripperDetector.glade" )
        
        self.window = builder.get_object( "winMain" )   
        self.dwgCameraImage = builder.get_object( "dwgCameraImage" )
        self.btnDetectGripper = builder.get_object( "btnDetectGripper" )
        self.checkShowOpticalFlow = builder.get_object( "checkShowOpticalFlow" )
        self.checkShowDetectedMotionBlocks = builder.get_object( "checkShowDetectedMotionBlocks" )
        self.checkShowCAMShiftTracking = builder.get_object( "checkShowCAMShiftTracking" )
        self.checkShowGripperProbability = builder.get_object( "checkShowGripperProbability" )
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
            
            # Create an OpenCV image to process the data
            curImage = cv.CreateImageHeader( ( rosImage.width, rosImage.height ), cv.IPL_DEPTH_8U, 3 )
            cv.SetData( curImage, rosImage.data, rosImage.step )
            
            curImageGray = cv.CreateImage( ( rosImage.width, rosImage.height ), cv.IPL_DEPTH_8U, 1 )
            
            if rosImage.encoding == "bgr8":
                cv.CvtColor( curImage, curImageGray, cv.CV_BGR2GRAY )
            else:
                cv.CvtColor( curImage, curImageGray, cv.CV_RGB2GRAY )
            
            # Look for optical flow between this image and the last one
            self.opticalFlowX, self.opticalFlowY = self.opticalFlowFilter.calcOpticalFlow( curImageGray )
            
            # Use CAMShift to track the gripper
            gripperProbabilityImage = None
            if self.gripperHistogram != None and self.gripperTrackWindow != None:
                
                imageRGB = cv.CloneImage( curImage )
                    
                r_plane = cv.CreateMat( imageRGB.height, imageRGB.width, cv.CV_8UC1 )
                g_plane = cv.CreateMat( imageRGB.height, imageRGB.width, cv.CV_8UC1 )
                b_plane = cv.CreateMat( imageRGB.height, imageRGB.width, cv.CV_8UC1 )
                cv.Split( imageRGB, r_plane, g_plane, b_plane, None )
                planes = [ r_plane, g_plane, b_plane ]
                
                backproject = cv.CreateImage(cv.GetSize(imageRGB), 8, 1)

                # Run the cam-shift
                cv.CalcArrBackProject( planes, backproject, self.gripperHistogram )
                
                if self.gripperTrackWindow[ 2 ] > 0 and self.gripperTrackWindow[ 3 ] > 0:
                    crit = ( cv.CV_TERMCRIT_EPS | cv.CV_TERMCRIT_ITER, 10, 1)
                    (iters, (area, value, rect), track_box) = cv.CamShift(backproject, self.gripperTrackWindow, crit)
                    self.gripperTrackWindow = rect
                
                #print self.gripperTrackWindow
                
                if self.checkShowGripperProbability.get_active():
                    #cv.Threshold( backproject, backproject, 1, 255, cv.CV_THRESH_BINARY )
                    cv.Threshold( backproject, backproject, 128, 255, cv.CV_THRESH_TOZERO )
                    
                    cv.CvtColor( backproject, imageRGB, cv.CV_GRAY2RGB )
                    gripperProbabilityImage = imageRGB
            
            # Display the image
            if gripperProbabilityImage != None:
                self.cameraImagePixBuf = gtk.gdk.pixbuf_new_from_data( 
                    gripperProbabilityImage.tostring(), 
                    gtk.gdk.COLORSPACE_RGB,
                    False,
                    8,
                    gripperProbabilityImage.width,
                    gripperProbabilityImage.height,
                    gripperProbabilityImage.width*3 )
            else:
                self.cameraImagePixBuf = gtk.gdk.pixbuf_new_from_data( 
                    rosImage.data, 
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
            self.lastImage = curImage
            
            # Check to see if we did everything fast enough
            #if self.dataBuffersSetup:
                #testSampleIdx = self.curSampleIdx
                
                #if self.opticalFlowSampleIdx == None:
                    #self.opticalFlowSampleIdx = testSampleIdx
                #else:
                    ## We expect the sample idx to have advanced by 1 since we were last here
                    #sampleIdxDiff = testSampleIdx - self.opticalFlowSampleIdx
                    #if sampleIdxDiff > 1:
                        #print "Missed {0} samples".format( sampleIdxDiff - 1 )
                    #elif sampleIdxDiff < 1:
                        #print "Not sampling fast enough"
                    
                    #self.opticalFlowSampleIdx = testSampleIdx

        else:
            rospy.logerr( "Unhandled image encoding - " + rosImage.encoding )
    
    #---------------------------------------------------------------------------
    def moveArmToJointAnglePosition( self, servoAnglesDict, jointSpeed ):
        
        self.wristAngle = servoAnglesDict[ "wrist_rotate" ]
        self.roboticArm.setJointAngles( servoAnglesDict, jointSpeed )
    
    #---------------------------------------------------------------------------
    def onBtnGotoSafePosClicked( self, widget, data = None ):
        
        self.tryingToDetectGripper = False
        self.locatingPositions = False
        
        servoAnglesDict = {
            "base_rotate" : math.radians( 72.65 ), 
            "shoulder_rotate" : 2.3561944901923448, 
            "elbow_rotate" : -2.748893571891069, 
            "wrist_rotate" : 1.9583014768641922,
            "gripper_rotate" : -0.0004890796739715704
        }

        self.moveArmToJointAnglePosition( servoAnglesDict, math.radians( 1.5 ) )
    
    #---------------------------------------------------------------------------
    def onBtnGotoExperimentPosClicked( self, widget, data = None ):
  
        self.tryingToDetectGripper = False
        self.locatingPositions = False
        
        servoAnglesDict = {
            "base_rotate" : math.radians( 72.65 ), 
            "shoulder_rotate" : 1.8248153311815414, 
            "elbow_rotate" : -2.0368307791722984, 
            "wrist_rotate" : 1.2301417017068466,
            "gripper_rotate" : -0.0004890796739715704
        }
        
        self.moveArmToJointAnglePosition( servoAnglesDict, math.radians( 1.5 ) )
    
    #---------------------------------------------------------------------------
    def onBtnWaveGripperClicked( self, widget, data = None ):
        
        self.tryingToDetectGripper = False
        self.locatingPositions = False
        
        self.tryToStartWavingGripper()

    #---------------------------------------------------------------------------
    def startToDetectGripper( self ):
        
        self.inputSignalDetectedArray = None
        self.gripperHistogram = None
        self.gripperTrackWindow = None
        self.gripperDetectionStartSampleIdx = self.curSampleIdx
        self.gripperDetectionWaveStarted = False
        self.btnDetectGripper.set_sensitive( False )
        self.tryingToDetectGripper = True
        
    #---------------------------------------------------------------------------
    def onBtnDetectGripperClicked( self, widget, data = None ):
        
        if self.dataBuffersSetup \
            and not self.tryingToDetectGripper \
            and not self.wavingGripper:
            
            self.locatingPositions = False
            self.startToDetectGripper()
     
    #---------------------------------------------------------------------------
    def onBtnLocatePositionsClicked( self, widget, data = None ):
        
        if self.dataBuffersSetup \
            and not self.locatingPositions \
            and not self.tryingToDetectGripper \
            and not self.wavingGripper:
                
            self.curTestPosIdx = 0
            self.testPosMeasurements = []
            for i in range( len( self.TEST_ARM_POSITIONS ) ):
                self.testPosMeasurements.append( [] )
                
            self.locatingPositions = True
            self.moveToTestPosition( self.curTestPosIdx )
     
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
              
            # Draw an overlay to show places where the input motion has been detected
            if self.checkShowDetectedMotionBlocks.get_active() \
                and self.inputSignalDetectedArray != None:
                    
                graphicsContext = widget.window.new_gc()
                graphicsContext.set_rgb_fg_color( gtk.gdk.Color( 65535, 65535, 0 ) )
                
                blockY = imgRect.y
                for y in range( self.inputSignalDetectedArray.shape[ 0 ] ):
                
                    blockX = imgRect.x
                    for x in range( self.inputSignalDetectedArray.shape[ 1 ] ):
                        
                        if self.inputSignalDetectedArray[ y, x ]:
                            points = [ (blockX+int((i*2)%self.OPTICAL_FLOW_BLOCK_WIDTH), blockY+2*int((i*2)/self.OPTICAL_FLOW_BLOCK_WIDTH)) \
                                for i in range( self.OPTICAL_FLOW_BLOCK_WIDTH*self.OPTICAL_FLOW_BLOCK_HEIGHT/4 ) ]
                                
                            widget.window.draw_points( graphicsContext, points )
                            
                        blockX += self.OPTICAL_FLOW_BLOCK_WIDTH
                        
                    blockY += self.OPTICAL_FLOW_BLOCK_HEIGHT
                
            # Draw the optical flow if it's available
            if self.checkShowOpticalFlow.get_active() \
                and self.opticalFlowX != None and self.opticalFlowY != None:
            
                graphicsContext = widget.window.new_gc()
                graphicsContext.set_rgb_fg_color( gtk.gdk.Color( 0, 65535, 0 ) )
                
                blockCentreY = self.OPTICAL_FLOW_BLOCK_HEIGHT / 2
                for y in range( self.opticalFlowX.shape[ 0 ] ):
                
                    blockCentreX = self.OPTICAL_FLOW_BLOCK_WIDTH / 2
                    for x in range( self.opticalFlowX.shape[ 1 ] ):
                        
                        endX = blockCentreX + cv.Get2D( self.opticalFlowX, y, x )[ 0 ]
                        endY = blockCentreY + cv.Get2D( self.opticalFlowY, y, x )[ 0 ]
                        
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
                   
            # Draw the CAMShift tracking window
            if self.checkShowCAMShiftTracking.get_active() \
                and self.gripperTrackWindow != None:
                    
                graphicsContext = widget.window.new_gc()
                graphicsContext.set_rgb_fg_color( gtk.gdk.Color( 65535, 65535/2, 0 ) )
                    
                widget.window.draw_rectangle( graphicsContext, False,
                    self.gripperTrackWindow[ 0 ], self.gripperTrackWindow[ 1 ],
                    self.gripperTrackWindow[ 2 ], self.gripperTrackWindow[ 3 ] )
                    
            # Draw the results of trying to locate positions on the screen using
            # the detect gripper routine
            if self.locatingPositions:
                
                realWorldPoints = [ p.screenPos for p in self.TEST_ARM_POSITIONS ]
                measuredPoints = []
                averagePoints = []
                
                for posIdx in range( len( self.TEST_ARM_POSITIONS ) ):
                    posMeasuredPoints = self.testPosMeasurements[ posIdx ]
                    numMeasuredPoints = len( posMeasuredPoints )
                    
                    if numMeasuredPoints > 0:
                        measuredPoints += posMeasuredPoints
                        
                        accX = 0
                        accY = 0
                        for p in posMeasuredPoints:
                            accX += p[ 0 ]
                            accY += p[ 1 ]
                        averagePoints.append( ( int( accX / numMeasuredPoints ), int( accY / numMeasuredPoints ) ) )
                
                graphicsContext = widget.window.new_gc()
                graphicsContext.set_rgb_fg_color( gtk.gdk.Color( 65535, 65535, 65535 ) )
                self.drawCircles( widget, graphicsContext, realWorldPoints, 2, True )
                
                graphicsContext.set_rgb_fg_color( gtk.gdk.Color( 0, 65535, 0 ) )
                self.drawCircles( widget, graphicsContext, measuredPoints, 2, True )
                
                graphicsContext.set_rgb_fg_color( gtk.gdk.Color( 65535, 0, 0 ) )
                self.drawCircles( widget, graphicsContext, averagePoints, 2, True )

    #---------------------------------------------------------------------------
    def drawCircles( self, widget, graphicsContext, circleCentres, radius, filled ):
        
        for circleCentre in circleCentres:
            
            arcX = int( circleCentre[ 0 ] - radius )
            arcY = int( circleCentre[ 1 ] - radius )
            arcWidth = arcHeight = int( radius * 2 )
        
            widget.window.draw_arc( graphicsContext, 
                filled, arcX, arcY, arcWidth, arcHeight, 0, 360 * 64 )

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
    def moveToTestPosition( self, testPosIdx ):
        
        if not self.locatingPositions:
            raise Exception( "Not in correct state" )
        
        self.moveArmToJointAnglePosition( 
            self.TEST_ARM_POSITIONS[ testPosIdx ].servoAnglesDict, 
            math.radians( 1.5 ) )
            
        self.curTestPosIdx = testPosIdx
        self.locatingPositionsState = self.LPS_MOVING_TO_TEST_POS
        self.moveStartTime = time.clock()

    #---------------------------------------------------------------------------
    def update( self ):

        UPDATE_FREQUENCY = self.SAMPLES_PER_SECOND    # Updates in Hz

        lastTime = time.clock()

        while 1:
            
            curTime = time.clock()
            
            if curTime - lastTime >= 1.0 / UPDATE_FREQUENCY:
            
                lastGripperDisplacement = 0.0
            
                # Update the wave if active
                if self.wavingGripper:
                    
                    waveTime = curTime - self.gripperWaveStartTime
                    totalWaveTime = self.GRIPPER_NUM_WAVES / self.GRIPPER_WAVE_FREQUENCY
                    waveFinished = False
                    
                    if waveTime >= totalWaveTime:
                        
                        waveTime = totalWaveTime
                        waveFinished = True
                        
                    # Work out the current displacement from the initial position
                    displacement = self.GRIPPER_WAVE_AMPLITUDE \
                        * math.sin( waveTime*self.GRIPPER_WAVE_FREQUENCY*2.0*math.pi )
                    lastGripperDisplacement = displacement
                    
                    servoAnglesDict = { "wrist_rotate" : self.wristAngle + displacement }
                    
                    self.roboticArm.setJointAngles( servoAnglesDict )
                    if waveFinished:
                        self.wavingGripper = False
                        
                # Update sample buffers with input and output data
                if not self.dataBuffersSetup:
                    self.attemptToSetupDataBuffers()
                else:
                    
                    # Store the data.
                    # NOTE: This all falls apart if we can't process the data fast enough
                    self.gripperAngleBuffer[ self.curSampleIdx ] = lastGripperDisplacement
                    self.opticalFlowBufferX[ :, :, self.curSampleIdx ] = self.opticalFlowX
                    self.opticalFlowBufferY[ :, :, self.curSampleIdx ] = self.opticalFlowY
                    
                    self.curSampleIdx = (self.curSampleIdx + 1)%self.numBufferSamples
                    
                    # Update gripper detection if it's on-going
                    if self.tryingToDetectGripper:
                        maxNumGripperDetectionSamples = int( self.GRIPPER_DETECTION_TIME*self.SAMPLES_PER_SECOND )
                        gripperDetectionWaveStartSampleIdx = int( self.GRIPPER_DETECTION_WAVE_START_TIME*self.SAMPLES_PER_SECOND )
                        
                        if self.curSampleIdx > self.gripperDetectionStartSampleIdx:
                            gripperDetectionSampleIdx = self.curSampleIdx - self.gripperDetectionStartSampleIdx
                        else:
                            gripperDetectionSampleIdx = len( self.gripperAngleBuffer ) \
                                - self.gripperDetectionStartSampleIdx + self.curSampleIdx
                            
                        if gripperDetectionSampleIdx >= maxNumGripperDetectionSamples:
                            
                            # Put the collected data in order
                            detectedGripperAngleBuffer = np.ndarray(shape=( maxNumGripperDetectionSamples ), dtype=np.float32)
            
                            opticalFlowBufferShape = ( self.opticalFlowX.shape[ 0 ], self.opticalFlowX.shape[ 1 ], maxNumGripperDetectionSamples )
                            detectedOpticalFlowBufferX = np.ndarray(shape=opticalFlowBufferShape, dtype=np.float32)
                            detectedOpticalFlowBufferY = np.ndarray(shape=opticalFlowBufferShape, dtype=np.float32)
        
                            if self.curSampleIdx > self.gripperDetectionStartSampleIdx:
                                detectedGripperAngleBuffer[:] = self.gripperAngleBuffer[self.gripperDetectionStartSampleIdx:self.curSampleIdx]
                                detectedOpticalFlowBufferX[:,:,:] = self.opticalFlowBufferX[:,:,self.gripperDetectionStartSampleIdx:self.curSampleIdx]
                                detectedOpticalFlowBufferY[:,:,:] = self.opticalFlowBufferY[:,:,self.gripperDetectionStartSampleIdx:self.curSampleIdx]
                            else:
                                numSamples = len( self.gripperAngleBuffer ) - self.gripperDetectionStartSampleIdx
                                detectedGripperAngleBuffer[:numSamples] = self.gripperAngleBuffer[self.gripperDetectionStartSampleIdx:]
                                detectedOpticalFlowBufferX[:,:,:numSamples] = self.opticalFlowBufferX[:,:,self.gripperDetectionStartSampleIdx:]
                                detectedOpticalFlowBufferY[:,:,:numSamples] = self.opticalFlowBufferY[:,:,self.gripperDetectionStartSampleIdx:]
                                
                                detectedGripperAngleBuffer[numSamples:] = self.gripperAngleBuffer[:self.curSampleIdx]
                                detectedOpticalFlowBufferX[:,:,numSamples:] = self.opticalFlowBufferX[:,:,:self.curSampleIdx]
                                detectedOpticalFlowBufferY[:,:,numSamples:] = self.opticalFlowBufferY[:,:,:self.curSampleIdx]
                            
                            # Try to detect the gripper using the data
                            self.detectGripperWithCollectedData( detectedGripperAngleBuffer,
                                detectedOpticalFlowBufferX, detectedOpticalFlowBufferY, self.lastImage )  
                                                      
                            self.tryingToDetectGripper = False
                            self.btnDetectGripper.set_sensitive( True )
                            
                        elif gripperDetectionSampleIdx >= gripperDetectionWaveStartSampleIdx \
                            and not self.gripperDetectionWaveStarted:
                            
                            self.gripperDetectionWaveStarted = self.tryToStartWavingGripper()

                # Update the Locating Positions state machine
                if self.locatingPositions:
                    if self.locatingPositionsState == self.LPS_MOVING_TO_TEST_POS:
                        
                        if curTime - self.moveStartTime > 3.0:
                            # Arm should have got to the position and settled by now
                            self.startToDetectGripper()
                            self.locatingPositionsState = self.LPS_DETECTING_GRIPPER
                            
                    elif self.locatingPositionsState == self.LPS_DETECTING_GRIPPER:
                        
                        if not self.tryingToDetectGripper:
                            
                            # Gripper detection has finished, measure the position of the gripper
                            numDetectedSignals = 0
                            accX = 0
                            accY = 0
                            
                            for rowIdx in range( self.inputSignalDetectedArray.shape[ 0 ] ):
                                
                                rowY = rowIdx*self.OPTICAL_FLOW_BLOCK_HEIGHT + self.OPTICAL_FLOW_BLOCK_HEIGHT/2
                                for colIdx in range( self.inputSignalDetectedArray.shape[ 1 ] ):
                                
                                    colX = colIdx*self.OPTICAL_FLOW_BLOCK_WIDTH + self.OPTICAL_FLOW_BLOCK_WIDTH/2
                                    if self.inputSignalDetectedArray[ rowIdx, colIdx ]:
                                        numDetectedSignals += 1
                                        accX += colX
                                        accY += rowY
                            
                            if numDetectedSignals > 0:
                                measuredPos = ( accX / numDetectedSignals, accY / numDetectedSignals )
                                self.testPosMeasurements[ self.curTestPosIdx ].append( measuredPos )
                                
                            # Now move on to the next test position
                            newTestPosIdx = (self.curTestPosIdx + 1)%len( self.TEST_ARM_POSITIONS )
                            self.moveToTestPosition( newTestPosIdx )

                # Save the time
                lastTime = curTime
                
            yield True
            
        yield False
        
    #---------------------------------------------------------------------------
    def attemptToSetupDataBuffers( self ):
        
        if self.opticalFlowX != None and self.opticalFlowY != None:
            
            # Optical flow has started so we have enough data to setup our buffers
            self.numBufferSamples = int( self.BUFFER_TIME_LENGTH*self.SAMPLES_PER_SECOND )
            
            self.gripperAngleBuffer = np.ndarray(shape=( self.numBufferSamples ), dtype=np.float32)
            
            opticalFlowBufferShape = ( self.opticalFlowX.shape[ 0 ], self.opticalFlowX.shape[ 1 ], self.numBufferSamples )
            self.opticalFlowBufferX = np.ndarray(shape=opticalFlowBufferShape, dtype=np.float32)
            self.opticalFlowBufferY = np.ndarray(shape=opticalFlowBufferShape, dtype=np.float32)
        
            self.curSampleIdx = 0
            self.dataBuffersSetup = True
            
    #---------------------------------------------------------------------------
    def tryToStartWavingGripper( self ):
        
        waveStarted = False
        if self.wavingGripper == False:
            self.gripperWaveStartTime = time.clock()
            self.wavingGripper = True
            waveStarted = True
            
        return waveStarted
        
    #---------------------------------------------------------------------------
    def detectGripperWithCollectedData( self, detectedGripperAngleBuffer,
        detectedOpticalFlowBufferX, detectedOpticalFlowBufferY, imageRGB ):
            
        t1 = time.time()
        
        maxLag = int( self.MAX_CORRELATION_LAG * self.SAMPLES_PER_SECOND )
        correlationsX = np.apply_along_axis( crossCorrelateComplete, 2, 
            detectedOpticalFlowBufferX, detectedGripperAngleBuffer, maxLag )
        correlationsY = np.apply_along_axis( crossCorrelateComplete, 2, 
            detectedOpticalFlowBufferY, detectedGripperAngleBuffer, maxLag )
        
        t2 = time.time()
        print 'Correlation took %0.3f ms' % ((t2-t1)*1000.0)
        
        # Detect the input signal based on the correlation in the x and y axis
        maxCorrelationArrayX = np.maximum.reduce( np.absolute( correlationsX ), axis=2 )
        maxCorrelationArrayY = np.maximum.reduce( np.absolute( correlationsY ), axis=2 )
        self.inputSignalDetectedArray = np.frompyfunc( isInputSignalPresent, 2, 1 )(
            maxCorrelationArrayX, maxCorrelationArrayY )
          
        # Build a histogram for the gripper  
        self.gripperHistogram = cv.CreateHist( 
            [ 256/8, 256/8, 256/8 ], cv.CV_HIST_ARRAY, [ (0,255), (0,255), (0,255) ], 1 )
            
        r_plane = cv.CreateMat( imageRGB.height, imageRGB.width, cv.CV_8UC1 )
        g_plane = cv.CreateMat( imageRGB.height, imageRGB.width, cv.CV_8UC1 )
        b_plane = cv.CreateMat( imageRGB.height, imageRGB.width, cv.CV_8UC1 )
        cv.Split( imageRGB, r_plane, g_plane, b_plane, None )
        planes = [ r_plane, g_plane, b_plane ]

        maskArray = np.zeros(shape=( imageRGB.height, imageRGB.width ), dtype=np.uint8 )
        for rowIdx in range( self.inputSignalDetectedArray.shape[ 0 ] ):
            for colIdx in range( self.inputSignalDetectedArray.shape[ 1 ] ):
                
                if self.inputSignalDetectedArray[ rowIdx, colIdx ]:
                    rowStartIdx = rowIdx*self.OPTICAL_FLOW_BLOCK_HEIGHT
                    rowEndIdx = rowStartIdx + self.OPTICAL_FLOW_BLOCK_HEIGHT
                    colStartIdx = colIdx*self.OPTICAL_FLOW_BLOCK_WIDTH
                    colEndIdx = colStartIdx + self.OPTICAL_FLOW_BLOCK_WIDTH
                    
                    maskArray[ rowStartIdx:rowEndIdx, colStartIdx:colEndIdx ] = 255

        cv.CalcHist( [ cv.GetImage( i ) for i in planes ], 
            self.gripperHistogram, 0, mask=maskArray )
            
        minX = 1000
        maxX = 0
        minY = 1000
        maxY = 0
        numMotionBlocks = 0
        
        # Create the track window from the blocks where motion was detected
        for rowIdx in range( self.inputSignalDetectedArray.shape[ 0 ] ):
            for colIdx in range( self.inputSignalDetectedArray.shape[ 1 ] ):
                
                if self.inputSignalDetectedArray[ rowIdx, colIdx ]:
                    
                    if rowIdx < minY: minY = rowIdx
                    if rowIdx > maxY: maxY = rowIdx
                    if colIdx < minX: minX = colIdx
                    if colIdx > maxX: maxX = colIdx
                    numMotionBlocks += 1
                    
        if numMotionBlocks > 0:
            windowX = minX*self.OPTICAL_FLOW_BLOCK_WIDTH
            windowY = minY*self.OPTICAL_FLOW_BLOCK_HEIGHT
            windowWidth = (maxX+1-minX)*self.OPTICAL_FLOW_BLOCK_WIDTH
            windowHeight = (maxY+1-minY)*self.OPTICAL_FLOW_BLOCK_HEIGHT
        
            self.gripperTrackWindow = ( windowX, windowY, windowWidth, windowHeight )
        
#-------------------------------------------------------------------------------
if __name__ == "__main__":

    mainWindow = MainWindow()
    mainWindow.main()

# Connect to camera and robot via ROS

# Display camera image

# Wait until robot and camera are ready

# Order arm to move to safe position - wait

# Order arm to move to experiment position - wait

# Waggle arm
