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

def printTiming(func):
    def wrapper(*arg):
    
        t1 = time.time()
        res = func(*arg)
        t2 = time.time()
        
        print '%s took %0.3f ms' % (func.func_name, (t2-t1)*1000.0)
        return res

    return wrapper

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
class MainWindow:
    
    OPTICAL_FLOW_BLOCK_WIDTH = 16
    OPTICAL_FLOW_BLOCK_HEIGHT = 16
    OPTICAL_FLOW_RANGE_WIDTH = 8    # Range to look outside of a block for motion
    OPTICAL_FLOW_RANGE_HEIGHT = 8
    
    GRIPPER_WAVE_FREQUENCY = 1.0    # Waves per second
    GRIPPER_NUM_WAVES = 3.0
    GRIPPER_WAVE_AMPLITUDE = math.radians( 20.0 )
 
    #---------------------------------------------------------------------------
    def __init__( self ):
    
        scriptPath = os.path.dirname( __file__ )
        self.cameraImagePixBuf = None
        self.servoConfigDict = getArmServoConfig()
        self.lastImageGray = None
        self.opticalFlowX = None
        self.opticalFlowY = None
        self.wavingGripper = False
        self.gripperWaveStartTime = None
        
        self.wristAngle = 0.0
            
        # Connect to the robot via ROS
        #rospy.init_node( 'GripperDetector', anonymous=True )
        
        # TODO: Move arm to all dictionary
        servoConfigList = [ self.servoConfigDict[ servoName ] for servoName in self.servoConfigDict ]
        self.roboticArm = RoboticArm( gaffa_teleop.LynxmotionArmDescription.ARM_DH_PROXIMAL, servoConfigList )
        
        self.cameraImageTopic = rospy.Subscriber( "/camera/image", 
            sensor_msgs.msg.Image, self.cameraImageCallback )
        
        # Setup the GUI        
        builder = gtk.Builder()
        builder.add_from_file( scriptPath + "/GUI/GripperDetector.glade" )
        
        self.dwgCameraImage = builder.get_object( "dwgCameraImage" )
        self.window = builder.get_object( "winMain" )
        
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
    def createOpticalFlowStorage( self ):
        
        if self.lastImageGray == None:
            raise Exception( "No image to measure for optical flow" )
        
        storageWidth = (self.lastImageGray.width - self.OPTICAL_FLOW_BLOCK_WIDTH)/self.OPTICAL_FLOW_BLOCK_WIDTH
        storageHeight = (self.lastImageGray.height - self.OPTICAL_FLOW_BLOCK_HEIGHT)/self.OPTICAL_FLOW_BLOCK_HEIGHT
        
        if self.opticalFlowX == None \
            or storageWidth != self.opticalFlowX.width \
            or storageHeight != self.opticalFlowX.height:
                
            self.opticalFlowX = cv.CreateMat( storageHeight, storageWidth, cv.CV_32FC1 )

        if self.opticalFlowY == None \
            or storageWidth != self.opticalFlowY.width \
            or storageHeight != self.opticalFlowY.height:
                
            self.opticalFlowY = cv.CreateMat( storageHeight, storageWidth, cv.CV_32FC1 )
        
    #---------------------------------------------------------------------------
    #@printTiming
    def calcOpticalFlow( self, curImageGray ):
        if self.lastImageGray != None:
                
            self.createOpticalFlowStorage()
            
            cv.CalcOpticalFlowBM( self.lastImageGray, curImageGray, 
                ( self.OPTICAL_FLOW_BLOCK_WIDTH, self.OPTICAL_FLOW_BLOCK_HEIGHT ),
                ( self.OPTICAL_FLOW_BLOCK_WIDTH, self.OPTICAL_FLOW_BLOCK_HEIGHT ),
                ( self.OPTICAL_FLOW_RANGE_WIDTH, self.OPTICAL_FLOW_RANGE_HEIGHT ),
                0, self.opticalFlowX, self.opticalFlowY )
            
        # Save the current image
        self.lastImageGray = curImageGray
        
    #---------------------------------------------------------------------------
    def cameraImageCallback( self, rosImage ):
        
        if rosImage.encoding == "rgb8":
            
            # Create an OpenCV image to process the data
            curImageRGB = cv.CreateImageHeader( ( rosImage.width, rosImage.height ), cv.IPL_DEPTH_8U, 3 )
            cv.SetData( curImageRGB, rosImage.data, rosImage.step )
            curImageGray = cv.CreateImage( ( rosImage.width, rosImage.height ), cv.IPL_DEPTH_8U, 1 )
            cv.CvtColor( curImageRGB, curImageGray, cv.CV_RGB2GRAY )
            
            # Look for optical flow between this image and the last one
            self.calcOpticalFlow( curImageGray )
            
            # Display the image
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

        else:
            rospy.logerr( "Unhandled image encoding - " + image.encoding )
        
    #---------------------------------------------------------------------------
    def onBtnGotoSafePosClicked( self, widget, data = None ):
        
        servoAnglesDict = {
            "base_rotate" : 0.02389963168290073, 
            "shoulder_rotate" : 2.3561944901923448, 
            "elbow_rotate" : -2.748893571891069, 
            "wrist_rotate" : 1.9583014768641922,
            "gripper_rotate" : -0.0004890796739715704
        }
        
        self.wristAngle = servoAnglesDict[ "wrist_rotate" ]
        
        self.roboticArm.setJointAngles( servoAnglesDict, math.radians( 2.5 ) )
        
    #---------------------------------------------------------------------------
    def onBtnGotoExperimentPosClicked( self, widget, data = None ):
  
        servoAnglesDict = {
            "base_rotate" : 0.43528090983473017, 
            "shoulder_rotate" : 1.8248153311815414, 
            "elbow_rotate" : -2.0368307791722984, 
            "wrist_rotate" : 1.2301417017068466,
            "gripper_rotate" : -0.0004890796739715704
        }
        
        self.wristAngle = servoAnglesDict[ "wrist_rotate" ]
        
        self.roboticArm.setJointAngles( servoAnglesDict, math.radians( 2.5 ) )
    
    #---------------------------------------------------------------------------
    def onBtnWaveGripperClicked( self, widget, data = None ):
        
        if self.wavingGripper == False:
            self.gripperWaveStartTime = time.clock()
            self.wavingGripper = True
      
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
                
            # Draw the optical flow if it's available
            if self.opticalFlowX != None and self.opticalFlowY != None:
            
                graphicsContext = widget.window.new_gc()
                graphicsContext.set_rgb_fg_color( gtk.gdk.Color( 0, 65535, 0 ) )
                
                blockCentreY = self.OPTICAL_FLOW_BLOCK_HEIGHT / 2
                for y in range( self.opticalFlowX.height ):
                
                    blockCentreX = self.OPTICAL_FLOW_BLOCK_WIDTH / 2
                    for x in range( self.opticalFlowX.width ):
                        
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

        UPDATE_FREQUENCY = 30.0    # Updates in Hz

        lastTime = time.clock()

        while 1:
            
            curTime = time.clock()
            
            if curTime - lastTime >= 1.0 / UPDATE_FREQUENCY:
            
                # Update the wave if active
                if self.wavingGripper:
                    
                    waveTime = curTime - self.gripperWaveStartTime
                    totalWaveTime = self.GRIPPER_NUM_WAVES / self.GRIPPER_WAVE_FREQUENCY
                    waveFinished = False
                    
                    print waveTime, totalWaveTime
                    if waveTime >= totalWaveTime:
                        
                        waveTime = totalWaveTime
                        waveFinished = True
                        
                    # Work out the current displacement from the initial position
                    displacement = self.GRIPPER_WAVE_AMPLITUDE \
                        * math.sin( waveTime*self.GRIPPER_WAVE_FREQUENCY*2.0*math.pi )
                    
                    
                    servoAnglesDict = { "wrist_rotate" : self.wristAngle + displacement }
                    
                    self.roboticArm.setJointAngles( servoAnglesDict )
                    if waveFinished:
                        self.wavingGripper = False

                # Save the time
                lastTime = curTime
                
            yield True
            
        yield False
        
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