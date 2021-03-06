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
from Homography.JointScreenHomography import JointScreenHomography





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
    
    # 49, 81 -> 52.05, 15.62, -34.16, 81.33         -  Top Left
    # 218, 76 -> 28.19, 58.89, -105.32, 48.80       - Top Right
    # 283, 135 -> 79.16, 67.70, -119.55, 48.80      - Bottom Right
    # 56, 142 -> 78.61, 29.24, -50.29, 65.60        - Bottom Left
        
    CORNER_JOINT_ARRAYS = [
        [ math.radians( 52.05 ), # Top Left
          math.radians( 15.62 ),
          math.radians( -34.16 ), 
          math.radians( 81.33 ),
          math.radians( 0.0 ) ],
        [ math.radians( 28.19 ), # Top Right
          math.radians( 58.89 ),
          math.radians( -105.32 ), 
          math.radians( 48.80 ),
          math.radians( 0.0 ) ],
        [ math.radians( 79.16 ), # Bottom Right
          math.radians( 67.70 ),
          math.radians( -119.55 ), 
          math.radians( 48.80 ),
          math.radians( 0.0 ) ],
        [ math.radians( 78.61 ), # Bottom Left
          math.radians( 29.24 ),
          math.radians( -50.29 ), 
          math.radians( 65.60 ),
          math.radians( 0.0 ) ] ]
        
    CORNER_SCREEN_POSITIONS = [
        ( 49, 81 ),     # Top Left
        ( 218, 76 ),    # Top Right
        ( 283, 135 ),   # Bottom Right
        ( 56, 142 ) ]   # Bottom Left
 
    #---------------------------------------------------------------------------
    def __init__( self ):
    
        scriptPath = os.path.dirname( __file__ )
        self.cameraImagePixBuf = None
        self.servoConfigDict = getArmServoConfig()
        
        self.wristAngle = 0.0
            
        # Connect to the robot via ROS
        
        # TODO: Move arm to all dictionary
        servoConfigList = [ self.servoConfigDict[ servoName ] for servoName in self.servoConfigDict ]
        self.roboticArm = RoboticArm( gaffa_teleop.LynxmotionArmDescription.ARM_DH_PROXIMAL, servoConfigList )
        
        self.cameraImageTopic = rospy.Subscriber( "/camera/image", 
            sensor_msgs.msg.Image, self.cameraImageCallback )
            
        self.jointScreenHomography = JointScreenHomography( 
            self.CORNER_JOINT_ARRAYS, self.CORNER_SCREEN_POSITIONS, self.roboticArm )
        
        # Setup the GUI        
        builder = gtk.Builder()
        builder.add_from_file( scriptPath + "/GUI/VideoTeleop.glade" )
        
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
            rospy.logerr( "Unhandled image encoding - " + rosImage.encoding )
    
    #---------------------------------------------------------------------------
    def moveArmToJointAnglePosition( self, servoAnglesDict, jointSpeed ):
        
        self.wristAngle = servoAnglesDict[ "wrist_rotate" ]
        self.roboticArm.setJointAngles( servoAnglesDict, jointSpeed )
    
    #---------------------------------------------------------------------------
    def onBtnGotoSafePosClicked( self, widget, data = None ):
        
        servoAnglesDict = {
            "base_rotate" : math.radians( 72.65 ), 
            "shoulder_rotate" : 2.3561944901923448, 
            "elbow_rotate" : -2.748893571891069, 
            "wrist_rotate" : 1.9583014768641922,
            "gripper_rotate" : -0.0004890796739715704
        }

        self.moveArmToJointAnglePosition( servoAnglesDict, math.radians( 1.5 ) )
    
    #---------------------------------------------------------------------------
    def onDwgCameraImageButtonPressEvent( self, widget, data ):
        
        if self.cameraImagePixBuf != None:
            
            imgRect = self.getImageRectangleInWidget( widget,
                self.cameraImagePixBuf.get_width(), self.cameraImagePixBuf.get_height() )
        
            screenX = data.x - imgRect.x 
            screenY = data.y - imgRect.y
            jointArray = self.jointScreenHomography.convertScreenPosToJointPos(
                ( screenX, screenY ) )
            jointArrayDegrees = [ math.degrees( angle ) for angle in jointArray ]
            
            print "press at", screenX, screenY, "->", jointArrayDegrees
            print self.roboticArm.GetEndEffectorPos( jointArray )
            
            servoAnglesDict = {
                "base_rotate" : jointArray[ 0 ], 
                "shoulder_rotate" : jointArray[ 1 ], 
                "elbow_rotate" : jointArray[ 2 ], 
                "wrist_rotate" : jointArray[ 3 ],
                "gripper_rotate" : jointArray[ 4 ]
            }

            self.moveArmToJointAnglePosition( servoAnglesDict, math.radians( 0.5 ) )
    
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

        UPDATE_FREQUENCY = self.SAMPLES_PER_SECOND    # Updates in Hz

        lastTime = time.clock()

        while 1:
            
            curTime = time.clock()
            
            if curTime - lastTime >= 1.0 / UPDATE_FREQUENCY:
            
                

                # Save the time
                lastTime = curTime
                
            yield True
            
        yield False
        
#-------------------------------------------------------------------------------
if __name__ == "__main__":

    mainWindow = MainWindow()
    mainWindow.main()


