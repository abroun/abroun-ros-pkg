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

import yaml
import pygtk
pygtk.require('2.0')
import gtk
import gobject

import gaffa_teleop.LynxmotionArmDescription
from lynxmotion_arm.SSC32Driver import ServoConfig
from gaffa_teleop.RoboticArm import DHFrame, RoboticArm
import sensor_msgs.msg

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
 
    #---------------------------------------------------------------------------
    def __init__( self ):
    
        scriptPath = os.path.dirname( __file__ )
        self.cameraImagePixBuf = None
        self.servoConfigDict = getArmServoConfig()
            
        # Connect to the robot via ROS
        #rospy.init_node( 'GripperDetector', anonymous=True )
        
        # TODO: Move arm to all dictionary
        servoConfigList = [ self.servoConfigDict[ servoName ] for servoName in self.servoConfigDict ]
        self.roboticArm = RoboticArm( gaffa_teleop.LynxmotionArmDescription.ARM_DH_PROXIMAL, servoConfigList )
        
        self.cameraImageTopic = rospy.Subscriber( "/camera/image", 
            sensor_msgs.msg.Image, self.cameraImageCallback )
        
        # Setup the GUI
        self.draggingInTopView = False
        self.draggingInSideView = False
        
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
    def cameraImageCallback( self, image ):
        
        if image.encoding == "rgb8":
            
            # Display the image
            self.cameraImagePixBuf = gtk.gdk.pixbuf_new_from_data( 
                image.data, 
                gtk.gdk.COLORSPACE_RGB,
                False,
                8,
                image.width,
                image.height,
                image.step )

            # Resize the drawing area if necessary
            if self.dwgCameraImage.get_size_request() != ( image.width, image.height ):
                self.dwgCameraImage.set_size_request( image.width, image.height )

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
        
        self.roboticArm.setJointAngles( servoAnglesDict, math.radians( 2.5 ) )
        
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

        lastTime = time.clock()

        while 1:
            
            curTime = time.clock()
            #if curTime - lastTime > 0.05 and self.chkSendAngles.get_active():
                #servoAnglesDict = {
                    #"base_rotate" : self.servoAngles[ 0 ],
                    #"shoulder_rotate" : self.servoAngles[ 1 ],
                    #"elbow_rotate" : self.servoAngles[ 2 ],
                    #"wrist_rotate" : self.servoAngles[ 3 ],
                    #"gripper_rotate" : self.servoAngles[ 4 ],
                #}
                #self.roboticArm.setJointAngles( servoAnglesDict, math.radians( 2.5 ) )

                #lastTime = curTime
                
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