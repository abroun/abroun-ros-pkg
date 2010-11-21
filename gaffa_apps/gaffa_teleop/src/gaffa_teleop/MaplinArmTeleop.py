#! /usr/bin/python

# ROS imports
import roslib
roslib.load_manifest( 'gaffa_teleop' )
import rospy

import sys
import math
import os.path
import time

import yaml
import pygtk
pygtk.require('2.0')
import gtk
import gobject

import sensor_msgs.msg
from maplin_arm.ROS_ArmClient import ROS_ArmClient
from maplin_arm.ArmDriver import MotorStates
import cv

#-------------------------------------------------------------------------------
class MainWindow:

    BASE_MOTOR_IDX = 0
    SHOULDER_MOTOR_IDX = 1
    ELBOW_MOTOR_IDX = 2
    WRIST_MOTOR_IDX = 3

    #---------------------------------------------------------------------------
    def __init__( self ):
    
        self.scriptPath = os.path.dirname( __file__ )
        self.cameraImagePixBuf = None
        self.lastImage = None
            
        # Connect to the robot via ROS
        rospy.init_node( 'MaplinArmTeleop', anonymous=True )
        
        self.cameraImageTopic = rospy.Subscriber( "/camera/image", 
            sensor_msgs.msg.Image, self.cameraImageCallback )
        self.rosArmClient = ROS_ArmClient()
        
        self.upButtonPressed = False
        self.downButtonPressed = False
        self.leftButtonPressed = False
        self.rightButtonPressed = False
        self.shoulderUpButtonPressed = False
        self.shoulderDownButtonPressed = False
        self.elbowUpButtonPressed = False
        self.elbowDownButtonPressed = False
        
        self.guideImagePixBuf = None
            
        # Setup the GUI        
        builder = gtk.Builder()
        builder.add_from_file( self.scriptPath + "/GUI/MaplinArmTeleop.glade" )
        
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
            self.cameraImagePixBuf.add_alpha( False, 0, 0, 0 )
                
            # Resize the drawing area if necessary
            if self.dwgCameraImage.get_size_request() != ( rosImage.width, rosImage.height ):
                self.dwgCameraImage.set_size_request( rosImage.width, rosImage.height )

            # Add the guide
            if self.guideImagePixBuf != None:
                self.guideImagePixBuf.composite( self.cameraImagePixBuf, 0, 0, 
                    self.guideImagePixBuf.get_width(), self.guideImagePixBuf.get_height(), 
                    0, 0, 1.0, 1.0, gtk.gdk.INTERP_NEAREST, 255 )

            self.dwgCameraImage.queue_draw()
            self.lastImage = rosImage

        else:
            rospy.logerr( "Unhandled image encoding - " + rosImage.encoding )
    
    #---------------------------------------------------------------------------
    def onBtnUpPressed( self, widget ):
        self.upButtonPressed = True
        
    #---------------------------------------------------------------------------
    def onBtnDownPressed( self, widget ):
        self.downButtonPressed = True
    
    #---------------------------------------------------------------------------
    def onBtnLeftPressed( self, widget ):
        self.leftButtonPressed = True
        
    #---------------------------------------------------------------------------
    def onBtnRightPressed( self, widget ):
        self.rightButtonPressed = True
        
    #---------------------------------------------------------------------------
    def onBtnShoulderUpPressed( self, widget ):
        self.shoulderUpButtonPressed = True
        
    #---------------------------------------------------------------------------
    def onBtnShoulderDownPressed( self, widget ):
        self.shoulderDownButtonPressed = True
        
    #---------------------------------------------------------------------------
    def onBtnElbowUpPressed( self, widget ):
        self.elbowUpButtonPressed = True
        
    #---------------------------------------------------------------------------
    def onBtnElbowDownPressed( self, widget ):
        self.elbowDownButtonPressed = True
        
    #---------------------------------------------------------------------------
    def onBtnUpReleased( self, widget ):
        self.upButtonPressed = False
        
    #---------------------------------------------------------------------------
    def onBtnDownReleased( self, widget ):
        self.downButtonPressed = False
    
    #---------------------------------------------------------------------------
    def onBtnLeftReleased( self, widget ):
        self.leftButtonPressed = False
        
    #---------------------------------------------------------------------------
    def onBtnRightReleased( self, widget ):
        self.rightButtonPressed = False
        
    #---------------------------------------------------------------------------
    def onBtnShoulderUpReleased( self, widget ):
        self.shoulderUpButtonPressed = False
        
    #---------------------------------------------------------------------------
    def onBtnShoulderDownReleased( self, widget ):
        self.shoulderDownButtonPressed = False
        
    #---------------------------------------------------------------------------
    def onBtnElbowUpReleased( self, widget ):
        self.elbowUpButtonPressed = False
        
    #---------------------------------------------------------------------------
    def onBtnElbowDownReleased( self, widget ):
        self.elbowDownButtonPressed = False
    
    #---------------------------------------------------------------------------    
    def onBtnSaveImageClicked( self, widget ):
        
        if self.lastImage != None:
            # Convert to an OpenCV image
            cvImage = cv.CreateMatHeader( self.lastImage.height, self.lastImage.width, cv.CV_8UC3 )
            cv.SetData( cvImage, self.lastImage.data, self.lastImage.step )
            
            # Convert to BGR as OpenCV likes it
            cv.CvtColor( cvImage, cvImage, cv.CV_RGB2BGR )
            
            # Find a name for the image
            nameFormatString = "/home/abroun/abroun-ros-pkg/gaffa_apps/object_detector/test_data/saliency/maplin_{0}.png"
            imageIdx = 0
            nameFound = False
            while not nameFound:
                imageName = nameFormatString.format( imageIdx )
                if not os.path.exists( imageName ):
                    nameFound = True
                else:
                    imageIdx += 1
            
            # Save the image
            cv.SaveImage( imageName, cvImage );
    
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
    def chooseImageFile( self ):
        
        result = None
        
        dialog = gtk.FileChooserDialog(
            title="Choose Image File",
            action=gtk.FILE_CHOOSER_ACTION_SAVE,
            buttons=(gtk.STOCK_CANCEL, gtk.RESPONSE_REJECT,
                      gtk.STOCK_OK, gtk.RESPONSE_ACCEPT) )

        dialog.set_current_folder( self.scriptPath + "/../../config" )
            
        filter = gtk.FileFilter()
        filter.add_pattern( "*.png" )
        filter.add_pattern( "*.bmp" )
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
    def onMenuItemLoadGuideActivate( self, widget ):
        
        filename = self.chooseImageFile()
        
        if filename != None:
            self.guideImagePixBuf = gtk.gdk.pixbuf_new_from_file( filename )
    
    #---------------------------------------------------------------------------
    def onMenuItemQuitActivate( self, widget ):
        self.onWinMainDestroy( widget )
        
    #---------------------------------------------------------------------------
    def update( self ):

        UPDATE_RATE = 30.0  # Updates per second

        lastTime = time.time()

        while 1:
            
            curTime = time.time()
            if curTime - lastTime > 1.0/UPDATE_RATE:
                
                # Work out whether the motors should be off or going forward/backwards
                baseMotorState = MotorStates.OFF
                shoulderMotorState = MotorStates.OFF
                elbowMotorState = MotorStates.OFF
                wristMotorState = MotorStates.OFF
                
                if self.leftButtonPressed and not self.rightButtonPressed:
                    baseMotorState = MotorStates.BACKWARD
                elif self.rightButtonPressed and not self.leftButtonPressed:
                    baseMotorState = MotorStates.FORWARD
                    
                if self.shoulderUpButtonPressed and not self.shoulderDownButtonPressed:
                    shoulderMotorState = MotorStates.FORWARD
                elif self.shoulderDownButtonPressed and not self.shoulderUpButtonPressed:
                    shoulderMotorState = MotorStates.BACKWARD
                    
                if self.elbowUpButtonPressed and not self.elbowDownButtonPressed:
                    elbowMotorState = MotorStates.FORWARD
                elif self.elbowDownButtonPressed and not self.elbowUpButtonPressed:
                    elbowMotorState = MotorStates.BACKWARD
                    
                if self.upButtonPressed and not self.downButtonPressed:
                    wristMotorState = MotorStates.FORWARD
                elif self.downButtonPressed and not self.upButtonPressed:
                    wristMotorState = MotorStates.BACKWARD
                
                # Send the motor states to the arm
                self.rosArmClient.setArmMotorStates(
                    [ ( self.BASE_MOTOR_IDX, baseMotorState ),
                    ( self.SHOULDER_MOTOR_IDX, shoulderMotorState ),
                    ( self.ELBOW_MOTOR_IDX, elbowMotorState ),
                    ( self.WRIST_MOTOR_IDX, wristMotorState ) ] )                      

                # Save the update time
                lastTime = curTime
                
            yield True
            
        yield False
        
        
#-------------------------------------------------------------------------------
if __name__ == "__main__":

    mainWindow = MainWindow()
    mainWindow.main()
