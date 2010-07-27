#! /usr/bin/python

# ROS imports
import roslib
roslib.load_manifest( 'gaffa_teleop' )
import rospy

import sys
import math
import os.path
import time

import numpy as np
import Homog3D

import yaml
import pygtk
pygtk.require('2.0')
import gtk
import gobject

import LynxmotionArmDescription
from lynxmotion_arm.SSC32Driver import SSC32Driver, ServoConfig
from RoboticArm import DHFrame, RoboticArm

#-------------------------------------------------------------------------------
class ConfigData( yaml.YAMLObject ):

    yaml_tag = u'!ConfigData'

    def __init__( self, servoConfigList, servoAngles ):
        self.servoConfigList = servoConfigList
        self.servoAngles = servoAngles

#-------------------------------------------------------------------------------
class MainWindow:
 
    NUM_SERVOS = 5
    MIN_SERVO_VALUE = 500
    MAX_SERVO_VALUE = 2500
    
    ARM_DRAWING_SCALE = 4.0
 
    CONFIG_FILENAME = "KinematicConfig.yaml"

    #---------------------------------------------------------------------------
    def __init__( self ):
    
        self.servoAngles = [0.0]*self.NUM_SERVOS

        scriptPath = os.path.dirname( __file__ )
        self.fullConfigFilename = scriptPath + "/" + self.CONFIG_FILENAME
        
        if os.path.exists( self.fullConfigFilename ):
            configFile = file( self.fullConfigFilename, "r" )
            configData = yaml.load( configFile )
            servoConfigList = configData.servoConfigList
            self.savedServoAngles = configData.servoAngles
            
            configFile.close()
        else:
            servoConfigList = []
            self.savedServoAngles = None
            for servoIdx in range( self.NUM_SERVOS ):
                servoConfigList.append( ServoConfig( 0, 0, 0, 0, servoIdx ) )
        
        for servoIdx in range( self.NUM_SERVOS ):
            servoConfigList[ servoIdx ].validateServoConfig()
            
        self.roboticArm = RoboticArm( LynxmotionArmDescription.ARM_DH_PROXIMAL, servoConfigList )
        self.jointPositions = self.roboticArm.GetJointPositions( self.servoAngles )

        # Setup the GUI
        self.draggingInTopView = False
        self.draggingInSideView = False
        
        builder = gtk.Builder()
        builder.add_from_file( scriptPath + "/GUI/GaffaTeleopGUI.glade" )
        
        self.window = builder.get_object( "winMain" )
        
        self.minServoTextBoxes = []
        self.maxServoTextBoxes = []
        self.minDegreesTextBoxes = []
        self.maxDegreesTextBoxes = []
        self.servoScales = []
        self.servoDisplays = []
        self.degreesDisplays = []
        for servoIdx in range( self.NUM_SERVOS ):
            self.minServoTextBoxes.append( builder.get_object( "tbxMinServo_" + str( servoIdx ) ) )
            self.maxServoTextBoxes.append( builder.get_object( "tbxMaxServo_" + str( servoIdx ) ) )
            self.minDegreesTextBoxes.append( builder.get_object( "tbxMinDegrees_" + str( servoIdx ) ) )
            self.maxDegreesTextBoxes.append( builder.get_object( "tbxMaxDegrees_" + str( servoIdx ) ) )
            self.servoScales.append( builder.get_object( "scaleServo_" + str( servoIdx ) ) )
            self.servoDisplays.append( builder.get_object( "lblServo_" + str( servoIdx ) ) )
            self.degreesDisplays.append( builder.get_object( "lblDegrees_" + str( servoIdx ) ) )

        self.chkSendAngles = builder.get_object( "chkSendAngles" )
        self.lblX = builder.get_object( "lblX" )
        self.lblY = builder.get_object( "lblY" )
        self.lblZ = builder.get_object( "lblZ" )
        self.dwgSideView = builder.get_object( "dwgSideView" )
        self.dwgTopView = builder.get_object( "dwgTopView" )
        
        builder.connect_signals( self )
        
        for servoIdx in range( self.NUM_SERVOS ):
            self.updateServoControlFromServoConfig( servoIdx )
        
        # Get the currently set joint angles from the arm
        #pulseWidths = self.armController.queryJointPulseWidths()
        #for servoIdx in range( self.NUM_SERVOS ):
        #    servoConfig = self.servoConfigList[ servoIdx ]
        #    normalisedServoValue = (pulseWidths[ servoIdx ] - servoConfig.startPulseWidth) \
        #        / servoConfig.getServoRange()
        #    self.servoScales[ servoIdx ].set_value( normalisedServoValue*abs( servoConfig.getServoRange() ) )
    
        if self.savedServoAngles != None:
            self.setServoAngles( self.savedServoAngles )
            
        updateLoop = self.update()
        gobject.idle_add( updateLoop.next )
        
        self.window.show()
        
    #---------------------------------------------------------------------------
    def setServoAngles( self, servoAngles ):
        for servoIdx in range( self.NUM_SERVOS ):
            servoConfig = self.roboticArm.GetServoConfig( servoIdx )
            
            normalisedServoValue = (servoAngles[ servoIdx ] - servoConfig.minAngle) \
                / (servoConfig.maxAngle - servoConfig.minAngle)
            if normalisedServoValue >= 0.0 and normalisedServoValue <= 1.0:
                self.servoScales[ servoIdx ].set_value( normalisedServoValue*abs( servoConfig.getServoRange() ) )


    #---------------------------------------------------------------------------
    def onWinMainDestroy( self, widget, data = None ):  
        gtk.main_quit()
        
    #---------------------------------------------------------------------------   
    def main( self ):
        # All PyGTK applications must have a gtk.main(). Control ends here
        # and waits for an event to occur (like a key press or mouse event).
        gtk.main()

    #---------------------------------------------------------------------------
    def update( self ):

        lastTime = time.clock()

        while 1:
            
            curTime = time.clock()
            if curTime - lastTime > 0.05 and self.chkSendAngles.get_active():
                servoAnglesDict = {
                    "base_rotate" : self.servoAngles[ 0 ],
                    "shoulder_rotate" : self.servoAngles[ 1 ],
                    "elbow_rotate" : self.servoAngles[ 2 ],
                    "wrist_rotate" : self.servoAngles[ 3 ],
                    "gripper_rotate" : self.servoAngles[ 4 ],
                }
                self.roboticArm.setJointAngles( servoAnglesDict, math.radians( 2.5 ) )

                lastTime = curTime
                
            yield True
            
        yield False
        
    #---------------------------------------------------------------------------
    def parseFloat( self, string ):
        
        result = 0.0
        try:
            result = float( string )
        except:
            pass    # Catch errors that may occur whilst parsing a number
        
        return result
    
    #---------------------------------------------------------------------------
    def updateServoControlFromServoConfig( self, servoIdx ):

        servoConfig = self.roboticArm.GetServoConfig( servoIdx )

        # Update control values
        self.minServoTextBoxes[ servoIdx ].set_text( str( servoConfig.startPulseWidth ) )
        self.maxServoTextBoxes[ servoIdx ].set_text( str( servoConfig.endPulseWidth ) )

        minDegrees = math.degrees( servoConfig.minAngle )
        maxDegrees = math.degrees( servoConfig.maxAngle )
        self.minDegreesTextBoxes[ servoIdx ].set_text( str( minDegrees ) )
        self.maxDegreesTextBoxes[ servoIdx ].set_text( str( maxDegrees ) )
        
        absServoRange = abs( servoConfig.getServoRange() )
        self.servoScales[ servoIdx ].set_range( 0, absServoRange )
        self.servoScales[ servoIdx ].set_increments( 1.0, 10.0 )
        
        self.updateServoValue( servoIdx )
    
    #---------------------------------------------------------------------------
    def updateServoRange( self, servoIdx ):
        
        servoConfig = self.roboticArm.GetServoConfig( servoIdx )
        
        # Read values from controls
        servoConfig.startPulseWidth = \
            int( self.parseFloat( self.minServoTextBoxes[ servoIdx ].get_text() ) )
        servoConfig.endPulseWidth = \
            int( self.parseFloat( self.maxServoTextBoxes[ servoIdx ].get_text() ) )
        servoConfig.minAngle = math.radians(
            self.parseFloat( self.minDegreesTextBoxes[ servoIdx ].get_text() ) )
        servoConfig.maxAngle = math.radians(
            self.parseFloat( self.maxDegreesTextBoxes[ servoIdx ].get_text() ) )
        
        # Validate Servo config
        servoConfig.validateServoConfig()

        # Update controls
        self.updateServoControlFromServoConfig( servoIdx )
        
    #---------------------------------------------------------------------------
    def updateServoValue( self, servoIdx ):
        
        minDegrees = self.parseFloat( self.minDegreesTextBoxes[ servoIdx ].get_text() )
        maxDegrees = self.parseFloat( self.maxDegreesTextBoxes[ servoIdx ].get_text() )
        degreesRange = maxDegrees - minDegrees
        
        # Get the requested servo value and convert it to degrees
        servoConfig = self.roboticArm.GetServoConfig( servoIdx )
        normalisedServoValue = abs( self.servoScales[ servoIdx ].get_value() 
            / float( servoConfig.getServoRange() ) )

        servoValue = int( servoConfig.startPulseWidth + normalisedServoValue*servoConfig.getServoRange() )
        degreesValue = minDegrees + normalisedServoValue*degreesRange
        
        self.servoDisplays[ servoIdx ].set_text( str( servoValue ) )
        self.degreesDisplays[ servoIdx ].set_text( "{0:.2f}".format( degreesValue ) )

        self.servoAngles[ servoIdx ] = math.radians( degreesValue )
        
        self.updatePositionDisplay()    # This may inefficiently be called many times at the start
        
    #---------------------------------------------------------------------------
    def updatePositionDisplay( self ):
        self.jointPositions = self.roboticArm.GetJointPositions( self.servoAngles )
        
        endEffectorPos = self.jointPositions[ -1 ]
        self.lblX.set_text( "{0:.2f}".format( endEffectorPos[ 0, 0 ] ) )
        self.lblY.set_text( "{0:.2f}".format( endEffectorPos[ 1, 0 ] ) )
        self.lblZ.set_text( "{0:.2f}".format( endEffectorPos[ 2, 0 ] ) )
        
        self.dwgSideView.queue_draw()
        self.dwgTopView.queue_draw()
        
    #---------------------------------------------------------------------------
    def tryToSetEndEffectorPos( self, newPos ):
        newPosFeasible = False
    
        newServoAngles = self.roboticArm.CalculateJointAnglesIK( newPos, self.servoAngles )
        if newServoAngles != None:
            # It is possible to reach the requested position
            newPosFeasible = True
            
            self.servoAngles = newServoAngles[:]
            self.setServoAngles( self.servoAngles )
            self.updatePositionDisplay()
            
        return newPosFeasible
       
    #---------------------------------------------------------------------------
    def tryToSetEndEffectorPosFromTopView( self, x, y ):
        
        curEndEffectorPos = self.jointPositions[ -1 ]
        targetPos = np.matrix( curEndEffectorPos )
        
        # Translate from the render position to world coordinates
        widgetX, widgetY, widgetWidth, widgetHeight = self.dwgTopView.get_allocation()
        originPos = Homog3D.CreateVector( 0.0, 0.0, 0.0 )
        originRenderPos = ( widgetWidth/2, widgetHeight-1 )
        targetRenderPos = ( x, y )
        targetPos[ 0, 0 ] = -( targetRenderPos[ 1 ] - originRenderPos[ 1 ] ) / self.ARM_DRAWING_SCALE
        targetPos[ 1, 0 ] = -( targetRenderPos[ 0 ] - originRenderPos[ 0 ] ) / self.ARM_DRAWING_SCALE
        
        targetPos = self.tryToEnsureThatTargetIsReachable( targetPos )
        return self.tryToSetEndEffectorPos( targetPos )
        
    #---------------------------------------------------------------------------
    def tryToSetEndEffectorPosFromSideView( self, x, y ):
    
        curEndEffectorPos = self.jointPositions[ -1 ]
        targetPos = np.matrix( curEndEffectorPos )
        
        # Translate from the render position to world coordinates
        widgetX, widgetY, widgetWidth, widgetHeight = self.dwgSideView.get_allocation()
        originPos = Homog3D.CreateVector( 0.0, 0.0, 0.0 )
        originRenderPos = ( widgetWidth/2, widgetHeight-1 )
        targetRenderPos = ( x, y )
        targetPos[ 0, 0 ] = ( targetRenderPos[ 0 ] - originRenderPos[ 0 ] ) / self.ARM_DRAWING_SCALE
        targetPos[ 2, 0 ] = -( targetRenderPos[ 1 ] - originRenderPos[ 1 ] ) / self.ARM_DRAWING_SCALE
        
        targetPos = self.tryToEnsureThatTargetIsReachable( targetPos )
        return self.tryToSetEndEffectorPos( targetPos )
        
    #---------------------------------------------------------------------------
    def tryToEnsureThatTargetIsReachable( self, targetPos ):
        
        shoulderPos = Homog3D.CreateVector( 0.0, 0.0, 
            LynxmotionArmDescription.GROUND_TO_CENTRE_OF_SHOULDER_HEIGHT )
        shoulderToTargetVector = targetPos - shoulderPos
        distanceFromShoulderToTarget = math.sqrt( shoulderToTargetVector.T * shoulderToTargetVector )
        
        reachableLength = LynxmotionArmDescription.TOP_ARM_LENGTH \
            + LynxmotionArmDescription.FORE_ARM_LENGTH + LynxmotionArmDescription.HAND_GRIP_LENGTH  
            
        if distanceFromShoulderToTarget > reachableLength:
            shoulderToTargetVector /= distanceFromShoulderToTarget
            targetPos = shoulderPos + (reachableLength-0.01)*shoulderToTargetVector
            
        return targetPos
        
    #---------------------------------------------------------------------------
    def onScaleServoValueChanged( self, widget, data = None ):
        servoIdx = int( gtk.Buildable.get_name( widget ).split( '_' )[ 1 ] )
        self.updateServoValue( servoIdx )

    #---------------------------------------------------------------------------
    def onTbxScaleFocusOutEvent( self, widget, data = None ):
        servoIdx = int( gtk.Buildable.get_name( widget ).split( '_' )[ 1 ] )
        self.updateServoRange( servoIdx )

    #---------------------------------------------------------------------------
    def onTbxScaleKeyPressedEvent( self, widget, keyPressEvent ):
        if gtk.gdk.keyval_name( keyPressEvent.keyval ) == "Return":
            self.onTbxScaleFocusOutEvent( widget )
       
    #---------------------------------------------------------------------------
    def onDwgTopViewExposeEvent( self, widget, data = None ):
    
        # Draw arm lines using x, y coordinates    
        widgetX, widgetY, widgetWidth, widgetHeight = widget.get_allocation()
        originPos = Homog3D.CreateVector( 0.0, 0.0, 0.0 )
        originRenderPos = ( widgetWidth/2, widgetHeight-1 )
        
        lastJointPos = originPos
        lastJointRenderPos = originRenderPos
        renderPosList = [ originRenderPos ]
        
        for jointIdx in range( self.roboticArm.numJoints ):
            # Calculate the relative difference between this joint and the last
            jointPos = self.jointPositions[ jointIdx ]
            posDiff = self.ARM_DRAWING_SCALE*( jointPos - lastJointPos )
            
            jointRenderPos = ( int( lastJointRenderPos[ 0 ] - posDiff[ 1, 0 ] ), 
                int( lastJointRenderPos[ 1 ] - posDiff[ 0, 0 ] ) )
            
            # Store the render pos and move onto the next joint
            renderPosList.append( jointRenderPos )
            lastJointPos = jointPos
            lastJointRenderPos = jointRenderPos
            
        # Draw the arm
        graphicsContext = widget.window.new_gc()
        graphicsContext.set_rgb_fg_color( gtk.gdk.Color( 65535, 65535, 65535 ) )

        widget.window.draw_rectangle( graphicsContext, filled=True,
            x=0, y=0, width=widgetWidth, height=widgetHeight ) 
        
        graphicsContext.set_rgb_fg_color( gtk.gdk.Color( 0, 0, 0 ) )
        widget.window.draw_lines( graphicsContext, renderPosList )

    #---------------------------------------------------------------------------
    def onDwgSideViewExposeEvent( self, widget, data = None ):
    
        # Draw arm lines using x, z coordinates     
        widgetX, widgetY, widgetWidth, widgetHeight = widget.get_allocation()
        originPos = Homog3D.CreateVector( 0.0, 0.0, 0.0 )
        originRenderPos = ( widgetWidth/2, widgetHeight-1 )
        
        lastJointPos = originPos
        lastJointRenderPos = originRenderPos
        renderPosList = [ originRenderPos ]
        
        for jointIdx in range( self.roboticArm.numJoints ):
            # Calculate the relative difference between this joint and the last
            jointPos = self.jointPositions[ jointIdx ]
            posDiff = self.ARM_DRAWING_SCALE*( jointPos - lastJointPos )
            
            jointRenderPos = ( int( lastJointRenderPos[ 0 ] + posDiff[ 0, 0 ] ), 
                int( lastJointRenderPos[ 1 ] - posDiff[ 2, 0 ] ) )
            
            # Store the render pos and move onto the next joint
            renderPosList.append( jointRenderPos )
            lastJointPos = jointPos
            lastJointRenderPos = jointRenderPos
            
        # Draw the arm
        graphicsContext = widget.window.new_gc()
        graphicsContext.set_rgb_fg_color( gtk.gdk.Color( 65535, 65535, 65535 ) )

        widget.window.draw_rectangle( graphicsContext, filled=True,
            x=0, y=0, width=widgetWidth, height=widgetHeight ) 
        
        graphicsContext.set_rgb_fg_color( gtk.gdk.Color( 0, 0, 0 ) )
        widget.window.draw_lines( graphicsContext, renderPosList )
    
    #---------------------------------------------------------------------------
    def onDwgTopViewButtonPressEvent( self, widget, data = None ):
    
        if data.button == 1:
            self.draggingInTopView = True
            self.tryToSetEndEffectorPosFromTopView( data.x, data.y )
    
    #---------------------------------------------------------------------------
    def onDwgTopViewButtonReleaseEvent( self, widget, data = None ):
        
        if data.button == 1 and self.draggingInTopView:
            self.draggingInTopView = False
    
    #---------------------------------------------------------------------------
    def onDwgTopViewMotionNotifyEvent( self, widget, data = None ):
    
        if self.draggingInTopView:
            self.tryToSetEndEffectorPosFromTopView( data.x, data.y )
    
    #---------------------------------------------------------------------------
    def onDwgSideViewButtonPressEvent( self, widget, data = None ):
        
        if data.button == 1:
            self.draggingInSideView = True
            self.tryToSetEndEffectorPosFromSideView( data.x, data.y )
    
    #---------------------------------------------------------------------------
    def onDwgSideViewButtonReleaseEvent( self, widget, data = None ):
        
        if data.button == 1 and self.draggingInSideView:
            self.draggingInSideView = False
    
    #---------------------------------------------------------------------------
    def onDwgSideViewMotionNotifyEvent( self, widget, data = None ):
        
        if self.draggingInSideView:
            self.tryToSetEndEffectorPosFromSideView( data.x, data.y )

    #---------------------------------------------------------------------------
    def onBtnSaveConfigClicked( self, widget, data = None ):
        saveFile = file( self.fullConfigFilename, "w" )
        servoConfigList = [self.roboticArm.GetServoConfig( jointIdx ) for jointIdx in range( self.roboticArm.numJoints )]
        configData = ConfigData( servoConfigList, self.servoAngles )
        yaml.dump( configData, saveFile )
        saveFile.close()
        
#-------------------------------------------------------------------------------
if __name__ == "__main__":

    mainWindow = MainWindow()
    mainWindow.main()