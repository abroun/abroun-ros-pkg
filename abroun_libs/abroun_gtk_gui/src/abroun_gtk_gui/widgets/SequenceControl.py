#-------------------------------------------------------------------------------
import os.path
import pygtk
pygtk.require('2.0')
import gtk

#-------------------------------------------------------------------------------
class SequenceControl( gtk.VBox ):
    """
    Composite widget for moving backwards and forwards through the frames of 
    an audio or video sequence
    """

    __gtype_name__ = 'SequenceControl'

    #---------------------------------------------------------------------------
    def __init__( self ):
        gtk.VBox.__init__( self )

        self.filter = None
        self.startingFolder = None
        self.frameIdx = 0
        self.numFrames = 1
        self.onFrameIdxChangedCallback = None

        gtk.widget_push_composite_child()
        
        builder = gtk.Builder()
        scriptPath = os.path.dirname( __file__ )
        builder.add_from_file( scriptPath + "/glade/SequenceControl.glade" )
        
        window = builder.get_object( "winContainer" )
        hboxWidget = builder.get_object( "hboxWidget" )
        self.tbxFrameNumber = builder.get_object( "tbxFrameNumber" )
        self.lblNumFrames = builder.get_object( "lblNumFrames" )
        self.lblNumFrames.set_text( "/" + str( self.numFrames ) )
        
        window.remove( hboxWidget )
        self.pack_start( hboxWidget )
        builder.connect_signals( self )

        gtk.widget_pop_composite_child()
        
    #---------------------------------------------------------------------------
    def setOnFrameIdxChangedCallback( self, callback ):
        self.onFrameIdxChangedCallback = callback
        
    #---------------------------------------------------------------------------
    def setNumFrames( self, numFrames ):
        
        self.numFrames = numFrames
        if self.numFrames < 1:
            self.numFrames = 1
        
        self.lblNumFrames.set_text( "/" + str( self.numFrames ) )
        self.setFrameIdx( self.frameIdx ) # Reset current frame index to keep it valid
        
    #---------------------------------------------------------------------------
    def setFrameIdx( self, frameIdx ):

        # Clip the frame index to a valid number
        if frameIdx < 0:
            frameIdx = 0
        elif frameIdx >= self.numFrames:
            frameIdx = self.numFrames - 1

        if frameIdx != self.frameIdx:
            # Move to the new frame
            self.frameIdx = frameIdx
            
            if self.onFrameIdxChangedCallback != None:
                self.onFrameIdxChangedCallback( self )
    
    #---------------------------------------------------------------------------
    def onBtnPrevFrameClicked( self, widget, data = None ):
        self.setFrameIdx( self.frameIdx - 1 )

    #---------------------------------------------------------------------------
    def onBtnNextFrameClicked( self, widget, data = None ):
        self.setFrameIdx( self.frameIdx + 1 )

    #---------------------------------------------------------------------------
    def onTbxFrameNumberFocusOutEvent( self, widget, data = None ):
        try:
            self.setCurFrameIdx( int( self.tbxFrameNumber.get_text() ) - 1 )
        except:
            pass    # Catch errors that may occur whilst parsing an integer

    #---------------------------------------------------------------------------
    def onTbxFrameNumberKeyPressed( self, widget, keyPressEvent ):
        if gtk.gdk.keyval_name( keyPressEvent.keyval ) == "Return":
            self.onTbxFrameNumberFocusOut( widget )
