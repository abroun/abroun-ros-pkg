#-------------------------------------------------------------------------------
import os.path
import pygtk
pygtk.require('2.0')
import gtk

#-------------------------------------------------------------------------------
class FilePath( gtk.VBox ):
    """
    Composite widget consisting of a textbox and a button which allows a user to
    either enter a filename or else click the button to open a file selector
    dialog
    """

    __gtype_name__ = 'FilePath'

    #---------------------------------------------------------------------------
    def __init__( self ):
        gtk.VBox.__init__( self )

        self.filter = None
        self.startingFolder = None
        self.onFilenameChangedCallback = None

        gtk.widget_push_composite_child()
        
        builder = gtk.Builder()
        scriptPath = os.path.dirname( __file__ )
        builder.add_from_file( scriptPath + "/glade/FilePath.glade" )
        
        window = builder.get_object( "window1" )
        topBox = builder.get_object( "topBox" )
        self.tbxFile = builder.get_object( "tbxFile" )
        
        window.remove( topBox )
        self.pack_start( topBox )
        builder.connect_signals( self )

        gtk.widget_pop_composite_child()
        
    #---------------------------------------------------------------------------
    def setFilter( self, filter ):
    
        self.filter = filter
    
    #---------------------------------------------------------------------------
    def setStartingFolder( self, startingFolder ):
        
        self.startingFolder = startingFolder
        
    #---------------------------------------------------------------------------
    def getFilename( self ):
        return self.tbxFile.get_text()
        
    #---------------------------------------------------------------------------
    def setFilename( self, filename ):
        return self.tbxFile.set_text( filename )
        
    #---------------------------------------------------------------------------
    def setOnFilenameChangedCallback( self, callback ):
        self.onFilenameChangedCallback = callback
    
    #---------------------------------------------------------------------------
    def onTbxFileChanged( self, widget, data = None ):
        if self.onFilenameChangedCallback != None:
            self.onFilenameChangedCallback( widget, data )
        
    #---------------------------------------------------------------------------
    def onBtnFileClicked( self, widget, data = None ):
    
        dialog = gtk.FileChooserDialog(
            title="Choose Output File",
            action=gtk.FILE_CHOOSER_ACTION_SAVE,
            buttons=(gtk.STOCK_CANCEL, gtk.RESPONSE_REJECT,
                      gtk.STOCK_OK, gtk.RESPONSE_ACCEPT) )

        if self.startingFolder != None:
            dialog.set_current_folder( self.startingFolder )
            
        if self.filter != None:
            dialog.add_filter( self.filter )
            dialog.set_filter( self.filter )
            
        result = dialog.run()

        if result == gtk.RESPONSE_ACCEPT:
            filename = dialog.get_filename()
            #if os.path.splitext( filename )[ 1 ] == "":
            #    filename += ".yaml"

            self.tbxFile.set_text( filename )

        dialog.destroy()