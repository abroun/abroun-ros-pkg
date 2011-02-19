#-------------------------------------------------------------------------------
import os.path
import pygtk
pygtk.require('2.0')
import gtk

#-------------------------------------------------------------------------------
class MinMaxSlider( gtk.VBox ):
    """
    Composite widget that provides a slider with minimum and maximum values
    """

    __gtype_name__ = 'MinMaxSlider'

    #---------------------------------------------------------------------------
    def __init__( self ):
        gtk.VBox.__init__( self )

        self.editingMinMaxRange = False
        self.minMaxRangeLocked = False
        self.onValueChangedCallback = None

        gtk.widget_push_composite_child()
        
        builder = gtk.Builder()
        scriptPath = os.path.dirname( __file__ )
        builder.add_from_file( scriptPath + "/glade/MinMaxSlider.glade" )
        
        window = builder.get_object( "winContainer" )
        hboxWidget = builder.get_object( "hboxWidget" )
        self.tbxMin = builder.get_object( "tbxMin" )
        self.tbxMax = builder.get_object( "tbxMax" )
        self.sliderValue = builder.get_object( "sliderValue" )
        self.adjValue = builder.get_object( "adjValue" )
        self.checkTicked = builder.get_object( "checkTicked" )
        
        window.remove( hboxWidget )
        self.pack_start( hboxWidget )
        builder.connect_signals( self )

        gtk.widget_pop_composite_child()
        
        # Setup with default values
        self.setMinAndMaxValues( 0.0, 0.0 )
    
    #---------------------------------------------------------------------------
    def setOnValueChangedCallback( self, callback ):
        self.onValueChangedCallback = callback
    
    #---------------------------------------------------------------------------
    def setMinMaxRangeLocked( self, minMaxRangeLocked ):
        self.minMaxRangeLocked = minMaxRangeLocked
    
    #---------------------------------------------------------------------------
    def setMinAndMaxValues( self, minValue, maxValue ):
        
        self.editingMinMaxRange = True
        
        oldMinValue = self.adjValue.get_lower()
        oldMaxValue = self.adjValue.get_upper()
        
        # Is it possible to edit the min and max values?
        if self.minMaxRangeLocked:
            minValue = oldMinValue
            maxValue = oldMaxValue
            
        # Is it possible to parse the new min and max values?
        try:
            minValue = float( minValue )
        except:
            # Catch parse error
            minValue = oldMinValue
        
        try:
            maxValue = float( maxValue )
        except:
            # Catch parse error
            maxValue = oldMaxValue
        
        # Make sure that the minimum and maximum values are in the correct order
        if minValue > maxValue:
            tmp = maxValue
            maxValue = minValue
            minValue = tmp
        
        self.tbxMin.set_text( "{0:2.02f}".format( minValue ) )
        self.tbxMax.set_text( "{0:2.02f}".format( maxValue ) )
        self.setupSliderRange( minValue, maxValue )
        
        self.editingMinMaxRange = False
    
    #---------------------------------------------------------------------------
    def setupSliderRange( self, minValue, maxValue ):
        
        STEP_INCREMENT = 1.0
        PAGE_INCREMENT = 0.0
        PAGE_SIZE = 0.0
        
        # Work out what value the slider should have
        newSliderValue = self.adjValue.get_value()
        
        if newSliderValue < minValue:
            newSliderValue = minValue
        if newSliderValue > maxValue:
            newSliderValue = maxValue
        
        # Update the adjustment
        self.adjValue.set_all( newSliderValue, minValue, maxValue, 
            STEP_INCREMENT, PAGE_INCREMENT, PAGE_SIZE )
       
    #---------------------------------------------------------------------------
    def getValue( self ):
        return self.adjValue.get_value()
        
    #---------------------------------------------------------------------------
    def setValue( self, newValue ):
        return self.adjValue.set_value( newValue )
        
    #---------------------------------------------------------------------------
    def isTicked( self ):
        return self.checkTicked.get_active()
       
    #---------------------------------------------------------------------------
    def onTbxMinChanged( self, widget ):
        
        if self.editingMinMaxRange:
            return
        
        self.setMinAndMaxValues( self.tbxMin.get_text(), self.tbxMax.get_text() )
        
    #---------------------------------------------------------------------------
    def onTbxMaxChanged( self, widget ):
        
        if self.editingMinMaxRange:
            return
        
        self.setMinAndMaxValues( self.tbxMin.get_text(), self.tbxMax.get_text() )
        
    #---------------------------------------------------------------------------
    def onAdjValueValueChanged( self, widget ):
        
        if self.onValueChangedCallback != None:
            self.onValueChangedCallback( self )
        
    