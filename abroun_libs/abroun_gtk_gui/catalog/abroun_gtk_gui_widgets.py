
import os.path
import sys

sys.path.append( os.path.dirname( __file__ ) + "/../src" )
from abroun_gtk_gui.widgets import FilePath

#import gtk
#from gtk import gdk

##-------------------------------------------------------------------------------
#class FilePath( gtk.Button ):
    #"""
    #A ProxyButton is a Button subclass which is implementing the features
    #required to be used inside the kiwi framework.

    #It has a specific feature not found in other implementations. If
    #the datatype is set to pixbuf a gtk.Image will be constructed from the
    #pixbuf and be set as a child for the Button
    #"""

    #__gtype_name__ = 'FilePath'

    #def __init__( self ):
        #gtk.Button.__init__( self )

    #def read(self):
        #if self.data_type == 'Pixbuf':
            #image = self.get_image()
            #if not image:
                #return

            #storage_type = image.get_storage_type()
            #if storage_type != gtk.IMAGE_PIXBUF:
                #raise ValueError(
                    #"the image of a ProxyButton must be loaded "
                    #"from a pixbuf, not %s" % storage_type)
            #return image.get_pixbuf()
        #else:
            #return self._from_string(self.get_label())

    #def update(self, data):
        #if self.data_type == 'Pixbuf':
            #if data == ValueUnset:
                #data = None

            #if not data:
                #image = None
            #else:
                #image = gtk.Image()
                #image.set_from_pixbuf(data)
                #image.show()

            #self.set_property('image', image)
        #else:
            #if data is None:
                #text = ""
            #else:
                #text = self._as_string(data)
            #self.set_label(text)

        #self.emit('content-changed')

