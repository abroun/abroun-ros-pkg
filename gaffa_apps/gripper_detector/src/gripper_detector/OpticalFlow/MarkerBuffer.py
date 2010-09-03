
import numpy as np
import yaml

#-------------------------------------------------------------------------------
def saveMarkerBuffer( markerBuffer, filename ):

    outputFile = file( filename, "w" )
    yaml.dump( markerBuffer.tolist(), outputFile )
    outputFile.close()

#-------------------------------------------------------------------------------
def loadMarkerBuffer( filename ):

    result = None

    markerFile = file( filename, "r" )
    markerBuffer = yaml.load( markerFile )
    
    if type( markerBuffer ) == list:
        markerBuffer = np.array( markerBuffer )
        if len( markerBuffer.shape ) == 2:
            
            result = markerBuffer
    
        else:
            print "Error: The data is not a 2D list"
    else:
        print "Error: The data is not a list"

    return result