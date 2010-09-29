
import cv
import numpy as np
import math
import scipy.ndimage

#-------------------------------------------------------------------------------
class ResidualSaliencyFilter:
    """Calculates a saliency map for an image based on the method of Hou and Zhang"""
    
    SALIENCY_MAP_HEIGHT = 64

    #---------------------------------------------------------------------------
    def __init__( self ):
        pass
  
    #---------------------------------------------------------------------------
    def calcSaliencyMap( self, imageGray ):
        
        imageWidth = imageGray.shape[ 1 ]
        imageHeight = imageGray.shape[ 0 ]
        
        # Scale the image, this determines the size of feature which is looked for
        scale = float( self.SALIENCY_MAP_HEIGHT ) / float( imageHeight )
        
        scaledImageGray = scipy.ndimage.interpolation.zoom( 
            imageGray, scale, output=np.uint8 )
        
        # Calculate the saliency map according to Hou and Zhang
        saliencyMap = self.numpySaliency( scaledImageGray )
                
        largeSaliencyMap = np.ndarray( ( imageHeight, imageWidth ), dtype=np.uint8 )
        
        scipy.ndimage.interpolation.zoom( 
            saliencyMap,
            ( float(imageHeight)/saliencyMap.shape[ 0 ], float(imageWidth)/saliencyMap.shape[ 1 ] ),
            output=largeSaliencyMap )
        
        # Return the saliency map at the size it was calculated and also at the
        # original image size
        return ( saliencyMap, largeSaliencyMap )
    
    #---------------------------------------------------------------------------
    def numpySaliency( self, scaledImageGray ):
        
        # Convert the image to the complex frequency domain
        imageSpectrum = np.fft.fft2( scaledImageGray )
        
        phaseAngle = np.angle( imageSpectrum )
        
        imageSpectrum.real = np.cos( phaseAngle )
        imageSpectrum.imag = np.sin( phaseAngle )
        
        # Reconstruct the image using just the phase angle
        newImage = np.fft.ifft2( imageSpectrum ).real
        
        # Square the image as we are interpreting it as an error...
        newImage = np.square( newImage )
        
        # Smmoth and normalise the image
        newImage = scipy.ndimage.filters.gaussian_filter( 
            newImage, 4.0, mode='constant' )
        minVal = np.amin( newImage )
        maxVal = np.amax( newImage )
        newImage = np.multiply( np.subtract( newImage, minVal ), 255.0/maxVal )
        
        saliencyMap = np.array( newImage, dtype=np.uint8 )
        
        return saliencyMap
