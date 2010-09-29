
import cv
import numpy as np
import math
import scipy.ndimage

#-------------------------------------------------------------------------------
class ImageFlowFilter:
    """Tries to estimate the difference between two successive images as a 
       combination of a rotation and translation by using a Sum of Square
       Differences (SSD) measure"""
    
    TRANSLATION_STEP = 0.25
    MAX_NUM_TRANSLATION_STEPS = 0.25
    ROTATION_STEP_ANGLE = math.radians( 0.5 ) 
    MAX_NUM_ROTATION_STEPS = 10

    #---------------------------------------------------------------------------
    def __init__( self ):
        
        self.lastImageGray = None

    #---------------------------------------------------------------------------
    def getLastImageGray( self ):
        return self.lastImageGray
  
    #---------------------------------------------------------------------------
    def calcImageFlow( self, curImageGray ):
        
        curImageGray = scipy.ndimage.interpolation.zoom( 
            np.array( curImageGray, dtype=np.int32 ), 0.25 )
        
        if self.lastImageGray == None:
            lastImageGray = curImageGray
        else:
            lastImageGray = self.lastImageGray
            
        transX = 0
        transY = 0
        transDistance = 0
        rotationAngle = 0
        bestTransformedImage = None
        
        imageWidth = lastImageGray.shape[ 1 ]
        imageHeight = lastImageGray.shape[ 0 ]
        transformedImage = np.ndarray( ( imageHeight, imageWidth ), dtype=np.int32 )
        
        # Try all possible translations
        minSSD = None
        translationOffsetArray = np.arange( 
            -self.MAX_NUM_TRANSLATION_STEPS*self.TRANSLATION_STEP, 
            (self.MAX_NUM_TRANSLATION_STEPS + 1)*self.TRANSLATION_STEP,
            self.TRANSLATION_STEP )
        for xOffset in translationOffsetArray:
            for yOffset in translationOffsetArray:
                
                scipy.ndimage.interpolation.shift( 
                    lastImageGray, ( yOffset, xOffset ), output=transformedImage )
                
                #transformedImage.fill( 0 )
                
                #srcX = max( -xOffset, 0 )
                #srcWidth = min( imageWidth - xOffset, imageWidth )
                #dstX = max( xOffset, 0 )
                #dstWidth = min( imageWidth + xOffset, imageWidth )
                
                #srcY = max( -yOffset, 0 )
                #srcHeight = min( imageHeight - yOffset, imageHeight )
                #dstY = max( yOffset, 0 )
                #dstHeight = min( imageHeight + yOffset, imageHeight )
                
                #transformedImage[ dstY:dstHeight, dstX:dstWidth ] = \
                    #lastImageGray[ srcY:srcHeight, srcX:srcWidth ]
                    
                transformSSD = np.sum( np.square( transformedImage - curImageGray ) )
                newTransDistance = math.sqrt( xOffset**2 + yOffset**2 )
                
                if minSSD == None \
                    or transformSSD < minSSD \
                    or ( transformSSD == minSSD and newTransDistance < transDistance ):
                    
                    transX = xOffset
                    transY = yOffset
                    transDistance = newTransDistance
                    bestTransformedImage = np.copy( transformedImage )
                    minSSD = transformSSD
            
        # Save the current image
        self.lastImageGray = curImageGray
        
        newImage = np.array( 
            scipy.ndimage.interpolation.zoom( bestTransformedImage, 4.0 ), dtype=np.uint8 )
        newImage[ newImage > 0 ] = 255
        #cv.Dilate( newImage, newImage )
        return ( transX, transY, rotationAngle, newImage )
