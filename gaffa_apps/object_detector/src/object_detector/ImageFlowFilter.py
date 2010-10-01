
import cv
import numpy as np
import math
import scipy.ndimage

#-------------------------------------------------------------------------------
class ImageFlowFilter:
    """Tries to estimate the difference between two successive images as a 
       combination of a rotation and translation by using a Sum of Square
       Differences (SSD) measure"""
    
    TRANSLATION_STEP = 1
    MAX_NUM_TRANSLATION_STEPS = 3
    ROTATION_STEP_ANGLE = math.radians( 0.5 ) 
    MAX_NUM_ROTATION_STEPS = 10

    #---------------------------------------------------------------------------
    def __init__( self ):
        
        self.lastImageGray = None

    #---------------------------------------------------------------------------
    def getLastImageGray( self ):
        return self.lastImageGray
  
    #---------------------------------------------------------------------------
    def calcImageFlow( self, targetImageGray, templateImageGray=None ):
        """Tries to move a template image to match a target image"""
        
        if templateImageGray == None:
            
            # Use the last image as the template if we have it
            if self.lastImageGray == None:
                templateImageGray = targetImageGray
            else:
                templateImageGray = self.lastImageGray
        
        # Save the current image
        self.lastImageGray = targetImageGray
        
        return self.calcImageFlowInternal( targetImageGray, templateImageGray )
            
    #---------------------------------------------------------------------------
    def calcImageFlowInternal( self, targetImageGray, templateImageGray ):
            
        transX = 0
        transY = 0
        transDistance = 0
        rotationAngle = 0
        bestTransformedImage = None
        
        smallTargetImageGray = scipy.ndimage.interpolation.zoom( 
            targetImageGray, 0.25 ).astype( np.int32 )
            
        smallTemplateImageGray = scipy.ndimage.interpolation.zoom( 
                templateImageGray, 0.25 ).astype( np.int32 )
        
        imageWidth = smallTemplateImageGray.shape[ 1 ]
        imageHeight = smallTemplateImageGray.shape[ 0 ]
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
                    smallTemplateImageGray, ( yOffset, xOffset ), output=transformedImage )
                
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
                    
                transformSSD = np.sum( np.square( transformedImage - smallTargetImageGray ) )
                newTransDistance = math.sqrt( xOffset**2 + yOffset**2 )
                
                if minSSD == None \
                    or transformSSD < minSSD \
                    or ( transformSSD == minSSD and newTransDistance < transDistance ):
                    
                    transX = xOffset
                    transY = yOffset
                    transDistance = newTransDistance
                    bestTransformedImage = np.copy( transformedImage )
                    minSSD = transformSSD
        
        newImage = scipy.ndimage.interpolation.shift( 
                    templateImageGray, ( 4.0*transY, 4.0*transX ) )
        
        #newImage = bestTransformedImage
        #newImage = np.array( 
        #    scipy.ndimage.interpolation.zoom( bestTransformedImage.astype( np.uint8 ), 4.0 ) )
        #newImage[ newImage > 255 ] = 255
        #newImage = np.array( newImage, dtype=np.uint8 )
        #newImage[ newImage > 0 ] = 255
        #cv.Dilate( newImage, newImage )
        return ( transX, transY, rotationAngle, newImage )
