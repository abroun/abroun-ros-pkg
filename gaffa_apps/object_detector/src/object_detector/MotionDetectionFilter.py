
import cv
import numpy as np

#-------------------------------------------------------------------------------
class MotionDetectionFilter:
    
    NUM_FRAMES_IN_MODEL = 10
    MINIMUM_MODEL_STD_DEV = 0.8
    
    #---------------------------------------------------------------------------
    def __init__( self ):
        
        self.pixelModelBuffer = None
        self.pixelModelBuilt = False
        self.imageSizeSet = False
        self.firstImageWidth = 0
        self.firstImageHeight = 0
        
    #---------------------------------------------------------------------------
    def isPixelModelBuilt( self ):
        return self.pixelModelBuilt
        
    #---------------------------------------------------------------------------
    def isImageCorrectSize( self, image ):
        
        if not self.imageSizeSet:
            return True
        
        return image.width == self.firstImageWidth \
            and image.height == self.firstImageHeight
        
    #---------------------------------------------------------------------------
    def addFrameForPixelModel( self, imageGray ):
        
        if self.isPixelModelBuilt():
            return # Nothing to do
        
        if self.pixelModelBuffer == None:
            self.firstImageWidth = imageGray.width
            self.firstImageHeight = imageGray.height
            self.imageSizeSet = True
            
            self.pixelModelBuffer = np.ndarray( 
                ( imageGray.height, imageGray.width, self.NUM_FRAMES_IN_MODEL ), 
                dtype=np.uint8 )
            
            self.numPixelModelFrames = 0
            
        if not self.isImageCorrectSize( imageGray ):
            raise Exception( "The input image size has changed. It must remain constant" )
        
        # Add the image to the model
        self.pixelModelBuffer[ :, :, self.numPixelModelFrames ] = imageGray
        self.numPixelModelFrames += 1
        
        # When all frames have been collected, construct the model
        if self.numPixelModelFrames == self.NUM_FRAMES_IN_MODEL:
            
            stdDev = np.std( self.pixelModelBuffer, axis=2 )
            
            # Have a minimum value for the standard deviation so that we don't have
            # highly sensitive pixels
            print "MaxStdDev", np.max( stdDev )
            stdDev[ stdDev < self.MINIMUM_MODEL_STD_DEV ] = 3.0 #self.MINIMUM_MODEL_STD_DEV
            stdDev[ stdDev > self.MINIMUM_MODEL_STD_DEV ] = 3.0
            
            self.pixelLowerThreshold = 10.0*stdDev
            self.pixelUpperThreshold = 25.0*stdDev
            self.pixelThresholdDiff = self.pixelUpperThreshold - self.pixelLowerThreshold
            self.pixelMean = np.mean( self.pixelModelBuffer, axis=2 )
            
            self.pixelModelBuilt = True
        
    #---------------------------------------------------------------------------
    def calcMotion( self, imageGray ):
        
        motionImage = None
        
        if not self.isPixelModelBuilt():
            self.addFrameForPixelModel( imageGray )
            
            # No motion yet so just return a blank image
            motionImage = np.zeros( ( imageGray.height, imageGray.width ), dtype=np.uint8 )
            
        else:        
            imageAbsDiff = np.abs( np.array( imageGray, dtype=np.int32 ) - self.pixelMean )
            
            prob = ( imageAbsDiff - self.pixelLowerThreshold ) / self.pixelThresholdDiff
            prob *= 255.0
            np.clip( prob, 0.0, 255.0, prob )
            motionImage = np.array( prob, dtype=np.uint8 )
            
            self.pixelMean = np.copy( imageGray ).astype( np.int32 )
    
        return motionImage
