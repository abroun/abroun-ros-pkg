
import cv

#-------------------------------------------------------------------------------
class OpticalFlowFilter:
    
    #---------------------------------------------------------------------------
    def __init__( self, 
        opticalFlowBlockWidth, opticalFlowBlockHeight, 
        opticalFlowRangeWidth, opticalFlowRangeHeight ):
            
        self.opticalFlowBlockWidth = opticalFlowBlockWidth
        self.opticalFlowBlockHeight = opticalFlowBlockHeight
        self.opticalFlowRangeWidth = opticalFlowRangeWidth
        self.opticalFlowRangeHeight = opticalFlowRangeHeight
        
        self.lastImageGray = None

    #---------------------------------------------------------------------------
    def getLastImageGray( self ):
        return self.lastImageGray

    #---------------------------------------------------------------------------
    def calcOpticalFlowWidth( self, imageWidth ):
        return (imageWidth - self.opticalFlowBlockWidth)/self.opticalFlowBlockWidth
        
    #---------------------------------------------------------------------------
    def calcOpticalFlowHeight( self, imageHeight ):
        return (imageHeight - self.opticalFlowBlockHeight)/self.opticalFlowBlockHeight
        
    #---------------------------------------------------------------------------
    def calcOpticalFlow( self, curImageGray ):
        
        if curImageGray.channels != 1 or curImageGray.depth != 8:
            raise Exception( "Only able to process gray-scale images" )
        
        if self.lastImageGray == None:
            lastImageGray = curImageGray
        else:
            lastImageGray = self.lastImageGray
        
        # Create storage for the optical flow
        storageWidth = self.calcOpticalFlowWidth( lastImageGray.width )
        storageHeight = self.calcOpticalFlowHeight( lastImageGray.height )
        
        opticalFlowArrayX = cv.CreateMat( storageHeight, storageWidth, cv.CV_32FC1 )
        opticalFlowArrayY = cv.CreateMat( storageHeight, storageWidth, cv.CV_32FC1 )
            
        cv.CalcOpticalFlowBM( lastImageGray, curImageGray, 
            ( self.opticalFlowBlockWidth, self.opticalFlowBlockHeight ),
            ( self.opticalFlowBlockWidth, self.opticalFlowBlockHeight ),
            ( self.opticalFlowRangeWidth, self.opticalFlowRangeHeight ),
            0, opticalFlowArrayX, opticalFlowArrayY )
            
        # Save the current image
        self.lastImageGray = curImageGray
        
        return ( opticalFlowArrayX, opticalFlowArrayY )
