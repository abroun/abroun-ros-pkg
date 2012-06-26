
import cv
import numpy as np

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
        #return int((imageWidth - self.opticalFlowBlockWidth)/self.opticalFlowBlockWidth)
        return int(imageWidth/self.opticalFlowBlockWidth)   # AB: Changed for OpenCV 2.4
        
    #---------------------------------------------------------------------------
    def calcOpticalFlowHeight( self, imageHeight ):
        #return int((imageHeight - self.opticalFlowBlockHeight)/self.opticalFlowBlockHeight)
        return int(imageHeight/self.opticalFlowBlockHeight) # AB: Changed for OpenCV 2.4
        
    #---------------------------------------------------------------------------
    def calcOpticalFlow( self, curImageGray, method="BlockMatching" ):
        
        if curImageGray.channels != 1:
            raise Exception( "Only able to process gray-scale images" )
        
        if self.lastImageGray == None:
            lastImageGray = curImageGray
        else:
            lastImageGray = self.lastImageGray
        
        # Create storage for the optical flow
        storageWidth = self.calcOpticalFlowWidth( lastImageGray.width )
        storageHeight = self.calcOpticalFlowHeight( lastImageGray.height )
        
        if method == "BlockMatching":
            opticalFlowArrayX = np.ndarray( shape=( storageHeight, storageWidth ), dtype=np.float32 )
            opticalFlowArrayY = np.ndarray( shape=( storageHeight, storageWidth ), dtype=np.float32 )
                
            cv.CalcOpticalFlowBM( lastImageGray, curImageGray, 
                ( self.opticalFlowBlockWidth, self.opticalFlowBlockHeight ),
                ( self.opticalFlowBlockWidth, self.opticalFlowBlockHeight ),
                ( self.opticalFlowRangeWidth, self.opticalFlowRangeHeight ),
                0, cv.fromarray( opticalFlowArrayX ), cv.fromarray( opticalFlowArrayY ) )
            
        elif method == "LucasKanade":
            
            largeOpticalFlowArrayX = np.ndarray( shape=( lastImageGray.height, lastImageGray.width ), dtype=np.float32 )
            largeOpticalFlowArrayY = np.ndarray( shape=( lastImageGray.height, lastImageGray.width ), dtype=np.float32 )
            
            cv.CalcOpticalFlowLK( lastImageGray, curImageGray,
                ( 15, 15 ), #( self.opticalFlowBlockWidth, self.opticalFlowBlockHeight ),
                cv.fromarray( largeOpticalFlowArrayX ), cv.fromarray( largeOpticalFlowArrayY ) )
                
            indexGrid = np.mgrid[ 0:storageHeight, 0:storageWidth ]
            indexGrid[ 0 ] = indexGrid[ 0 ]*self.opticalFlowBlockHeight + self.opticalFlowBlockHeight/2
            indexGrid[ 1 ] = indexGrid[ 1 ]*self.opticalFlowRangeWidth + self.opticalFlowRangeWidth/2
            opticalFlowArrayX = largeOpticalFlowArrayX[ indexGrid[ 0 ], indexGrid[ 1 ] ]
            opticalFlowArrayY = largeOpticalFlowArrayY[ indexGrid[ 0 ], indexGrid[ 1 ] ]
        elif method == "HornSchunck":
            
            largeOpticalFlowArrayX = np.ndarray( shape=( lastImageGray.height, lastImageGray.width ), dtype=np.float32 )
            largeOpticalFlowArrayY = np.ndarray( shape=( lastImageGray.height, lastImageGray.width ), dtype=np.float32 )
            
            cv.CalcOpticalFlowHS( lastImageGray, curImageGray,
                0, cv.fromarray( largeOpticalFlowArrayX ), cv.fromarray( largeOpticalFlowArrayY ),
                1.0, (cv.CV_TERMCRIT_ITER | cv.CV_TERMCRIT_EPS, 10, 0.01) )
                
            indexGrid = np.mgrid[ 0:storageHeight, 0:storageWidth ]
            indexGrid[ 0 ] = indexGrid[ 0 ]*self.opticalFlowBlockHeight + self.opticalFlowBlockHeight/2
            indexGrid[ 1 ] = indexGrid[ 1 ]*self.opticalFlowRangeWidth + self.opticalFlowRangeWidth/2
            opticalFlowArrayX = largeOpticalFlowArrayX[ indexGrid[ 0 ], indexGrid[ 1 ] ]
            opticalFlowArrayY = largeOpticalFlowArrayY[ indexGrid[ 0 ], indexGrid[ 1 ] ]
            
        else:
            raise Exception( "Unhandled method" )
            
        # Save the current image
        self.lastImageGray = curImageGray
        
        return ( opticalFlowArrayX, opticalFlowArrayY )
