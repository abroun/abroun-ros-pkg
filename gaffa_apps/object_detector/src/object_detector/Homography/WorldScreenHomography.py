
import cv
import numpy as np

#-------------------------------------------------------------------------------
class WorldScreenHomography:
    '''Calculates the homography between a plane in world space and a plane in
       screen space'''

    #---------------------------------------------------------------------------
    def __init__( self, worldPositions, screenPositions ):

        assert len( worldPositions ) == 4
        assert len( screenPositions ) == 4

        worldPosMtx = np.matrix( worldPositions, dtype=np.float32 )
        screenPosMtx = np.matrix( screenPositions, dtype=np.float32 )

        self.worldToScreenHomography = np.matrix( np.ndarray( shape=( 3, 3 ), dtype=np.float32 ) )
        self.screenToWorldHomography = np.matrix( np.ndarray( shape=( 3, 3 ), dtype=np.float32 ) )
        
        cv.FindHomography( worldPosMtx, screenPosMtx, self.worldToScreenHomography )
        cv.Invert( self.worldToScreenHomography, self.screenToWorldHomography )
        
    #---------------------------------------------------------------------------
    def convertWorldPosToScreenPos( self, worldPos ):
        
        w = np.matrix( [ [ worldPos[ 0 ] ], [ worldPos[ 1 ] ], [ 1.0 ] ], dtype=np.float32 )
        s = self.worldToScreenHomography*w
        
        return ( s[ 0, 0 ]/s[ 2, 0 ], s[ 1, 0 ]/s[ 2, 0 ] )

    #---------------------------------------------------------------------------
    def convertScreenPosToWorldPos( self, screenPos ):

        s = np.matrix( [ [ screenPos[ 0 ] ], [ screenPos[ 1 ] ], [ 1.0 ] ], dtype=np.float32 )
        w = self.screenToWorldHomography*s
        
        return ( w[ 0, 0 ]/w[ 2, 0 ], w[ 1, 0 ]/w[ 2, 0 ] )