
import numpy as np
from WorldScreenHomography import WorldScreenHomography

#-------------------------------------------------------------------------------
class JointScreenHomography:
    '''Calculates the homography between a plane in world space and a plane in
       screen space. The world space plane is defined by 4 joint space positions'''

    DEFAULT_VERTICAL_OFFSET = 0.4

    #---------------------------------------------------------------------------
    def __init__( self, jointSpaceArrays, screenPositions, 
        kinematicInterface, verticalOffset = None ):

        if verticalOffset == None:
            verticalOffset = self.DEFAULT_VERTICAL_OFFSET

        assert len( jointSpaceArrays ) == 4
        assert len( screenPositions ) == 4
        
        self.kinematicInterface = kinematicInterface
        self.worldPlaneHeight = 0.0
        self.verticalOffset = verticalOffset

        # Translate joint space positions to world positions and build a 
        # homography from world space to screen space
        worldPositions = []
        for posIdx in range( 4 ):
            worldPos = self.kinematicInterface.GetEndEffectorPos( jointSpaceArrays[ posIdx ] )
            worldPositions.append( ( worldPos[ 0, 0 ], worldPos[ 1, 0 ] ) )
            
            print worldPos[ 2, 0 ]
            self.worldPlaneHeight += worldPos[ 2, 0 ]

        # Crappy assumptions that
        #   A - The 4 world positions are co-planar
        #   B - They are all in the xy plane
        self.worldPlaneHeight /= 4.0

        self.worldScreenHomography = WorldScreenHomography( worldPositions, screenPositions )
        
    #---------------------------------------------------------------------------
    def convertJointPosToScreenPos( self, jointSpaceArray ):
        
        worldPos = self.kinematicInterface.GetEndEffectorPos( jointSpaceArray )
        return self.worldScreenHomography.convertWorldPosToScreenPos( 
            ( worldPos[ 0, 0 ], worldPos[ 1, 0 ] ) )

    #---------------------------------------------------------------------------
    def convertScreenPosToJointPos( self, screenPos ):

        worldPlanePos = self.worldScreenHomography.convertScreenPosToWorldPos( screenPos )
        worldPos = np.matrix( [
            [ worldPlanePos[ 0 ] ], 
            [ worldPlanePos[ 1 ] ], 
            [ self.worldPlaneHeight + self.verticalOffset ], 
            [ 1.0 ]] )
        return self.kinematicInterface.CalculateJointAnglesIK( worldPos )
