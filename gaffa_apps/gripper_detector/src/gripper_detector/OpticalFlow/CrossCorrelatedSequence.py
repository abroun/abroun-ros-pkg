
import numpy as np
import Utils

#-------------------------------------------------------------------------------    
class CrossCorrelatedSequence:
    '''Contains the result of cross correlating a regularised input sequences 
       servo angles with the optical flow'''
    
    #---------------------------------------------------------------------------   
    def __init__( self, regularisedInputSequence, maxLag, combinationMethod = "NoChange" ):
        '''The x and y components can be combined in the following ways
                NoChange - X and Y components are correlated individually with the input
                OnlyX - Only the X component is correlated
                OnlyY - Only the Y component is correlated
                Magnitude - The magnitude of the (X,Y) vector is used
                Addition - We correlate X+Y and X-Y so we should get constructive 
                    interference on one'''
    
        self.combinationMethod = combinationMethod
        self.correlationChannels = None
    
        maxSampleLag = int( maxLag * regularisedInputSequence.samplesPerSecond )
        if combinationMethod == "NoChange":
            
            correlationsX = np.apply_along_axis( Utils.crossCorrelateComplete, 2, 
                regularisedInputSequence.regularOpticalFlowArrayX, 
                regularisedInputSequence.regularServoAngleData, maxSampleLag )
            correlationsY = np.apply_along_axis( Utils.crossCorrelateComplete, 2, 
                regularisedInputSequence.regularOpticalFlowArrayY, 
                regularisedInputSequence.regularServoAngleData, maxSampleLag )
            
            self.correlationChannels = [ correlationsX, correlationsY ]
            
        elif combinationMethod == "OnlyX":
            
            correlationsX = np.apply_along_axis( Utils.crossCorrelateComplete, 2, 
                regularisedInputSequence.regularOpticalFlowArrayX, 
                regularisedInputSequence.regularServoAngleData, maxSampleLag )
            
            self.correlationChannels = [ correlationsX ]
            
        elif combinationMethod == "OnlyY":
            
            correlationsY = np.apply_along_axis( Utils.crossCorrelateComplete, 2, 
                regularisedInputSequence.regularOpticalFlowArrayY, 
                regularisedInputSequence.regularServoAngleData, maxSampleLag )
            
            self.correlationChannels = [ correlationsY ]
            
        elif combinationMethod == "Magnitude":
            
            opticalFlowMag = np.sqrt( np.add(
                np.square( regularisedInputSequence.regularOpticalFlowArrayX ),
                np.square( regularisedInputSequence.regularOpticalFlowArrayY ) ) )
            
            correlationsMag = np.apply_along_axis( Utils.crossCorrelateComplete, 2, 
                opticalFlowMag, regularisedInputSequence.regularServoAngleData, 
                maxSampleLag )
            
            self.correlationChannels = [ correlationsMag ]
            
        elif combinationMethod == "Addition":
            
            opticalFlowXPlusY = np.add(
                regularisedInputSequence.regularOpticalFlowArrayX,
                regularisedInputSequence.regularOpticalFlowArrayY )
            opticalFlowXMinusY = np.subtract(
                regularisedInputSequence.regularOpticalFlowArrayX,
                regularisedInputSequence.regularOpticalFlowArrayY )
            
            correlationsXPlusY = np.apply_along_axis( Utils.crossCorrelateComplete, 2, 
                opticalFlowXPlusY, regularisedInputSequence.regularServoAngleData, 
                maxSampleLag )
            correlationsXMinusY = np.apply_along_axis( Utils.crossCorrelateComplete, 2, 
                opticalFlowXMinusY, regularisedInputSequence.regularServoAngleData, 
                maxSampleLag )
            
            self.correlationChannels = [ correlationsXPlusY, correlationsXMinusY ]
        
        self.maxAbsCorrelations = [
            np.maximum.reduce( np.absolute( channel ), axis=2 ) \
            for channel in self.correlationChannels ]
        
    #---------------------------------------------------------------------------      
    def detectInputSequence( self, threshold ):
        
        inputSignalDetectedArray = self.getBlockScores() >= threshold
        return inputSignalDetectedArray
        
    #---------------------------------------------------------------------------      
    def getBlockScores( self ):
        
        blockScores = 0
        for maxCorrelationArray in self.maxAbsCorrelations:
            blockScores = np.maximum( blockScores, maxCorrelationArray )
        
        return blockScores