
import math
import numpy as np
import Utils
import scipy.ndimage

#-------------------------------------------------------------------------------
class RegularisedInputSequence:
    '''A regularised form of an input sequence where the optical flow and the
       servo angles have been resampled so that they have a consistent regular
       sample time'''
      
    #--------------------------------------------------------------------------- 
    def __init__( self, inputSequence, samplesPerSecond ):
        
        assert inputSequence.opticalFlowCalculated
        
        # We assume that the image sequence goes on for longer than the angle sequence.
        # If it doesn't then we have to write code to pad the end of the optical
        # flow data with zeros
        assert inputSequence.imageTimes[ -1 ] > inputSequence.servoAngleTimes[ -1 ]
        
        self.samplesPerSecond = samplesPerSecond
        
        # Construct regular sampled data from the input servo data
        dataDuration = math.floor( inputSequence.imageTimes[ -1 ] )
        
        self.regularSampleTimes = np.arange( 0.0, dataDuration, 1.0/self.samplesPerSecond )
        
        # Pad the servo readings with zeros at the start and end
        preTimes = np.arange( 0.0, inputSequence.servoAngleTimes[ 0 ], 0.1 )
        preReadings = np.zeros( len( preTimes ) )
        postTimes = np.arange( inputSequence.servoAngleTimes[ -1 ] + 0.1, dataDuration + 1.0, 0.1 )
        postReadings = np.zeros( len( postTimes ) )
        
        npAngleTimes = np.concatenate( [ preTimes, np.array( inputSequence.servoAngleTimes ), postTimes ] )
        npAngleData = np.concatenate( [ preReadings, np.array( inputSequence.servoAngleData ), postReadings ] )
        
        self.regularServoAngleData = Utils.resampleSequence( 
            npAngleData, npAngleTimes, self.regularSampleTimes )
                    
        self.regularOpticalFlowArrayX = np.apply_along_axis( Utils.resampleSequence, 2, 
            inputSequence.opticalFlowArraysX, inputSequence.imageTimes, self.regularSampleTimes )
        self.regularOpticalFlowArrayY = np.apply_along_axis( Utils.resampleSequence, 2, 
            inputSequence.opticalFlowArraysY, inputSequence.imageTimes, self.regularSampleTimes )            
           
    #--------------------------------------------------------------------------- 
    def smoothOpticalFlow( self, gaussianStdDev ):
        self.regularOpticalFlowArrayX = np.apply_along_axis( 
            scipy.ndimage.gaussian_filter1d,
            2, self.regularOpticalFlowArrayX, gaussianStdDev )
        self.regularOpticalFlowArrayY = np.apply_along_axis( 
            scipy.ndimage.gaussian_filter1d,
            2, self.regularOpticalFlowArrayY, gaussianStdDev )
