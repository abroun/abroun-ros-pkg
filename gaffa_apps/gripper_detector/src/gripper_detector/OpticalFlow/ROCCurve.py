
#
# Uses algorithms from 
#   [Fawcett06] An introduction to ROC analysis by Tom Fawcett

#

import numpy as np

#-------------------------------------------------------------------------------
class ROCCurve:
    '''The ROC curve for a classifier run'''
    
    DEFAULT_THRESHOLD_STEP_SIZE = 0.01
    
    #---------------------------------------------------------------------------
    def __init__( self ):
            
        self.truePositiveRates = []
        self.falsePositiveRates = []
        self.sensitivity = self.truePositiveRates
        self.specificity = []
        self.accuracy = []
        self.score = []
        
    #---------------------------------------------------------------------------
    @staticmethod
    def averageROCCurves( curveList ):
        
        numCurves = len( curveList )
        assert numCurves > 0
        
        thresholdStepSize = curveList[ 0 ].thresholdStepSize
        for curveIdx in range( 1, numCurves ):
            if curveList[ curveIdx ].thresholdStepSize != thresholdStepSize:
                raise Exception( "The threshold step size must be the same for all ROC curves" )
            
        avgROCCurve = ROCCurve( thresholdStepSize )
        varianceROCCurve = ROCCurve( thresholdStepSize )
        
        thresholds = curveList[ 0 ].thresholds
        for thresholdIdx in range( len( thresholds ) ):
            
            # Calculate average for threshold
            avgTruePositiveRate = 0
            avgFalsePositiveRate = 0
            avgSpecificity = 0
            avgAccuracy = 0
            for curve in curveList:
                
                avgTruePositiveRate += curve.truePositiveRates[ thresholdIdx ]
                avgFalsePositiveRate += curve.falsePositiveRates[ thresholdIdx ]
                avgSpecificity += curve.specificity[ thresholdIdx ]
                avgAccuracy += curve.accuracy[ thresholdIdx ]    
                
            avgTruePositiveRate /= numCurves
            avgFalsePositiveRate /= numCurves
            avgSpecificity /= numCurves
            avgAccuracy /= numCurves
            
            avgROCCurve.truePositiveRates.append( avgTruePositiveRate )
            avgROCCurve.falsePositiveRates.append( avgFalsePositiveRate )
            avgROCCurve.specificity.append( avgSpecificity )
            avgROCCurve.accuracy.append( avgAccuracy )
 
            # Also calculate variance
            varianceTruePositiveRate = 0
            varianceFalsePositiveRate = 0
            varianceSpecificity = 0
            varianceAccuracy = 0
            for curve in curveList:
                
                varianceTruePositiveRate += (curve.truePositiveRates[ thresholdIdx ] - avgTruePositiveRate)**2
                varianceFalsePositiveRate += (curve.falsePositiveRates[ thresholdIdx ] - avgFalsePositiveRate)**2
                varianceSpecificity += (curve.specificity[ thresholdIdx ] - avgSpecificity)**2
                varianceAccuracy += (curve.accuracy[ thresholdIdx ] - avgAccuracy)**2
                
            varianceTruePositiveRate /= numCurves
            varianceFalsePositiveRate /= numCurves
            varianceSpecificity /= numCurves
            varianceAccuracy /= numCurves
            
            varianceROCCurve.truePositiveRates.append( varianceTruePositiveRate )
            varianceROCCurve.falsePositiveRates.append( varianceFalsePositiveRate )
            varianceROCCurve.specificity.append( varianceSpecificity )
            varianceROCCurve.accuracy.append( varianceAccuracy )
        
        return (avgROCCurve, varianceROCCurve)
        
        
#-------------------------------------------------------------------------------
class GripperDetectorROCCurve( ROCCurve ):
    '''The ROC curve for the gripper detector classifier'''
    
    #---------------------------------------------------------------------------
    def __init__( self, crossCorrelatedSequence, markerBuffer ):
        ROCCurve.__init__( self )
        
        # Get the maximum cross correlation value from each optical flow block
        flatBlockScores = crossCorrelatedSequence.getBlockScores()
        
        # The marker buffer matches the block scores one to one and gives the
        # ground truth of whether the block is part of the gripper or not
        flatMarkerBuffer = markerBuffer.flat
        positiveNegativeCounts = np.bincount( flatMarkerBuffer )
        actualNegativeCount = positiveNegativeCounts[ 0 ]
        actualPositiveCount = positiveNegativeCounts[ 1 ]
        totalSampleCount = actualNegativeCount + actualPositiveCount
        
        # Build the ROC curve using the algorithm given in Fawcett06
        
        # Get indices that are ordered so that the block scores are sorted from
        # high to low
        sortedIndices = np.flipud( np.argsort( flatBlockScores ) )
        
        numTruePositives = 0
        numFalsePositives = 0
        numTrueNegatives = 0
        lastScore = None
        
        for i in range( len( flatBlockScores ) ):
            
            score = flatBlockScores[ sortedIndices[ i ] ]
            if score != lastScore:
                
                # New ROC point
                truePositiveRate = float( numTruePositives ) / float( actualPositiveCount )
                falsePositiveRate = float( numFalsePositives ) / float( actualNegativeCount )
                self.truePositiveRates.append( truePositiveRate )
                self.falsePositiveRates.append( falsePositiveRate )
                self.specificity.append( 1.0 - falsePositiveRate )
                self.accuracy.append( float(numTruePositives + numTrueNegatives)/totalSampleCount )
                self.scores.append( score )
                
                lastScore = score
        
            if flatMarkerBuffer[ sortedIndices[ i ] ] == True:
                numTruePositives += 1
            else:
                numFalsePositives += 1
                
        # Make sure that we have an end point for the threshold that lets everything through
        self.truePositiveRates.append( 1.0 )
        self.falsePositiveRates.append( 1.0 )
        self.specificity.append( 0.0 )
        self.accuracy.append( float(numTruePositives + numTrueNegatives)/totalSampleCount )
        self.scores.append( 0.0 )
