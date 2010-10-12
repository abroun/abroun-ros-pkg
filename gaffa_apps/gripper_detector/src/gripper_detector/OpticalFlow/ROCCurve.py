
#
# Uses algorithms from 
#   [Fawcett06] An introduction to ROC analysis by Tom Fawcett

#

import numpy as np

#-------------------------------------------------------------------------------
class ROCValues:
    '''Values for the ROC curve at a particular FPR'''
    
    #---------------------------------------------------------------------------
    def __init__( self, truePositiveRate, accuracy, score ):
        
        self.truePositiveRate = truePositiveRate
        self.accuracy = accuracy
        self.score = score

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
        self.scores = []
        self.areaUnderCurve = 0.0

    #---------------------------------------------------------------------------
    def getROCValuesForFPR( self, fpr ):
        
        i = 0
        numPoints = len( self.truePositiveRates )
        
        while i < numPoints - 1 \
            and self.falsePositiveRates[ i + 1 ] < fpr:
            
            i += 1
            
        if self.falsePositiveRates[ i ] == fpr:
            # Exact match found
            return ROCValues( self.truePositiveRates[ i ], self.accuracy[ i ], self.scores[ i ] )
        else:
            # Return an interpolated set of values
            fprDiff = self.falsePositiveRates[ i + 1 ] - self.falsePositiveRates[ i ]
            tprSlope = (self.truePositiveRates[ i + 1 ] - self.truePositiveRates[ i ])/fprDiff
            accuracySlope = (self.accuracy[ i + 1 ] - self.accuracy[ i ])/fprDiff
            scoreSlope = (self.scores[ i + 1 ] - self.scores[ i ])/fprDiff
            
            interpDiff = fpr - self.falsePositiveRates[ i ]
            
            return ROCValues( 
                self.truePositiveRates[ i ] + interpDiff*tprSlope, 
                self.accuracy[ i ] + interpDiff*accuracySlope, 
                self.scores[ i ] + interpDiff*scoreSlope )

    #---------------------------------------------------------------------------
    @staticmethod
    def trapezoidArea( x1, x2, y1, y2 ):
            base = abs( x1 - x2 )
            averageHeight = float( y1 + y2 )/2.0
            return base*averageHeight
            
    #---------------------------------------------------------------------------
    @staticmethod
    def averageROCCurves( curveList, numSteps = 50 ):
        
        numCurves = len( curveList )
        assert numCurves > 0
        
        #thresholdStepSize = curveList[ 0 ].thresholdStepSize
        #for curveIdx in range( 1, numCurves ):
        #    if curveList[ curveIdx ].thresholdStepSize != thresholdStepSize:
        #        raise Exception( "The threshold step size must be the same for all ROC curves" )
            
        avgROCCurve = ROCCurve()
        varianceROCCurve = ROCCurve()
        
        fprThresholdStepSize = 1.0 / numSteps
        avgROCCurve.falsePositiveRates = np.arange( 0.0, 1.0 + fprThresholdStepSize, fprThresholdStepSize )
        avgROCCurve.specificity = np.flipud( avgROCCurve.falsePositiveRates )
        varianceROCCurve.falsePositiveRates = np.arange( 0.0, 1.0 + fprThresholdStepSize, fprThresholdStepSize )
        varianceROCCurve.specificity = np.flipud( varianceROCCurve.falsePositiveRates )
        
        valueList = [ [ curveList[ curveIdx ].getROCValuesForFPR( fpr ) \
            for curveIdx in range( numCurves ) ] for fpr in avgROCCurve.falsePositiveRates ]
            
        prevAvgTPR = 0.0
        prevAvgFPR = 0.0
        
        for i in range( len( avgROCCurve.falsePositiveRates ) ):
            
            # Calculate average for FPR threshold            
            tprList = [ valueList[ i ][ curveIdx ].truePositiveRate for curveIdx in range( numCurves ) ]
            accuracyList = [ valueList[ i ][ curveIdx ].accuracy for curveIdx in range( numCurves ) ]
            scoreList = [ valueList[ i ][ curveIdx ].score for curveIdx in range( numCurves ) ]
            
            avgTruePositiveRate = sum( tprList )/numCurves
            avgAccuracy = sum( accuracyList )/numCurves
            avgScore = sum( scoreList )/numCurves
            
            avgROCCurve.truePositiveRates.append( avgTruePositiveRate )
            avgROCCurve.accuracy.append( avgAccuracy )
            avgROCCurve.scores.append( avgScore )
 
            # Also calculate variance
            tprSquaredDiffList = [ 
                (valueList[ i ][ curveIdx ].truePositiveRate - avgTruePositiveRate)**2 \
                for curveIdx in range( numCurves ) ]
            accuracySquaredDiffList = [ 
                (valueList[ i ][ curveIdx ].accuracy - avgAccuracy)**2 \
                for curveIdx in range( numCurves ) ]
            
            varianceTruePositiveRate = sum( tprSquaredDiffList )/numCurves
            varianceAccuracy = sum( accuracySquaredDiffList )/numCurves
            
            varianceROCCurve.truePositiveRates.append( varianceTruePositiveRate )
            varianceROCCurve.accuracy.append( varianceAccuracy )
            
            # Update the AUC
            avgROCCurve.areaUnderCurve += ROCCurve.trapezoidArea( 
                avgROCCurve.falsePositiveRates[ i ], prevAvgFPR,
                avgTruePositiveRate, prevAvgTPR )
            prevAvgTPR = avgTruePositiveRate
            prevAvgFPR = avgROCCurve.falsePositiveRates[ i ]
        
        return (avgROCCurve, varianceROCCurve)
        
        
#-------------------------------------------------------------------------------
class GripperDetectorROCCurve( ROCCurve ):
    '''The ROC curve for the gripper detector classifier'''
    
    #---------------------------------------------------------------------------
    def __init__( self, crossCorrelatedSequence, markerBuffer ):
        ROCCurve.__init__( self )
        
        # Get the maximum cross correlation value from each optical flow block
        flatBlockScores = crossCorrelatedSequence.getBlockScores().flat
        
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
        
        prevScore = 1.0
        prevNumTruePositives = 0
        prevNumFalsePositives = 0
        
        for i in range( len( flatBlockScores ) ):
            
            score = flatBlockScores[ sortedIndices[ i ] ]
            
            if prevScore != score:

                numTrueNegatives = actualNegativeCount - numFalsePositives

                # New ROC point
                truePositiveRate = float( numTruePositives ) / float( actualPositiveCount )
                falsePositiveRate = float( numFalsePositives ) / float( actualNegativeCount )
                self.truePositiveRates.append( truePositiveRate )
                self.falsePositiveRates.append( falsePositiveRate )
                self.specificity.append( 1.0 - falsePositiveRate )
                self.accuracy.append( float(numTruePositives + numTrueNegatives)/totalSampleCount )
                self.scores.append( score )
                
                self.areaUnderCurve += self.trapezoidArea( 
                    numFalsePositives, prevNumFalsePositives,
                    numTruePositives, prevNumTruePositives )
                
                prevScore = score
                prevNumTruePositives = numTruePositives
                prevNumFalsePositives = numFalsePositives
        
            if flatMarkerBuffer[ sortedIndices[ i ] ] == True:
                numTruePositives += 1
            else:
                numFalsePositives += 1
                
        # Finish calculating the AUC
        self.areaUnderCurve += self.trapezoidArea( 
            actualNegativeCount, prevNumFalsePositives,
            actualPositiveCount, prevNumTruePositives )
        self.areaUnderCurve /= ( actualNegativeCount*actualPositiveCount )  # Scale to the unit square
                    
        # Make sure that we have an end point for the threshold that lets everything through
        self.truePositiveRates.append( 1.0 )
        self.falsePositiveRates.append( 1.0 )
        self.specificity.append( 0.0 )
        self.accuracy.append( float(numTruePositives + numTrueNegatives)/totalSampleCount )
        self.scores.append( 0.0 )
