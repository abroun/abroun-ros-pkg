
import numpy as np

#-------------------------------------------------------------------------------
class ROCCurve:
    '''The ROC curve for a classifier run'''
    
    DEFAULT_THRESHOLD_STEP_SIZE = 0.01
    
    #---------------------------------------------------------------------------
    def __init__( self, thresholdStepSize = None ):
        
        if thresholdStepSize == None:
            thresholdStepSize = self.DEFAULT_THRESHOLD_STEP_SIZE
            
        self.thresholdStepSize = thresholdStepSize
        self.truePositiveRates = []
        self.falsePositiveRates = []
        self.sensitivity = self.truePositiveRates
        self.specificity = []
        self.accuracy = []
        
    #---------------------------------------------------------------------------
    def calculateThresholds( self ):
        
        return np.arange( 0.0, 1.0 + self.thresholdStepSize, self.thresholdStepSize )
        
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
        
        thresholds = curveList[ 0 ].calculateThresholds()
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
    def __init__( self, crossCorrelatedSequence, markerBuffer, thresholdStepSize = None ):
        ROCCurve.__init__( self, thresholdStepSize )
        
        positiveNegativeCounts = np.bincount( markerBuffer.flat )
        actualNegativeCount = positiveNegativeCounts[ 0 ]
        actualPositiveCount = positiveNegativeCounts[ 1 ]
        totalSampleCount = actualNegativeCount + actualPositiveCount
        
        thresholds = self.calculateThresholds()
        
        # Calculate the true positive rate and false positive rate for each
        # threshold value
        for threshold in thresholds:
                        
            inputSignalDetectedArray = \
                crossCorrelatedSequence.detectInputSequence( threshold )
            
            numTruePositives = 0
            numTrueNegatives = 0
            numFalsePositives = 0
            
            for rowIdx in range( markerBuffer.shape[ 0 ] ):
                for colIdx in range( markerBuffer.shape[ 1 ] ):
                    
                    if inputSignalDetectedArray[ rowIdx, colIdx ]:
                        
                        if markerBuffer[ rowIdx, colIdx ]:
                            numTruePositives += 1
                        else:
                            numFalsePositives += 1
                    
                    else:
                        
                        if not markerBuffer[ rowIdx, colIdx ]:
                            numTrueNegatives += 1
                        
        
            truePositiveRate = float( numTruePositives ) / float( actualPositiveCount )
            falsePositiveRate = float( numFalsePositives ) / float( actualNegativeCount )
            self.truePositiveRates.append( truePositiveRate )
            self.falsePositiveRates.append( falsePositiveRate )
            self.specificity.append( 1.0 - falsePositiveRate )
            self.accuracy.append( float(numTruePositives + numTrueNegatives)/totalSampleCount )
