
import numpy as np

#-------------------------------------------------------------------------------
class ROCCurve:
    '''The ROC curve for a classifier run'''
    
    DEFAULT_THRESHOLD_STEP_SIZE = 0.01
    
    #---------------------------------------------------------------------------
    def __init__( self, crossCorrelatedSequence, markerBuffer, thresholdStepSize = None ):
        
        if thresholdStepSize == None:
            thresholdStepSize = self.DEFAULT_THRESHOLD_STEP_SIZE
                
        positiveNegativeCounts = np.bincount( markerBuffer.flat )
        actualNegativeCount = positiveNegativeCounts[ 0 ]
        actualPositiveCount = positiveNegativeCounts[ 1 ]
        
        positiveProportion = actualPositiveCount / ( actualNegativeCount + actualPositiveCount )
        negativeProportion = 1.0 - positiveProportion
        
        self.truePositiveRates = []
        self.falsePositiveRates = []
        self.sensitivity = self.truePositiveRates
        self.specificity = []
        self.accuracy = []
        self.thresholds = np.arange( 0.0, 1.0 + thresholdStepSize, thresholdStepSize )
        
        # Calculate the true positive rate and false positive rate for each
        # threshold value
        for threshold in self.thresholds:
                        
            inputSignalDetectedArray = \
                crossCorrelatedSequence.detectInputSequence( threshold )
            
            numTruePositives = 0
            numFalsePositives = 0
            
            for rowIdx in range( markerBuffer.shape[ 0 ] ):
                for colIdx in range( markerBuffer.shape[ 1 ] ):
                    
                    if inputSignalDetectedArray[ rowIdx, colIdx ]:
                        
                        if markerBuffer[ rowIdx, colIdx ]:
                            numTruePositives += 1
                        else:
                            numFalsePositives += 1
        
            truePositiveRate = float( numTruePositives ) / float( actualPositiveCount )
            falsePositiveRate = float( numFalsePositives ) / float( actualNegativeCount )
            self.truePositiveRates.append( truePositiveRate )
            self.falsePositiveRates.append( falsePositiveRate )
            self.specificity.append( 1.0 - falsePositiveRate )
            #self.accuracy.append( positiveProportion*truePositiveRate \
            #    + negativeProportion*( 1.0 - truePositiveRate )

        # TODO: Fix accuracy calculation and calculate area under curve