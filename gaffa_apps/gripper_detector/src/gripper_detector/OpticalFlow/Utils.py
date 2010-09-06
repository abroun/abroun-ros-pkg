
import math
import numpy as np
import scipy.signal
import scipy.interpolate
import scipy.ndimage

#-------------------------------------------------------------------------------
def crossCorrelateComplete( sequence, laggedSequence, maxLag = None ):
    '''Calculates the cross correlation of a sequence for all possible lags'''
    
    if maxLag != None:
        numCorrelations = maxLag + 1
    else:
        numCorrelations = len( sequence )

    # x
    x = sequence
    sum_x = np.add.accumulate( x[ ::-1 ] )[ ::-1 ] 
    sum_x2 = np.add.accumulate( np.square( x )[ ::-1 ] )[ ::-1 ] 
    len_x = np.arange( len( x ), 0, -1 )

    #var_x = np.divide( np.subtract( np.multiply( sum_x2, len_x ), np.square( sum_x ) ), np.square( len_x ) )
    var_x = np.subtract( np.multiply( sum_x2, len_x ), np.square( sum_x ) )

    # y
    y = laggedSequence

    sum_y = np.add.accumulate( y )
    sum_y2 = np.add.accumulate( np.square( y ) )
    len_y = np.arange( 1, len( y ) + 1, 1 )

    #var_y = np.divide( np.subtract( np.multiply( sum_y2, len_y ), np.square( sum_y ) ), np.square( len_y ) )
    var_y = np.subtract( np.multiply( sum_y2, len_y ), np.square( sum_y ) )

    # xy
    sum_xy = np.correlate( x, y, mode="full" )[ len( x ) - 1: ]
    #cov_xy = np.divide( np.subtract( np.multiply( sum_xy, len_x ), np.multiply( sum_x, sum_y[::-1] ) ), np.square( len_x ) )
    cov_xy = np.subtract( np.multiply( sum_xy[ : numCorrelations], len_x[ : numCorrelations] ), np.multiply( sum_x[ : maxLag+1], sum_y[::-1][ : numCorrelations] ) )

    corrCoeff = np.divide( cov_xy, np.sqrt( np.multiply( var_x[ : numCorrelations], var_y[ ::-1 ][ : numCorrelations] ) ) )
    corrCoeff[ np.logical_or( np.isnan( corrCoeff ), np.isinf( corrCoeff ) ) ] = 0.0
    
    return corrCoeff
            
#-------------------------------------------------------------------------------
def normaliseSequence( sequence ):
    '''Normalises a sequence of numbers by subtracting the mean and then dividing
       by the standard deviation. This means that the sequence is centred at 0 and
       'mostly' within the range [-1,1]'''
    
    result = []
    
    numElements = len( sequence )
    if numElements > 0:
        
        mean = sum( sequence )/numElements
        
        deviations = [ element - mean for element in sequence ]
        #stdDev = math.sqrt( sum( [ deviation*deviation for deviation in deviations ] ) / numElements )
        maxVal = math.sqrt( max( [ deviation*deviation for deviation in deviations ] ) )
        
        if maxVal == 0:
            result = deviations
        else:
            result = [ deviation / maxVal for deviation in deviations ]
        
    return result
    
#-------------------------------------------------------------------------------
def resampleSequence( sequenceData, sequenceTimes, newSequenceTimes ):
    
    # Construct a spline to represent the sequence
    tck = scipy.interpolate.splrep( sequenceTimes, sequenceData )

    # Evaluate spline at new sample points
    return scipy.interpolate.splev( newSequenceTimes,tck )
    #return scipy.ndimage.gaussian_filter1d( scipy.interpolate.splev( newSequenceTimes,tck ), 5.0 )
    