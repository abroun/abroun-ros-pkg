#! /usr/bin/env python

import numpy as np
import scipy.stats
import matplotlib
from matplotlib.figure import Figure
from matplotlib.axes import Subplot
from matplotlib.backends.backend_gtkagg import FigureCanvasGTKAgg as FigureCanvas
import sys

#  Use preset data
NO_DISTRACTOR_DATA = np.array( [
    0.985403329065,
    0.983354673496,
    0.98339735382,
    0.983475601081,
    0.982365912648 ] )
ARTIFICIAL_DISTRACTOR_DATA = np.array( [
    0.985438896002,
    0.98031014369,
    0.982636221369,
    0.982856736378,
    0.98151230616 ] )
NATURAL_DISTRACTOR_DATA = np.array( [
    0.98681177977,
    0.985467349552,
    0.950320102433,
    0.983809930289,
    0.976184379001 ] )

# Plot bar graph
figurePlot = Figure( figsize=(8,7), dpi=72 )
canvas = FigureCanvas( figurePlot )
axisPlot = figurePlot.add_subplot( 111 )

barWidth = 0.8
barPositions = np.array( [ 1.0, 1.0+1.5*barWidth, 1.0+2.0*1.5*barWidth ] )

meanValues = [ np.mean( NO_DISTRACTOR_DATA ), 
    np.mean( ARTIFICIAL_DISTRACTOR_DATA ), 
    np.mean( NATURAL_DISTRACTOR_DATA ) ]
stdDevValues = [ np.std( NO_DISTRACTOR_DATA ), 
    np.std( ARTIFICIAL_DISTRACTOR_DATA ), 
    np.std( NATURAL_DISTRACTOR_DATA ) ]

axisPlot.bar( barPositions, meanValues, width=barWidth, color='r', yerr=stdDevValues )
axisPlot.set_ylim( 0.96, 1.03 )

# Add the result of the t-test between no distractors and artificial distractors
leftX = barPositions[ 0 ] + barWidth/2.0
rightX = barPositions[ 1 ] + barWidth/2.0
bottomY = meanValues[ 0 ] + 0.01
topY = meanValues[ 0 ] + 0.02
line2D = matplotlib.lines.Line2D( 
    [ leftX, leftX, rightX, rightX ], 
    [ bottomY, topY, topY, bottomY ], color='black' )
    
labelText = "p={0:.3f}".format( scipy.stats.ttest_ind( 
    NO_DISTRACTOR_DATA, ARTIFICIAL_DISTRACTOR_DATA )[ 1 ] )
label = matplotlib.text.Text( (leftX+rightX)/2.0, topY+0.001, labelText, ha='center', va='bottom' )
axisPlot.add_line( line2D )
axisPlot.add_artist( label )

# Add the result of the t-test between no distractors and natural distractors
leftX = barPositions[ 0 ] + barWidth/2.0
rightX = barPositions[ 2 ] + barWidth/2.0
bottomY = meanValues[ 0 ] + 0.025
topY = meanValues[ 0 ] + 0.035
line2D = matplotlib.lines.Line2D( 
    [ leftX, leftX, rightX, rightX ], 
    [ bottomY, topY, topY, bottomY ], color='black' )
  
labelText = "p={0:.3f}".format( scipy.stats.ttest_ind( 
    NO_DISTRACTOR_DATA, NATURAL_DISTRACTOR_DATA )[ 1 ] )
label = matplotlib.text.Text( (leftX+rightX)/2.0, topY+0.001, labelText, ha='center', va='bottom' )
axisPlot.add_line( line2D )
axisPlot.add_artist( label )


axisPlot.set_xlim( barPositions[ 0 ] - barWidth/2.0, barPositions[ -1 ] + 1.5*barWidth )
axisPlot.set_xticks( barPositions + barWidth/2.0 )
axisPlot.set_xticklabels( ('No Distractors', 'Artificial Distractors', 'Natural Distractors' ) )

#axisPlot.set_xticks( numWavesData )

axisPlot.set_xlabel( 'Test Conditions', labelpad=20 )
axisPlot.set_ylabel( 'AUC' )



# Add variation intervals

# Add p value between baseline and distractors

# Save the graph
figurePlot.savefig( "DistractorGraph.eps" )