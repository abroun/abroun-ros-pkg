#! /usr/bin/env python

import matplotlib
from matplotlib.figure import Figure
from matplotlib.axes import Subplot
from matplotlib.backends.backend_gtkagg import FigureCanvasGTKAgg as FigureCanvas
import sys

#-------------------------------------------------------------------------------
if len( sys.argv ) < 2:
    print "Error: No datafile given"
    sys.exit( -1 )
    
dataFilename = sys.argv[ 1 ]

# Read the data from the file
dataFile = open( dataFilename, "r" )
lineIdx = 0
aucData = []

for line in dataFile:
    
    if lineIdx == 0:    # Skip the first line
        lineIdx += 1
        continue
    
    lineArray = line.split( "," )
    aucData.append( lineArray[ 1 ] )
    
    lineIdx += 1
    
# Plot the data
numWavesData = range( 1, len( aucData ) + 1 )

figurePlot = Figure( figsize=(8,6), dpi=72 )
canvas = FigureCanvas( figurePlot )
axisPlot = figurePlot.add_subplot( 111 )

axisPlot.plot( numWavesData, aucData, '-x' )
axisPlot.set_xlim( 0.5, len( aucData ) + 0.5 )
axisPlot.set_xticks( numWavesData )

axisPlot.set_xlabel( 'Num Waves' )
axisPlot.set_ylabel( 'AUC' )

# Save the graph
figurePlot.savefig( "AUCvsNumWaves.eps" )
    
