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
dataDic = {}
gaussianStdDev = 0.0
aucData = None

for line in dataFile:
    
    lineArray = line.split( "," )
    if len( lineArray ) == 1:
        if aucData != None:
            dataDic[ str( gaussianStdDev ) ] = aucData
        aucData = []
        gaussianStdDev = lineArray[ 0 ]
    else:
        aucData.append( lineArray[ 1 ] )
    
    lineIdx += 1
    
if aucData != None:
    dataDic[ str( gaussianStdDev ) ] = aucData

# Plot the data
numWaves = len( aucData )
numWavesData = range( 1, numWaves + 1 )

figurePlot = Figure( figsize=(8,6), dpi=72 )
canvas = FigureCanvas( figurePlot )
axisPlot = figurePlot.add_subplot( 111 )

legendKeys = []
for stdDev in dataDic:
    legendKeys.append( stdDev )
    axisPlot.plot( numWavesData, dataDic[ stdDev ], '-x' )
    
axisPlot.legend( legendKeys, loc=4, title='Gaussian $\sigma$' )

axisPlot.set_xlim( 0.5, numWaves + 0.5 )
axisPlot.set_xticks( numWavesData )

axisPlot.set_xlabel( 'Num Waves' )
axisPlot.set_ylabel( 'AUC' )

# Save the graph
figurePlot.savefig( "SmoothedAUCvsNumWaves.eps" )
    
