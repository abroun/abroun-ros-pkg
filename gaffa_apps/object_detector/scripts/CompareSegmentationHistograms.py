#! /usr/bin/env python

# ROS imports
import roslib
roslib.load_manifest( 'object_detector' )

import cv
import numpy as np

learntHistogramData = {
    "Screwdriver" : [ "Screwdriver_Prod_2010-09-24-15-24-25_segmentation.png",
        "Screwdriver_Prod_2010-09-24-15-24-52_segmentation.png" ],
    "Sellotape" : [ "Sellotape_Prod_2010-09-24-15-22-02_segmentation.png",
        "Sellotape_Prod_2010-09-24-15-22-24_segmentation.png" ],
    "Stapler" : [ "Stapler_2010-11-24-21-51-01_segmentation.png",
        "Stapler_2010-11-24-21-51-15_segmentation.png",
        "Stapler_2010-11-24-21-51-29_segmentation.png" ],
    "Tissue" : [ "Tissue_Prod_2010-09-24-15-17-40_segmentation.png",
        "Tissue_Prod_2010-09-24-15-18-09_segmentation.png",
        "Tissue_Prod_2010-09-24-15-18-36_segmentation.png",
        "Tissue_Prod_2010-09-24-15-19-03_segmentation.png" ],
    "WaterPistol" : [ "WaterPistol_2010-11-19-23-24-46_segmentation.png",
        "WaterPistol_2010-11-20-00-26-43_segmentation.png",
        "WaterPistol_2010-11-20-00-41-11_segmentation.png",
        "WaterPistol_2010-11-22-15-40-51_segmentation.png" ]
}

testImages = {
    "Screwdriver" : "Screwdriver_Prod_2010-09-24-15-25-47_segmentation.png",
    "Sellotape" : "Sellotape_Prod_2010-09-24-15-23-09_segmentation.png",
    "Stapler" : "Stapler_2010-11-24-21-51-44_segmentation.png",
    "Tissue" : "Tissue_Prod_2010-09-24-15-19-29_segmentation.png",
    "WaterPistol" : "WaterPistol_2010-11-22-15-41-09_segmentation.png",
}

SEGMENTATION_DIR = "/home/abroun/abroun-ros-pkg/gaffa_apps/object_detector/test_data/impact_images/segmentations"

# Create the learnt histograms
learntHistograms = {}
for objectName in learntHistogramData.keys():
    
    histogram = cv.CreateHist( 
        [ 256/8, 256/8, 256/8 ], cv.CV_HIST_ARRAY, [ (0,255), (0,255), (0,255) ], 1 )
    cv.ClearHist( histogram )
    
    
    imageFilenameList = learntHistogramData[ objectName ]
    
    for imageFilename in imageFilenameList:
        
        if objectName == "Sellotape":
            print imageFilename
         
        image = cv.LoadImageM( SEGMENTATION_DIR + "/" + imageFilename )
        maskArray = np.zeros( ( image.height, image.width ), dtype=np.uint8 )
        
        imageNP = np.copy( np.array( image, dtype=np.uint8 ) )
        maskedPixels = (imageNP[ :, :, 0 ] == 255) & (imageNP[ :, :, 1 ] == 0) & (imageNP[ :, :, 2 ] == 255)
        #maskedPixels = (imageNP[ :, :, 0 ] >= 255) & (imageNP[ :, :, 1 ] <= 0) & (imageNP[ :, :, 2 ] >= 255)
        maskArray[ maskedPixels == False ] = 255
            
        r_plane = cv.CreateMat( image.height, image.width, cv.CV_8UC1 )
        g_plane = cv.CreateMat( image.height, image.width, cv.CV_8UC1 )
        b_plane = cv.CreateMat( image.height, image.width, cv.CV_8UC1 )
        cv.Split( image, r_plane, g_plane, b_plane, None )
        
        planes = [ r_plane, g_plane, b_plane ]    
        cv.CalcHist( [ cv.GetImage( i ) for i in planes ], 
                histogram, accumulate=1, mask=maskArray )
                
    # Normalise the histogram
    cv.NormalizeHist( histogram, 1.0 )
    
    if objectName == "Sellotape":
        savedHistogram = histogram
                
    # Store the histogram
    learntHistograms[ objectName ] = histogram

# Now compare the test images to the learnt histograms
for objectName in testImages.keys():

    image = cv.LoadImageM( SEGMENTATION_DIR + "/" + testImages[ objectName ] )
    
    # Create a histogram for the test image
    maskArray = np.zeros( ( image.height, image.width ), dtype=np.uint8 )
        
    imageNP = np.array( image, dtype=np.uint8 )
    maskedPixels = (imageNP[ :, :, 0 ] >= 255) & (imageNP[ :, :, 1 ] <= 0) & (imageNP[ :, :, 2 ] >= 255)
    maskArray[ maskedPixels == False ] = 255
    
    
        
    r_plane = cv.CreateMat( image.height, image.width, cv.CV_8UC1 )
    g_plane = cv.CreateMat( image.height, image.width, cv.CV_8UC1 )
    b_plane = cv.CreateMat( image.height, image.width, cv.CV_8UC1 )
    cv.Split( image, r_plane, g_plane, b_plane, None )
    
    planes = [ r_plane, g_plane, b_plane ]    
    
    histogram = cv.CreateHist( 
        [ 256/8, 256/8, 256/8 ], cv.CV_HIST_ARRAY, [ (0,255), (0,255), (0,255) ], 1 )
    cv.ClearHist( histogram )
    cv.CalcHist( [ cv.GetImage( i ) for i in planes ], 
            histogram, accumulate=0, mask=maskArray )
    cv.NormalizeHist( histogram, 1.0 )

    # Test it against each of the learnt histograms in turn
    print objectName
    for otherObjectName in learntHistograms.keys():
        print "    " + otherObjectName + ": " \
            + str( cv.CompareHist( histogram, learntHistograms[ otherObjectName ], cv.CV_COMP_INTERSECT ) )
            #learntHistograms[ otherObjectName ], cv.CV_COMP_INTERSECT ) )
    
    #if objectName == "Sellotape":
    #    savedHistogram = histogram
        
        #z = np.array( r_plane )
        #x = np.array( g_plane )
        #y = np.array( b_plane )
        #print z[ (z > 0) & (x > 0) & (y > 0) & (maskArray == 255) ].size
        
        #for i in range( 8 ):
            #for j in range( 8 ):
                #a = ""
                #for k in range( 8 ):
                    #a += str( cv.QueryHistValue_3D( histogram, i, j, k )  )
                    #a += " "
                
                #print a
            
    #print maskArray[ maskArray == 0 ].size
    #asd = np.copy( imageNP )
    #bsd = np.copy( imageNP )
    #imageNP[ maskArray == 255, 0 ] = 255 
    #imageNP[ maskArray == 255, 1 ] = 255 
    #imageNP[ maskArray == 255, 2 ] = 255
    #cv.SaveImage( objectName + ".png", imageNP )
    
#for i in range( 8 ):
    #for j in range( 8 ):
        #a = ""
        #for k in range( 8 ):
            #a += str( cv.QueryHistValue_3D( savedHistogram, i, j, k )  )
            #a += " "
        
        #print a
