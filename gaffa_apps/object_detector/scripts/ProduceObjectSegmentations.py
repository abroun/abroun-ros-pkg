#! /usr/bin/env python

import os.path
import subprocess

scriptPath = os.path.dirname( __file__ )

testScript = scriptPath + "/../src/object_detector/ObjectDetectorExplorer.py"

segDataList = [ 
    #scriptPath + "/../test_data/Bags/WaterPistol/WaterPistol_2010-11-20-00-26-43.bag",
    #scriptPath + "/../test_data/Bags/WaterPistol/WaterPistol_2010-11-20-00-41-11.bag",
    #scriptPath + "/../test_data/Bags/WaterPistol/WaterPistol_2010-11-22-15-40-51.bag",
    #scriptPath + "/../test_data/Bags/WaterPistol/WaterPistol_2010-11-22-15-41-09.bag",
    ( scriptPath + "/../test_data/Bags/Screwdriver/Screwdriver_Prod_2010-09-24-15-24-25.bag", False, 2 ),
    ( scriptPath + "/../test_data/Bags/Screwdriver/Screwdriver_Prod_2010-09-24-15-24-52.bag", False, 2 ),
    ( scriptPath + "/../test_data/Bags/Screwdriver/Screwdriver_Prod_2010-09-24-15-25-47.bag", False, 2 ),
    ( scriptPath + "/../test_data/Bags/Screwdriver/Screwdriver_Prod_2010-09-24-15-26-14.bag", False, 2 ),
    ( scriptPath + "/../test_data/Bags/Sellotape/Sellotape_Prod_2010-09-24-15-22-02.bag", False, 2 ),
    ( scriptPath + "/../test_data/Bags/Sellotape/Sellotape_Prod_2010-09-24-15-22-24.bag", False, 2 ),
    ( scriptPath + "/../test_data/Bags/Sellotape/Sellotape_Prod_2010-09-24-15-22-49.bag", False, 2 ),
    ( scriptPath + "/../test_data/Bags/Sellotape/Sellotape_Prod_2010-09-24-15-23-09.bag", False, 2 ),
    ( scriptPath + "/../test_data/Bags/Tissues/Tissue_Prod_2010-09-24-15-17-40.bag", False, 2 ),
    ( scriptPath + "/../test_data/Bags/Tissues/Tissue_Prod_2010-09-24-15-18-09.bag", False, 2 ),
    ( scriptPath + "/../test_data/Bags/Tissues/Tissue_Prod_2010-09-24-15-18-36.bag", False, 2 ),
    ( scriptPath + "/../test_data/Bags/Tissues/Tissue_Prod_2010-09-24-15-19-03.bag", False, 2 ),
    ( scriptPath + "/../test_data/Bags/Tissues/Tissue_Prod_2010-09-24-15-19-29.bag", False, 2 ),
    ( scriptPath + "/../test_data/Bags/Stapler/Stapler_2010-11-24-21-51-01.bag", True, 1 ),
    ( scriptPath + "/../test_data/Bags/Stapler/Stapler_2010-11-24-21-51-15.bag", True, 1 ),
    ( scriptPath + "/../test_data/Bags/Stapler/Stapler_2010-11-24-21-51-29.bag", True, 1 ),
    ( scriptPath + "/../test_data/Bags/Stapler/Stapler_2010-11-24-21-51-44.bag", True, 1 ),
    ( scriptPath + "/../test_data/Bags/Stapler/Stapler_2010-11-24-21-51-57.bag", True, 1 ) ]

for segData in segDataList:
    
    bagFilename = segData[ 0 ]
    flip = segData[ 1 ]
    frameSkip = segData[ 2 ]
    
    prefix = os.path.splitext( os.path.split( bagFilename )[ 1 ] )[ 0 ]
    print "Processing", prefix
    p = subprocess.Popen( [ testScript, "-q", "True", "-p", prefix, 
        "-f", str( flip ), "-s", str( frameSkip ), bagFilename ] )
    p.wait()
