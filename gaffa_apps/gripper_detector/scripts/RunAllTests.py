#! /usr/bin/env python

import os.path
import subprocess

scriptPath = os.path.dirname( __file__ )

testScript = scriptPath + "/../src/gripper_detector/OpticalFlowTestScript.py"

bagFilename = scriptPath + "/../test_data/06_Wave.bag"
markerFilename = scriptPath + "/../config/OnTablePosGripper.yaml"

p = subprocess.Popen( [ testScript, bagFilename, markerFilename ] )
p.wait()