
import rosbag
import numpy as np
import cv
import Utils

import math
import random
import sys
sys.path.append( "../" )
from OpticalFlowFilter import OpticalFlowFilter

#-------------------------------------------------------------------------------
class Distractor:
    
    #---------------------------------------------------------------------------
    def __init__( self, radius, startPos, endPos, frequency ):
        
        self.radius = radius
        self.startPos = startPos
        self.endPos = endPos
        self.frequency = frequency

#-------------------------------------------------------------------------------
class InputSequence:
    '''A sequence of images and a sequence of servo angles read from a bag file''' 
    
    #---------------------------------------------------------------------------
    def __init__( self, bagFilename ):
        
        self.servoAngleTimes = []
        self.servoAngleData = []
        self.cameraImages = []
        self.imageTimes = []
        
        # Optical flow variables
        self.opticalFlowArraysX = None
        self.opticalFlowArraysY = None
        self.opticalFlowMethod = None
        self.opticalFlowFilter = None
        self.opticalFlowCalculated = False
        
        # Extract messages from the bag
        startTime = None
        lastAngleTime = -1.0
        lastCameraTime = -1.0
        
        bag = rosbag.Bag( bagFilename )
        for topic, msg, t in bag.read_messages():
            if startTime == None:
                startTime = t
                
            bagTime = t - startTime
            bagTimeSec = bagTime.to_sec()
                
            if msg._type == "arm_driver_msgs/SetServoAngles" \
                and bagTimeSec - lastAngleTime > 0.01:
                
                lastAngleTime = bagTimeSec
                self.servoAngleTimes.append( bagTimeSec )
                
                servoAngle = msg.servoAngles[ 0 ].angle
                self.servoAngleData.append( servoAngle )
                
            elif msg._type == "sensor_msgs/Image" \
                and bagTimeSec - lastCameraTime > 0.01:

                lastCameraTime = bagTimeSec
                self.imageTimes.append( bagTimeSec )
                
                # Convert the image to a numpy array
                if msg.encoding == "rgb8" or msg.encoding == "bgr8":
            
                    image = np.fromstring( msg.data, dtype=np.uint8 )
                    image.shape = ( msg.height, msg.width, 3 )
                    self.cameraImages.append( image )
            
                else:
                    rospy.logerr( "Unhandled image encoding - " + rosImage.encoding )
        
        bag.close()   
        del bag     
        self.servoAngleData = Utils.normaliseSequence( self.servoAngleData )
       
    #---------------------------------------------------------------------------
    def addDistractorObjects( self, distractors, randomSeed = None ):
        
        if randomSeed != None:
            random.seed( randomSeed )
        
        for distractor in distractors:
            
            distractorRadius = int( distractor.radius )
            distractorDim = distractorRadius*2 + 1
            i = np.indices( ( distractorDim, distractorDim ) ) - distractorRadius
            distractorIndices = np.where( i[0]*i[0] + i[1]*i[1] <= distractorRadius*distractorRadius )
            
            startPos = ( int( distractor.startPos[ 0 ] ), int( distractor.startPos[ 1 ] ) )
            endPos = ( int( distractor.endPos[ 0 ] ), int( distractor.endPos[ 1 ] ) )
            frequency = distractor.frequency
                
            distractorData = np.array( 
                np.random.rand( distractorDim, distractorDim )*255, dtype=np.int8 )
            
            # Add the distractor to the image
            for imageIdx, image in enumerate( self.cameraImages ):
                
                time = self.imageTimes[ imageIdx ]
                offset = math.cos( time*frequency*2.0*math.pi )
                offset = (1.0 - offset) / 2.0
                
                posX = int( startPos[ 0 ] + offset*(endPos[ 0 ] - startPos[ 0 ]) )
                posY = int( startPos[ 1 ] + offset*(endPos[ 1 ] - startPos[ 1 ]) )
                
                imageTargetIndices = ( 
                    distractorIndices[ 0 ] - distractorRadius + posX,
                    distractorIndices[ 1 ] - distractorRadius + posY )
                
                image[ imageTargetIndices[ 0 ], imageTargetIndices[ 1 ], 0 ] = distractorData[ distractorIndices ]
                image[ imageTargetIndices[ 0 ], imageTargetIndices[ 1 ], 1 ] = distractorData[ distractorIndices ]
                image[ imageTargetIndices[ 0 ], imageTargetIndices[ 1 ], 2 ] = distractorData[ distractorIndices ]

    #---------------------------------------------------------------------------
    def calculateOpticalFlow( self,
        opticalFlowBlockWidth, opticalFlowBlockHeight, 
        opticalFlowRangeWidth, opticalFlowRangeHeight, opticalFlowMethod ):
          
        numImages = len( self.cameraImages )  
        if numImages <= 0:
            return False  # Nothing to calculate optical flow with
            
        self.opticalFlowMethod = opticalFlowMethod
        self.opticalFlowFilter = OpticalFlowFilter(
            opticalFlowBlockWidth, opticalFlowBlockHeight, 
            opticalFlowRangeWidth, opticalFlowRangeHeight )
        
        # Create arrays to hold the optical flow
        opticalFlowWidth = self.opticalFlowFilter.calcOpticalFlowWidth( self.cameraImages[ 0 ].shape[ 1 ] )
        opticalFlowHeight = self.opticalFlowFilter.calcOpticalFlowHeight( self.cameraImages[ 0 ].shape[ 0 ] )
        opticalFlowArrayShape = ( opticalFlowHeight, opticalFlowWidth, numImages )
        self.opticalFlowArraysX = np.ndarray( shape=opticalFlowArrayShape, dtype=np.float32 )
        self.opticalFlowArraysY = np.ndarray( shape=opticalFlowArrayShape, dtype=np.float32 )

        for imageIdx, image in enumerate( self.cameraImages ):
            opticalFlowArrayX, opticalFlowArrayY = self.processCameraImage( image )
            
            self.opticalFlowArraysX[ :, :, imageIdx ] = np.array( opticalFlowArrayX )
            self.opticalFlowArraysY[ :, :, imageIdx ] = np.array( opticalFlowArrayY )
            
        self.opticalFlowCalculated = True
        return True
            
    #---------------------------------------------------------------------------
    def processCameraImage( self, image ):
        
        opticalFlowArrayX = None
        opticalFlowArrayY = None
        
        # Create an OpenCV image to process the data
        curImageGray = cv.CreateImage( ( image.shape[ 1 ], image.shape[ 0 ] ), cv.IPL_DEPTH_8U, 1 )
        cv.CvtColor( cv.fromarray( image ), curImageGray, cv.CV_RGB2GRAY )
        
        # Look for optical flow between this image and the last one
        opticalFlowArrayX, opticalFlowArrayY = \
            self.opticalFlowFilter.calcOpticalFlow( curImageGray, self.opticalFlowMethod )
            
        return ( opticalFlowArrayX, opticalFlowArrayY )