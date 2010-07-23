#-------------------------------------------------------------------------------
import math
from RoboticArm import DHFrame

#-------------------------------------------------------------------------------
GROUND_TO_CENTRE_OF_SHOULDER_HEIGHT = 6.193
TOP_ARM_LENGTH = 15.4
FORE_ARM_LENGTH = 15.4
#DISTANCE_TO_CENTRE_OF_ROTATE = 0.15
#END_EFFECTOR_LENGTH = 0.15
HAND_GRIP_LENGTH = 10.16

# Define arm as a set of DHFrames
ARM_DH_DISTAL = [
    DHFrame( d=GROUND_TO_CENTRE_OF_SHOULDER_HEIGHT, theta=math.pi/2.0, a=0.0, alpha=math.pi/2.0 ),  # Base
    DHFrame( d=0.0, theta=0.0, a=TOP_ARM_LENGTH, alpha=0.0 ),               # Shoulder
    DHFrame( d=0.0, theta=0.0, a=FORE_ARM_LENGTH, alpha=0.0 ),              # Elbow
    DHFrame( d=0.0, theta=math.pi/2.0, a=0.0, alpha=math.pi/2.0 ), # Wrist Bend
    DHFrame( d=HAND_GRIP_LENGTH, theta=0.0, a=0.0, alpha=0.0 ) ] # Wrist Rotate
    
ARM_DH_PROXIMAL = [
    DHFrame( a=0.0, alpha=0.0, d=GROUND_TO_CENTRE_OF_SHOULDER_HEIGHT, theta=0.0 ),  # Base
    DHFrame( a=0.0, alpha=math.pi/2.0, d=0.0, theta=0.0 ),  # Shoulder
    DHFrame( a=TOP_ARM_LENGTH, alpha=0.0, d=0.0, theta=0.0,  ), # Elbow
    DHFrame( a=FORE_ARM_LENGTH, alpha=0.0, d=0.0, theta=0.0 ), # Wrist Bend            
    DHFrame( a=0.0, alpha=math.pi/2.0, d=HAND_GRIP_LENGTH, theta=0.0 ) ] # Wrist Rotate
