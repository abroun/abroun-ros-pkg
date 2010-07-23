#! /usr/bin/python

import numpy as np
import math

#-------------------------------------------------------------------------------
def CreateVector( x, y, z, w = 1.0 ):
    return np.matrix( [[ x ],
                       [ y ],
                       [ z ],
                       [ w ]] )

#-------------------------------------------------------------------------------
def CreateIdentity():
    return np.matrix( np.identity( 4 ) )

#-------------------------------------------------------------------------------
def CreateTranslation( x, y, z ):
    result = np.matrix( np.identity( 4 ) )
    result[ 0, 3 ] = x
    result[ 1, 3 ] = y
    result[ 2, 3 ] = z

    return result

#-------------------------------------------------------------------------------
def CreateRotationX( angle ):
    sinAngle = math.sin( angle )
    cosAngle = math.cos( angle )

    result = np.matrix( np.identity( 4 ) )
    result[ 1, 1 ] = cosAngle
    result[ 1, 2 ] = -sinAngle
    result[ 2, 1 ] = sinAngle
    result[ 2, 2 ] = cosAngle

    return result

#-------------------------------------------------------------------------------
def CreateRotationY( angle ):
    sinAngle = math.sin( angle )
    cosAngle = math.cos( angle )

    result = np.matrix( np.identity( 4 ) )
    result[ 0, 0 ] = cosAngle
    result[ 0, 2 ] = sinAngle
    result[ 2, 0 ] = -sinAngle
    result[ 2, 2 ] = cosAngle

    return result

#-------------------------------------------------------------------------------
def CreateRotationZ( angle ):
    sinAngle = math.sin( angle )
    cosAngle = math.cos( angle )

    result = np.matrix( np.identity( 4 ) )
    result[ 0, 0 ] = cosAngle
    result[ 0, 1 ] = -sinAngle
    result[ 1, 0 ] = sinAngle
    result[ 1, 1 ] = cosAngle

    return result

#-------------------------------------------------------------------------------
def CreateProximalDenavitHartenburgMatrix( prevA, prevAlpha, d, theta,  ):
    sinTheta = math.sin( theta )
    cosTheta = math.cos( theta )
    sinPrevAlpha = math.sin( prevAlpha )
    cosPrevAlpha = math.cos( prevAlpha )

    result = np.matrix( np.identity( 4 ) )
    result[ 0, 0 ] = cosTheta
    result[ 0, 1 ] = -sinTheta
    #result[ 0, 2 ] = 0.0
    result[ 0, 3 ] = prevA
    result[ 1, 0 ] = sinTheta*cosPrevAlpha
    result[ 1, 1 ] = cosTheta*cosPrevAlpha
    result[ 1, 2 ] = -sinPrevAlpha
    result[ 1, 3 ] = -sinPrevAlpha*d
    result[ 2, 0 ] = sinTheta*sinPrevAlpha
    result[ 2, 1 ] = cosTheta*sinPrevAlpha
    result[ 2, 2 ] = cosPrevAlpha
    result[ 2, 3 ] = cosPrevAlpha*d

    return result

#-------------------------------------------------------------------------------
def CreateDistalDenavitHartenburgMatrix( d, theta, a, alpha ):
    sinTheta = math.sin( theta )
    cosTheta = math.cos( theta )
    sinAlpha = math.sin( alpha )
    cosAlpha = math.cos( alpha )

    result = np.matrix( np.identity( 4 ) )
    result[ 0, 0 ] = cosTheta
    result[ 0, 1 ] = -sinTheta*cosAlpha
    result[ 0, 2 ] = sinTheta*sinAlpha
    result[ 0, 3 ] = a*cosTheta
    result[ 1, 0 ] = sinTheta
    result[ 1, 1 ] = cosTheta*cosAlpha
    result[ 1, 2 ] = -cosTheta*sinAlpha
    result[ 1, 3 ] = a*sinTheta
    result[ 2, 1 ] = sinAlpha
    result[ 2, 2 ] = cosAlpha
    result[ 2, 3 ] = d

    return result