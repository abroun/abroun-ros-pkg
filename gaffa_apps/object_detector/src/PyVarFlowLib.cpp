//------------------------------------------------------------------------------
// Exposes VarFlowLib as a Python extension
//------------------------------------------------------------------------------

#include "Python.h"
#include "numpy/arrayobject.h"
#include <math.h>
#include "VarFlowLib/VarFlow.h"

//------------------------------------------------------------------------------
static PyObject* createMotionMask( PyObject* pSelf, PyObject* args )
{
    PyArrayObject* pFirstImgArray;
    PyArrayObject* pSecondImgArray;
    
    // Get the input arrays
    if ( !PyArg_ParseTuple( args, "OO", &pFirstImgArray, &pSecondImgArray ) )  
    {
        return NULL;
    }
    if ( NULL == pFirstImgArray )
    {
        return NULL;
    }
    if ( NULL == pSecondImgArray )
    {
        return NULL;
    }

    // Check that they're the correct type
    if ( NPY_UBYTE != PyArray_TYPE( pFirstImgArray ) || 2 != pFirstImgArray->nd )  
    {
        PyErr_SetString( PyExc_ValueError,
            "pFirstImgArray invalid - Array must be of type uint8 and 2 dimensional (n x m)." );
        return NULL;
    }
    if ( NPY_UBYTE != PyArray_TYPE( pSecondImgArray ) || 2 != pSecondImgArray->nd )  
    {
        PyErr_SetString( PyExc_ValueError,
            "pSecondImgArray invalid - Array must be of type uint8 and 2 dimensional (n x m)." );
        return NULL;
    }
    
    // Process the data
    int height = PyArray_DIM( pFirstImgArray, 0 );
    int width = PyArray_DIM( pFirstImgArray, 1 );
    
    //We will start at level 0 (full size image) and go down to level 4 (coarse image 16 times smaller than original)
    //Experiment with these values to see how they affect the flow field as well as calculation time
    int max_level = 4;
    int start_level = 0;

    //Two pre and post smoothing steps, should be greater than zero
    int n1 = 2;
    int n2 = 2;

    //Smoothing and regularization parameters, experiment but keep them above zero
    float rho = 2.8;
    float alpha = 1400;
    float sigma = 1.5;
    
    // Set up VarFlow class
    VarFlow OpticalFlow( width, height, max_level, start_level, n1, n2, rho, alpha, sigma );
    
    IplImage* imgA = cvCreateImageHeader(cvSize(width, height), 8, 1);
    IplImage* imgB = cvCreateImageHeader(cvSize(width, height), 8, 1);
    
    cvSetData( imgA, PyArray_DATA( pFirstImgArray ), PyArray_STRIDE( pFirstImgArray, 0 ) );
    cvSetData( imgB, PyArray_DATA( pSecondImgArray ), PyArray_STRIDE( pSecondImgArray, 0 ) );
    
    IplImage* imgU = cvCreateImage(cvSize(width, height), IPL_DEPTH_32F, 1);
    IplImage* imgV = cvCreateImage(cvSize(width, height), IPL_DEPTH_32F, 1);
    
    // Calculate optical flow
    OpticalFlow.CalcFlow(imgA, imgB, imgU, imgV, 0);
    
    // Create the output mask
    int dims[] = { height, width };
    PyObject* pOutputArray = PyArray_SimpleNew( 2, dims, NPY_UBYTE );
    //PyObject* pOutputArray = PyArray_NewCopy( pFirstImgArray, NPY_CORDER );
    
    // Set mask to thresholded version of optical flow magnitude
    const float THRESHOLD = 1.25f;
    
    uchar* pOutputData = (uchar*)PyArray_DATA( pOutputArray );
    int outputStride = PyArray_STRIDE( pOutputArray, 0 );
     
    static int i = -1;
    i = (i + 1)%width;
    
    for ( int y = 0; y < height; y++ ) 
    {
        for( int x = 0; x < width; x++ )
        {            
            float deltaX = *((float*)(imgU->imageData + y*imgU->widthStep)+x);
            float deltaY = -(*((float*)(imgV->imageData + y*imgV->widthStep)+x));
            uchar* pPixel = (uchar*)(pOutputData + y*outputStride + x);

            float hyp = sqrt(deltaX*deltaX + deltaY*deltaY);
            *pPixel = ( hyp > THRESHOLD ? 255 : 0 );
        }
    }
    
    cvReleaseImageHeader(&imgA);
    cvReleaseImageHeader(&imgB);
    cvReleaseImage(&imgU);
    cvReleaseImage(&imgV);
    
    // Return the processed data
    return Py_BuildValue( "O", pOutputArray );
}

//------------------------------------------------------------------------------
// Methods table
static PyMethodDef PyVarFlowLibMethods[] = {
    { "createMotionMask", createMotionMask, METH_VARARGS, "Labels blobs in the binary image" },
    { NULL, NULL, 0, NULL }     // Sentinel - marks the end of this structure
};

//------------------------------------------------------------------------------
PyMODINIT_FUNC initPyVarFlowLib()  
{
    Py_InitModule( "PyVarFlowLib", PyVarFlowLibMethods );
    import_array();  // Must be present for NumPy.  Called first after above line.
}


