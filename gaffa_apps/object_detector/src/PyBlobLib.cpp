//------------------------------------------------------------------------------
// Exposes BlobLib as a Python extension
//------------------------------------------------------------------------------

#include "Python.h"
#include "numpy/arrayobject.h"
#include <math.h>

//------------------------------------------------------------------------------
static PyObject* labelBlobs( PyObject* pSelf, PyObject* args )
{
    PyArrayObject* pImgArray;
    
    // Get the input array
    if ( !PyArg_ParseTuple( args, "O", &pImgArray ) )  
    {
        return NULL;
    }
    if ( NULL == pImgArray )
    {
        return NULL;
    }

    // Check that it's the correct type
    if ( NPY_UBYTE != PyArray_TYPE( pImgArray ) || 2 != pImgArray->nd )  
    {
        PyErr_SetString( PyExc_ValueError,
            "Array must be of type uint8 and 2 dimensional (n x m)." );
        return NULL;
    }
    
    // Create a copy to output
    PyObject* pOutputArray = PyArray_NewCopy( pImgArray, NPY_CORDER );
    
    // Process the data
    unsigned char* pData = (unsigned char*)PyArray_DATA( pOutputArray );
    int yStride = PyArray_STRIDE( pOutputArray, 0 );
    int xStride = PyArray_STRIDE( pOutputArray, 1 );
    int height = PyArray_DIM( pOutputArray, 0 );
    int width = PyArray_DIM( pOutputArray, 1 );
    int extraYStep = yStride - width*xStride;
      
    for ( int y = 0; y < height/2; y++ )
    {
        for ( int x = 0; x < width; x++ )
        {
            *pData = 255;
            pData += xStride;
        }
        pData += extraYStep;
    }

    // Return the processed data
    return pOutputArray;
}

//------------------------------------------------------------------------------
// Methods table
static PyMethodDef PyBlobLibMethods[] = {
    { "labelBlobs", labelBlobs, METH_VARARGS, "Labels blobs in the binary image" },
    { NULL, NULL, 0, NULL }     // Sentinel - marks the end of this structure
};

//------------------------------------------------------------------------------
PyMODINIT_FUNC initPyBlobLib()  
{
    Py_InitModule( "PyBlobLib", PyBlobLibMethods );
    import_array();  // Must be present for NumPy.  Called first after above line.
}


