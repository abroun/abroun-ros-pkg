//------------------------------------------------------------------------------
// File: BlobLib.cpp
// Desc: Routines for for extracting distint blobs from binary images
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#include <stdio.h>
#include "BlobLib/BlobLib.h"

//------------------------------------------------------------------------------
// Constants and definitions
const U8 EMPTY_PIXEL = 0;
const U8 FULL_PIXEL = 255;
const U8 MAX_BLOB_ID = 255;

//------------------------------------------------------------------------------
// Utility class
class ByteArray
{
    public: ByteArray( U8* pByteArrayData, 
                       S32 width, S32 height, S32 xStep, S32 yStep )
        : mpByteArrayData( pByteArrayData ),
          mWidth( width ), mHeight( width ), 
          mXStep( xStep ), mYStep( yStep ) {}
    
    // Width/Height Accessors
    public: S32 GetWidth() const { return mWidth; }
    public: S32 GetHeight() const { return mHeight; }
    
    // Byte Accessors - /should/ hopefully be inlined
    public: U8 GetByte( int x, int y ) const 
    { 
        return mpByteArrayData[ y*mYStep + x*mXStep ];
    }
    
    public: void SetByte( int x, int y, U8 value ) const 
    { 
        mpByteArrayData[ y*mYStep + x*mXStep ] = value;
    }
    
    // Members
    private: U8* mpByteArrayData;
    private: S32 mWidth;
    private: S32 mHeight;
    private: S32 mXStep;
    private: S32 mYStep;
};

//------------------------------------------------------------------------------
// Typedefs
typedef void (*MarkBlobFunc)( ByteArray&  byteArray, int x, int y, U8 blobId );

//------------------------------------------------------------------------------
// Function declarations
void MarkBlob4Connect( ByteArray& byteArray, int x, int y, U8 blobId );
void MarkBlob8Connect( ByteArray& byteArray, int x, int y, U8 blobId );

//------------------------------------------------------------------------------
int SegmentByteArray( U8* pByteArrayDataInOut, 
                       S32 width, S32 height, S32 xStep, S32 yStep,
                       eConnectedType connectedType )
{
    U8 nextBlobId = 1;
    MarkBlobFunc markBlob = NULL;
    ByteArray byteArray( pByteArrayDataInOut, width, height, xStep, yStep );
    
    // Get the blob marking routine to use
    if ( eCT_Connect4 == connectedType )
    {
        markBlob = MarkBlob4Connect;
    }
    else // eCT_Connect8 == connectedType
    {
        markBlob = MarkBlob8Connect;
    }
            
    for ( int y = 0; y < height; y++ )
    {
        for ( int x = 0; x < width; x++ )
        {
            if ( byteArray.GetByte( x, y ) == FULL_PIXEL )
            {
                // We've found an unmarked object
                if ( nextBlobId == MAX_BLOB_ID )
                {
                    fprintf( stderr, "Warning: Too many blobs in byte array to mark\n" );
                    goto FinishedSegmentation;
                }
                
                markBlob( byteArray, x, y, nextBlobId );
                nextBlobId++;
            }
        }
    }
    
FinishedSegmentation:
    // All done
    int numBlobsFound = nextBlobId - 1;
    return numBlobsFound;
}

//------------------------------------------------------------------------------
void MarkBlob4Connect( ByteArray& byteArray, int x, int y, U8 blobId )
{
    // Mark the given pixel
    byteArray.SetByte( x, y, blobId );
    
    // Check neighbouring 4 connected pixels and mark if needed
    int leftX = x - 1;
    int rightX = x + 1;
    int topY = y - 1;
    int bottomY = y + 1;
    if ( leftX >= 0 && byteArray.GetByte( leftX, y ) == FULL_PIXEL )
    {
        MarkBlob4Connect( byteArray, leftX, y, blobId );
    }
    if ( rightX < byteArray.GetWidth() && byteArray.GetByte( rightX, y ) == FULL_PIXEL )
    {
        MarkBlob4Connect( byteArray, rightX, y, blobId );
    }
    if ( topY >= 0 && byteArray.GetByte( x, topY ) == FULL_PIXEL )
    {
        MarkBlob4Connect( byteArray, x, topY, blobId );
    }
    if ( bottomY < byteArray.GetHeight() && byteArray.GetByte( x, bottomY ) == FULL_PIXEL )
    {
        MarkBlob4Connect( byteArray, x, bottomY, blobId );
    }
}

//------------------------------------------------------------------------------
void MarkBlob8Connect( ByteArray& byteArray, int x, int y, U8 blobId )
{
    // Mark the given pixel
    byteArray.SetByte( x, y, blobId );
    
    // Check neighbouring 8 connected pixels and mark if needed
    int leftX = x - 1;
    int rightX = x + 1;
    int topY = y - 1;
    int bottomY = y + 1;
    int width = byteArray.GetWidth();
    int height = byteArray.GetHeight();
    
    // Left pixels
    if ( leftX >= 0 )
    {
        if ( topY >= 0 && byteArray.GetByte( leftX, topY ) == FULL_PIXEL )
        {
            MarkBlob8Connect( byteArray, leftX, topY, blobId );
        }
        if ( byteArray.GetByte( leftX, y ) == FULL_PIXEL )
        {
            MarkBlob8Connect( byteArray, leftX, y, blobId );
        }
        if ( bottomY < height && byteArray.GetByte( leftX, bottomY ) == FULL_PIXEL )
        {
            MarkBlob8Connect( byteArray, leftX, bottomY, blobId );
        }
    }
    
    // Top and bottom pixels
    if ( topY >= 0 && byteArray.GetByte( x, topY ) == FULL_PIXEL )
    {
        MarkBlob8Connect( byteArray, x, topY, blobId );
    }
    if ( bottomY < height && byteArray.GetByte( x, bottomY ) == FULL_PIXEL )
    {
        MarkBlob8Connect( byteArray, x, bottomY, blobId );
    }
    
    // Right pixels
    if ( rightX < width )
    {
        if ( topY >= 0 && byteArray.GetByte(  rightX, topY ) == FULL_PIXEL )
        {
            MarkBlob8Connect( byteArray, rightX, topY, blobId );
        }
        if ( byteArray.GetByte( rightX, y ) == FULL_PIXEL )
        {
            MarkBlob8Connect( byteArray, rightX, y, blobId );
        }
        if ( bottomY < height && byteArray.GetByte( rightX, bottomY ) == FULL_PIXEL )
        {
            MarkBlob8Connect( byteArray, rightX, bottomY, blobId );
        }
    }
}

