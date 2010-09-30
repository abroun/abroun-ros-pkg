//------------------------------------------------------------------------------
// File: BlobLib.h
// Desc: Routines for for extracting distint blobs from binary images
//------------------------------------------------------------------------------

#ifndef BLOBLIB_H
#define BLOBLIB_H

extern "C"
{

typedef unsigned char U8;
typedef int S32;
    
enum eConnectedType
{
    eCT_Connect4,
    eCT_Connect8
};

// Takes in a binary image with empty pixels marked as 0 and full pixels
// marked as 255. Segments the image into blobs by marking each individual
// blob with its own ID.
// As bytes are used to store the pixels the maximum number of blobs that
// can be present in an image is 254. The routine will leave extra blobs
// unmarked.
// Returns the number of blobs found.
int SegmentByteArray( U8* pByteArrayDataInOut, 
                      S32 width, S32 height, S32 xStep, S32 yStep,
                      eConnectedType connectedType );
            
// Extracts a blob from a previously segmented byte array and returns it
// in its own rectangular array. The destination array needs to be big 
// enough to hold the extracted blob and so to be on the safe side it should
// probably be as big as the source image.
// Returns true if it was possible to extract the blob and false otherwise
bool ExtractBlob( const U8* pByteArrayData, 
                  S32 width, S32 height, S32 xStep, S32 yStep,
                  S32 blobIdx, 
                  U8* pBlobDataInOut, S32 blobDataSize,
                  S32* pBlobWidthOut, S32* pBlobbHeightOut );
    
} // extern "C"

#endif // BLOBLIB_H