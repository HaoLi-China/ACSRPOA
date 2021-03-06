// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#ifdef NDEBUG
#undef NDEBUG
#include <assert.h>
#define NDEBUG
#else
#include <assert.h>
#endif // NDEBUG

/// Kinect2 support is disabled by default (to not add the Kinect2 SDK dependency)
#ifndef COMPILE_WITHOUT_Kinect2
#define COMPILE_WITHOUT_Kinect2
#endif

#ifndef COMPILE_WITHOUT_CUDA
#include <cuda_runtime.h>
#endif

#if defined(__CUDACC__) && defined(__CUDA_ARCH__)
#define _CPU_AND_GPU_CODE_ __device__	// for CUDA device code
#else
#define _CPU_AND_GPU_CODE_ 
#endif

#include "../Utils/ITMMath.h" 

//////////////////////////////////////////////////////////////////////////
// Voxel Hashing definition and helper functions
//////////////////////////////////////////////////////////////////////////

#define SDF_BLOCK_SIZE 8				// SDF block size
#define SDF_BLOCK_SIZE3 512				// SDF_BLOCK_SIZE3 = SDF_BLOCK_SIZE * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE
#define SDF_LOCAL_BLOCK_NUM 0x40000		// Number of locally stored blocks, currently 2^18

#define SDF_GLOBAL_BLOCK_NUM 0x120000	// Number of globally stored blocks: SDF_BUCKET_NUM * SDF_ENTRY_NUM_PER_BUCKET + SDF_EXCESS_LIST_SIZE
#define SDF_TRANSFER_BLOCK_NUM 0x1000	// Maximum number of blocks transfered in one swap operation

#define SDF_ENTRY_NUM_PER_BUCKET 1		// Number of entries in each Hash Bucket
#define SDF_BUCKET_NUM 0x100000			// Number of Hash Bucket, should be 2^n and bigger than SDF_LOCAL_BLOCK_NUM, SDF_HASH_MASK = SDF_BUCKET_NUM - 1
#define SDF_HASH_MASK 0xfffff			// Used for get hashing value of the bucket index,  SDF_HASH_MASK = SDF_BUCKET_NUM - 1
#define SDF_EXCESS_LIST_SIZE 0x20000	// 0x20000 Size of excess list, used to handle collisions. Also max offset (unsigned short) value.

//////////////////////////////////////////////////////////////////////////
// Voxel Hashing data structures
//////////////////////////////////////////////////////////////////////////

/** \brief
    A single entry in the hash table.
*/
struct ITMHashEntry
{
	/** Position of the corner of the 8x8x8 volume, that identifies the entry. */
	Vector3s pos;
	/** Offset in the excess list. */
	int offset;
	/** Pointer to the voxel block array.
	    - >= 0 identifies an actual allocated entry in the voxel block array
	    - -1 identifies an entry that has been removed (swapped out)
	    - <-1 identifies an unallocated block
	*/
	int ptr;
};

struct ITMHashCacheState
{
	uchar cacheFromHost;	// 0 - don't do anything, 1 - should cache from host, 2 - has cached from host
};

#include "../Objects/ITMVoxelBlockHash.h"
#include "../Objects/ITMPlainVoxelArray.h"

/** \brief
    Stores the information of a single voxel in the volume
*/
struct ITMVoxel_f_rgb
{
	_CPU_AND_GPU_CODE_ static float SDF_initialValue() { return 1.0f; }
	_CPU_AND_GPU_CODE_ static float SDF_valueToFloat(float x) { return x; }
	_CPU_AND_GPU_CODE_ static float SDF_floatToValue(float x) { return x; }

	static const bool hasColorInformation = true;

	/** Value of the truncated signed distance transformation. */
	float sdf;
	/** Number of fused observations that make up @p sdf. */
	uchar w_depth;
	/** RGB colour information stored for this voxel. */
	Vector3u clr;
	/** Number of observations that made up @p clr. */
	uchar w_color;

	_CPU_AND_GPU_CODE_ ITMVoxel_f_rgb()
	{
		sdf = SDF_initialValue();
		w_depth = 0;
		clr = (uchar)0;
		w_color = 0;
	}
};

/** \brief
    Stores the information of a single voxel in the volume
*/
struct ITMVoxel_s_rgb
{
	_CPU_AND_GPU_CODE_ static short SDF_initialValue() { return 32767; }
	_CPU_AND_GPU_CODE_ static float SDF_valueToFloat(float x) { return (float)(x) / 32767.0f; }
	_CPU_AND_GPU_CODE_ static short SDF_floatToValue(float x) { return (short)((x) * 32767.0f); }

	static const bool hasColorInformation = true;

	/** Value of the truncated signed distance transformation. */
	short sdf;
	/** Number of fused observations that make up @p sdf. */
	uchar w_depth;
	/** Padding that may or may not improve performance on certain GPUs */
	//uchar pad;
	/** RGB colour information stored for this voxel. */
	Vector3u clr;
	/** Number of observations that made up @p clr. */
	uchar w_color;

	_CPU_AND_GPU_CODE_ ITMVoxel_s_rgb()
	{
		sdf = SDF_initialValue();
		w_depth = 0;
		clr = (uchar)0;
		w_color = 0;
	}
};

struct ITMVoxel_s
{
	_CPU_AND_GPU_CODE_ static short SDF_initialValue() { return 32767; }
	_CPU_AND_GPU_CODE_ static float SDF_valueToFloat(float x) { return (float)(x) / 32767.0f; }
	_CPU_AND_GPU_CODE_ static short SDF_floatToValue(float x) { return (short)((x) * 32767.0f); }

  //_CPU_AND_GPU_CODE_ static short NORMAL_initialValue() { return 0; }//hao modified it 
  //_CPU_AND_GPU_CODE_ static float NORMAL_valueToFloat(float x) { return (float)(x) / 32767.0f; }//hao modified it 
  //_CPU_AND_GPU_CODE_ static short NORMAL_floatToValue(float x) { return (short)((x) * 32767.0f); }//hao modified it 

	static const bool hasColorInformation = false;

  ushort id;//hao modified it 
  uchar r;//hao modified it 
  uchar g;//hao modified it 
  uchar b;//hao modified it 
  //short normal_x;//hao modified it 
  //short normal_y;//hao modified it 
  //short normal_z;//hao modified it 

	/** Value of the truncated signed distance transformation. */
	short sdf;
	/** Number of fused observations that make up @p sdf. */
	uchar w_depth;
	/** Padding that may or may not improve performance on certain GPUs */
	//uchar pad;

  //hao modified it 
	_CPU_AND_GPU_CODE_ ITMVoxel_s()
	{
		sdf = SDF_initialValue();
		w_depth = 0;
    id = 0;
    r = 255;
    g = 255;
    b = 255;
    //normal_x = NORMAL_initialValue();
    //normal_y = NORMAL_initialValue();
    //normal_z = NORMAL_initialValue();
	}
};

struct ITMVoxel_f
{
	_CPU_AND_GPU_CODE_ static float SDF_initialValue() { return 1.0f; }
	_CPU_AND_GPU_CODE_ static float SDF_valueToFloat(float x) { return x; }
	_CPU_AND_GPU_CODE_ static float SDF_floatToValue(float x) { return x; }

	static const bool hasColorInformation = false;

	/** Value of the truncated signed distance transformation. */
	float sdf;
	/** Number of fused observations that make up @p sdf. */
	uchar w_depth;
	/** Padding that may or may not improve performance on certain GPUs */
	//uchar pad;

	_CPU_AND_GPU_CODE_ ITMVoxel_f()
	{
		sdf = SDF_initialValue();
		w_depth = 0;
	}
};

/** This chooses the information stored at each voxel. At the moment, valid
    options are ITMVoxel_s, ITMVoxel_f, ITMVoxel_s_rgb and ITMVoxel_f_rgb 
*/
typedef ITMVoxel_s ITMVoxel;

/** This chooses the way the voxels are addressed and indexed. At the moment,
    valid options are ITMVoxelBlockHash and ITMPlainVoxelArray.
*/
typedef ITMLib::Objects::ITMVoxelBlockHash ITMVoxelIndex;
//typedef ITMLib::Objects::ITMPlainVoxelArray ITMVoxelIndex;

//////////////////////////////////////////////////////////////////////////
// Do not change below this point
//////////////////////////////////////////////////////////////////////////
#ifndef ITMFloatImage
#define ITMFloatImage ITMImage<float>
#endif

#ifndef ITMFloat2Image
#define ITMFloat2Image ITMImage<Vector2f>
#endif

#ifndef ITMFloat4Image
#define ITMFloat4Image ITMImage<Vector4f>
#endif

#ifndef ITMShortImage
#define ITMShortImage ITMImage<short>
#endif

#ifndef ITMShort3Image
#define ITMShort3Image ITMImage<Vector3s>
#endif

#ifndef ITMShort4Image
#define ITMShort4Image ITMImage<Vector4s>
#endif

#ifndef ITMUShortImage
#define ITMUShortImage ITMImage<ushort>
#endif

#ifndef ITMUIntImage
#define ITMUIntImage ITMImage<uint>
#endif

#ifndef ITMIntImage
#define ITMIntImage ITMImage<int>
#endif

#ifndef ITMUCharImage
#define ITMUCharImage ITMImage<uchar>
#endif

#ifndef ITMUChar4Image
#define ITMUChar4Image ITMImage<Vector4u>
#endif

#ifndef ITMBoolImage
#define ITMBoolImage ITMImage<bool>
#endif

//debug
#ifndef DEBUGBREAK
#define DEBUGBREAK \
{ \
	int ryifrklaeybfcklarybckyar=0; \
	ryifrklaeybfcklarybckyar++; \
}
#endif

