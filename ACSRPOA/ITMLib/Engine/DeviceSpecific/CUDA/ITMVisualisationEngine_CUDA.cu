// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMVisualisationEngine_CUDA.h"
#include "ITMCUDAUtils.h"
#include "../../DeviceAgnostic/ITMSceneReconstructionEngine.h"
#include "../../DeviceAgnostic/ITMVisualisationEngine.h"

using namespace ITMLib::Engine;

inline dim3 getGridSize(dim3 taskSize, dim3 blockSize)
{
	return dim3((taskSize.x + blockSize.x - 1) / blockSize.x, (taskSize.y + blockSize.y - 1) / blockSize.y, (taskSize.z + blockSize.z - 1) / blockSize.z);
}

inline dim3 getGridSize(Vector2i taskSize, dim3 blockSize) { return getGridSize(dim3(taskSize.x, taskSize.y), blockSize); }

// declaration of device functions

__global__ void buildVisibleList_device(const ITMHashEntry *hashTable, /*ITMHashCacheState *cacheStates, bool useSwapping,*/ int noTotalEntries,
	int *liveEntryIDs, int *noLiveEntries, uchar *entriesVisibleType, Matrix4f M, Vector4f projParams, Vector2i imgSize, float voxelSize);

template<typename T>
__global__ void memsetKernel_device(T *devPtr, const T val, size_t nwords);

__global__ void projectAndSplitBlocks_device(const ITMHashEntry *hashEntries, const int *liveEntryIDs, int noLiveEntries,
	const Matrix4f pose_M, const Vector4f intrinsics, const Vector2i imgSize, float voxelSize, RenderingBlock *renderingBlocks,
	uint *noTotalBlocks);

__device__ static inline void atomicMin(float* address, float val);
__device__ static inline void atomicMax(float* address, float val);
__global__ void fillBlocks_device(const uint *noTotalBlocks, const RenderingBlock *renderingBlocks,
	Vector2i imgSize, Vector2f *minmaxData);

//hao modified it
template<class TVoxel, class TIndex, class TRaycastRenderer>
__global__ void genericRaycastAndRender_device(TRaycastRenderer renderer,
	 TVoxel *voxelData, const typename TIndex::IndexData *voxelIndex, Vector2i imgSize, Matrix4f invM, Vector4f projParams,
	float oneOverVoxelSize, const Vector2f *minmaxdata, float mu, Vector3f lightSource, Vector3f* colors, ushort* objectId);

//hao modified it
template<class TVoxel, class TIndex, class TRaycastRenderer>
__global__ void new_genericRaycastAndRender_device(TRaycastRenderer renderer,
	 TVoxel *voxelData, const typename TIndex::IndexData *voxelIndex, Vector2i imgSize, Matrix4f invM, Vector4f projParams,
	float oneOverVoxelSize, const Vector2f *minmaxdata, float mu, Vector3f lightSource, Vector3f *colors, ushort *objectIds, bool flag);

//hao modified it
template<class TVoxel, class TIndex>
__global__ void getAllPoints_device(const TVoxel *voxelData, const typename TIndex::IndexData *voxelIndex, float mu, Vector3f *points, Vector3f *normals, Vector3f *colors, ushort *objectId, int &num);

//hao modified it
template<class TVoxel, class TIndex>
__global__ void computePointNormal_device(const TVoxel *voxelData, Vector3f pt, Vector3f *normal);

//hao modified it
template<class TVoxel, class TIndex, class TRaycastRenderer>
__global__ void getRaycastImage_device(TRaycastRenderer renderer,
	TVoxel *voxelData, const typename TIndex::IndexData *voxelIndex, Vector2i imgSize, Matrix4f invM, Vector4f projParams, 
	float oneOverVoxelSize, const Vector2f *minmaxdata, float mu, Vector3f lightSource, Vector3f *points, Vector3f *normals, 
  Vector3f *colors, ushort *objectIds, Vector3f *table_sum_normal);

//hao modified it
__global__ void genIdMaps1_device(Vector2i imgSize, Vector3f *points, Vector3f *normals, Vector3f *colors, ushort *objectIds, Vector3f* table_avg_normal);

//hao modified it
__global__ void genIdMaps2_device(Vector2i imgSize, float x0, float x1, float y0, float y1, float z, Vector3f *points, Vector3f *normals, Vector3f *colors, ushort *objectIds);

//hao modified it
__global__ void copute_table_avg_normal_device(Vector3f *table_sum_normal, int length, Vector3f* table_avg_normal);

// class implementation

template<class TVoxel>
ITMVisualisationEngine_CUDA<TVoxel,ITMVoxelBlockHash>::State::State(const Vector2i & imgSize)
 : ITMVisualisationState(imgSize, true)
{
	static const int noTotalEntries = ITMVoxelBlockHash::noVoxelBlocks;
	ITMSafeCall(cudaMalloc((void**)&entriesVisibleType, sizeof(uchar) * noTotalEntries));
	ITMSafeCall(cudaMalloc((void**)&visibleEntryIDs, sizeof(int) * SDF_LOCAL_BLOCK_NUM));
	ITMSafeCall(cudaMalloc((void**)&visibleEntriesNum_ptr, sizeof(int)));
}

template<class TVoxel>
ITMVisualisationEngine_CUDA<TVoxel,ITMVoxelBlockHash>::State::~State(void)
{
	ITMSafeCall(cudaFree(entriesVisibleType));
	ITMSafeCall(cudaFree(visibleEntryIDs));
	ITMSafeCall(cudaFree(visibleEntriesNum_ptr));
}

template<class TVoxel, class TIndex>
ITMVisualisationEngine_CUDA<TVoxel,TIndex>::ITMVisualisationEngine_CUDA(void)
{
	ITMSafeCall(cudaMalloc((void**)&noTotalPoints_device, sizeof(uint)));
}

template<class TVoxel, class TIndex>
ITMVisualisationEngine_CUDA<TVoxel,TIndex>::~ITMVisualisationEngine_CUDA(void)
{
	ITMSafeCall(cudaFree(noTotalPoints_device));
}

template<class TVoxel>
ITMVisualisationEngine_CUDA<TVoxel,ITMVoxelBlockHash>::ITMVisualisationEngine_CUDA(void)
{
	ITMSafeCall(cudaMalloc((void**)&renderingBlockList_device, sizeof(RenderingBlock) * MAX_RENDERING_BLOCKS));
	ITMSafeCall(cudaMalloc((void**)&noTotalBlocks_device, sizeof(uint)));
	ITMSafeCall(cudaMalloc((void**)&noTotalPoints_device, sizeof(uint)));
}

template<class TVoxel>
ITMVisualisationEngine_CUDA<TVoxel,ITMVoxelBlockHash>::~ITMVisualisationEngine_CUDA(void)
{
	ITMSafeCall(cudaFree(noTotalPoints_device));
	ITMSafeCall(cudaFree(noTotalBlocks_device));
	ITMSafeCall(cudaFree(renderingBlockList_device));
}

template<class TVoxel, class TIndex>
void ITMVisualisationEngine_CUDA<TVoxel,TIndex>::FindVisibleBlocks(const ITMScene<TVoxel,TIndex> *scene, const ITMPose *pose, const ITMIntrinsics *intrinsics, ITMVisualisationState *_state)
{
}

template<class TVoxel>
void ITMVisualisationEngine_CUDA<TVoxel,ITMVoxelBlockHash>::FindVisibleBlocks(const ITMScene<TVoxel,ITMVoxelBlockHash> *scene, const ITMPose *pose, const ITMIntrinsics *intrinsics, ITMVisualisationState *_state)
{
	State *state = (State*)_state;
	const ITMHashEntry *hashTable = scene->index.GetEntries();
	int noTotalEntries = scene->index.noVoxelBlocks;
	float voxelSize = scene->sceneParams->voxelSize;
	Vector2i imgSize = state->minmaxImage->noDims;

	Matrix4f M = pose->M;
	Vector4f projParams = intrinsics->projectionParamsSimple.all;

	ITMSafeCall(cudaMemset(state->visibleEntriesNum_ptr, 0, sizeof(int)));

	dim3 cudaBlockSizeAL(256, 1);
	dim3 gridSizeAL((int)ceil((float)noTotalEntries / (float)cudaBlockSizeAL.x));
	buildVisibleList_device << <gridSizeAL, cudaBlockSizeAL >> >(hashTable, /*cacheStates, scene->useSwapping,*/ noTotalEntries, state->visibleEntryIDs,
		state->visibleEntriesNum_ptr, state->entriesVisibleType, M, projParams, imgSize, voxelSize);

/*	if (scene->useSwapping)
        {
                reAllocateSwappedOutVoxelBlocks_device << <gridSizeAL, cudaBlockSizeAL >> >(voxelAllocationList, hashTable, noTotalEntries,
                        noAllocatedVoxelEntries_device, entriesVisibleType);
        }*/

        ITMSafeCall(cudaMemcpy(&state->visibleEntriesNum, state->visibleEntriesNum_ptr, sizeof(int), cudaMemcpyDeviceToHost));
}

template<class TVoxel, class TIndex>
void ITMVisualisationEngine_CUDA<TVoxel,TIndex>::CreateExpectedDepths(const ITMScene<TVoxel,TIndex> *scene, const ITMPose *pose,
	const ITMIntrinsics *intrinsics, ITMFloat2Image *minmaximg, const ITMVisualisationState *state)
{
	Vector2f *minmaxData = minmaximg->GetData(true);

	{
		dim3 blockSize(256);
		dim3 gridSize((int)ceil((float)minmaximg->dataSize / (float)blockSize.x));
		Vector2f init;
		//TODO : this could be improved a bit...
		init.x = 0.2f; init.y = 3.0f;
		memsetKernel_device<Vector2f> << <gridSize, blockSize >> >(minmaxData, init, minmaximg->dataSize);
	}
}

template<class TVoxel>
void ITMVisualisationEngine_CUDA<TVoxel,ITMVoxelBlockHash>::CreateExpectedDepths(const ITMScene<TVoxel,ITMVoxelBlockHash> *scene, const ITMPose *pose,
	const ITMIntrinsics *intrinsics, ITMFloat2Image *minmaximg, const ITMVisualisationState *state)
{
	float voxelSize = scene->sceneParams->voxelSize;

	Vector2i imgSize = minmaximg->noDims;
	Vector2f *minmaxData = minmaximg->GetData(true);

	{
		dim3 blockSize(256);
		dim3 gridSize((int)ceil((float)minmaximg->dataSize / (float)blockSize.x));
		Vector2f init;
		init.x = FAR_AWAY; init.y = VERY_CLOSE;
		memsetKernel_device<Vector2f> << <gridSize, blockSize >> >(minmaxData, init, minmaximg->dataSize);
	}

	//go through list of visible 8x8x8 blocks
	{
		const ITMHashEntry *hash_entries = scene->index.GetEntries();
		const int *liveEntryIDs = scene->index.GetLiveEntryIDs();
		int noLiveEntries = scene->index.noLiveEntries;
		if (state != NULL) {
			const State *s = (const State*)state;
			liveEntryIDs = s->visibleEntryIDs;
			noLiveEntries = s->visibleEntriesNum;
		}

		dim3 blockSize(256);
		dim3 gridSize((int)ceil((float)noLiveEntries / (float)blockSize.x));
		ITMSafeCall(cudaMemset(noTotalBlocks_device, 0, sizeof(uint)));
		projectAndSplitBlocks_device << <gridSize, blockSize >> >(hash_entries, liveEntryIDs, noLiveEntries, pose->M,
			intrinsics->projectionParamsSimple.all, imgSize, voxelSize, renderingBlockList_device, noTotalBlocks_device);
	}

	uint noTotalBlocks;
	ITMSafeCall(cudaMemcpy(&noTotalBlocks, noTotalBlocks_device, sizeof(uint), cudaMemcpyDeviceToHost));
	if (noTotalBlocks > (unsigned)MAX_RENDERING_BLOCKS) noTotalBlocks = MAX_RENDERING_BLOCKS;

	// go through rendering blocks
	{
		// fill minmaxData
		dim3 blockSize(16, 16);
		dim3 gridSize((unsigned int)ceil((float)noTotalBlocks/4.0f), 4);
		fillBlocks_device << <gridSize, blockSize >> >(noTotalBlocks_device, renderingBlockList_device, imgSize, minmaxData);
	}
}

template<class TVoxel, class TIndex>
static void RenderImage_common(ITMScene<TVoxel,TIndex> *scene, const ITMPose *pose, const ITMIntrinsics *intrinsics, const ITMVisualisationState *state, ITMUChar4Image *outputImage, bool useColour)
{ //printf("000000000000000\n");
	Vector2i imgSize = outputImage->noDims;
	float oneOverVoxelSize = 1.0f / scene->sceneParams->voxelSize;

	Matrix4f invM = pose->invM;
	Vector4f projParams = intrinsics->projectionParamsSimple.all;
	projParams.x = 1.0f / projParams.x;
	projParams.y = 1.0f / projParams.y;

	float mu = scene->sceneParams->mu;
	Vector3f lightSource = -Vector3f(invM.getColumn(2));

	Vector4u *outRendering = outputImage->GetData(true);
	const Vector2f *minmaximg = state->minmaxImage->GetData(true);

	dim3 cudaBlockSize(8, 8);
	dim3 gridSize((int)ceil((float)imgSize.x / (float)cudaBlockSize.x), (int)ceil((float)imgSize.y / (float)cudaBlockSize.y));

	if (useColour&&TVoxel::hasColorInformation) {
		RaycastRenderer_ColourImage<TVoxel,TIndex> renderer(outRendering, scene->localVBA.GetVoxelBlocks(), scene->index.getIndexData());

		genericRaycastAndRender_device<TVoxel,TIndex> << <gridSize, cudaBlockSize >> >(renderer, scene->localVBA.GetVoxelBlocks(), scene->index.getIndexData(), imgSize, invM, projParams, oneOverVoxelSize, minmaximg, mu, lightSource, NULL, NULL);
	} else {
		RaycastRenderer_GrayImage renderer(outRendering);

		genericRaycastAndRender_device<TVoxel,TIndex> << <gridSize, cudaBlockSize >> >(renderer, scene->localVBA.GetVoxelBlocks(), scene->index.getIndexData(), imgSize, invM, projParams, oneOverVoxelSize, minmaximg, mu, lightSource, NULL, NULL);
	}
}

template<class TVoxel, class TIndex>
class RaycastRenderer_PointCloud {
	private:
	Vector4u *outRendering;
	Vector4f *locations;
	Vector4f *colours;
	uint *noTotalPoints;
	float voxelSize;
	bool skipPoints;
	const TVoxel *voxelData;
	const typename TIndex::IndexData *voxelIndex;

	public:
	RaycastRenderer_PointCloud(Vector4u *_outRendering, Vector4f *_locations, Vector4f *_colours, uint *_noTotalPoints, float _voxelSize, bool _skipPoints, const TVoxel *_voxelData, const typename TIndex::IndexData *_voxelIndex)
	 : outRendering(_outRendering), locations(_locations), colours(_colours),
	   noTotalPoints(_noTotalPoints), voxelSize(_voxelSize), skipPoints(_skipPoints),
	   voxelData(_voxelData), voxelIndex(_voxelIndex)
	{}

	__device__ inline void processPixel(int x, int y, int locId, bool foundPoint, const Vector3f & point, const Vector3f & outNormal, float angle)
	{
		__shared__ bool shouldPrefix;
		shouldPrefix = false;
		__syncthreads();

		drawRendering(foundPoint, angle, outRendering[locId]);

		if (skipPoints && ((x % 2 == 0) || (y % 2 == 0))) foundPoint = false;
		if (foundPoint) shouldPrefix = true;
		__syncthreads();

		if (shouldPrefix)
		{
			int offset = computePrefixSum_device<uint>(foundPoint, noTotalPoints, blockDim.x * blockDim.y, threadIdx.x + threadIdx.y * blockDim.x);

			if (offset != -1)
			{
				Vector4f tmp;
				tmp = VoxelColorReader<TVoxel::hasColorInformation,TVoxel,typename TIndex::IndexData>::interpolate(voxelData, voxelIndex, point);
				if (tmp.w > 0.0f) { tmp.x /= tmp.w; tmp.y /= tmp.w; tmp.z /= tmp.w; tmp.w = 1.0f; }
				colours[offset] = tmp;

				Vector4f pt_ray_out;
				pt_ray_out.x = point.x * voxelSize; pt_ray_out.y = point.y * voxelSize;
				pt_ray_out.z = point.z * voxelSize; pt_ray_out.w = 1.0f;
				locations[offset] = pt_ray_out;
			}
		}
	}

   //hao modified it
  __device__ inline void new_processPixel(int x, int y, int locId, bool foundPoint, const Vector3f & point, const Vector3f & outNormal, float angle, Vector3f color)
  {
    __shared__ bool shouldPrefix;
		shouldPrefix = false;
		__syncthreads();

		drawRendering(foundPoint, angle, outRendering[locId]);

		if (skipPoints && ((x % 2 == 0) || (y % 2 == 0))) foundPoint = false;
		if (foundPoint) shouldPrefix = true;
		__syncthreads();

		if (shouldPrefix)
		{
			int offset = computePrefixSum_device<uint>(foundPoint, noTotalPoints, blockDim.x * blockDim.y, threadIdx.x + threadIdx.y * blockDim.x);

			if (offset != -1)
			{
				Vector4f tmp;
				tmp = VoxelColorReader<TVoxel::hasColorInformation,TVoxel,typename TIndex::IndexData>::interpolate(voxelData, voxelIndex, point);
				if (tmp.w > 0.0f) { tmp.x /= tmp.w; tmp.y /= tmp.w; tmp.z /= tmp.w; tmp.w = 1.0f; }
				colours[offset] = tmp;

				Vector4f pt_ray_out;
				pt_ray_out.x = point.x * voxelSize; pt_ray_out.y = point.y * voxelSize;
				pt_ray_out.z = point.z * voxelSize; pt_ray_out.w = 1.0f;
				locations[offset] = pt_ray_out;
			}
		}
  }
};

template<class TVoxel, class TIndex>
static void CreatePointCloud_common(ITMScene<TVoxel,TIndex> *scene, const ITMView *view, ITMTrackingState *trackingState, bool skipPoints, uint *noTotalPoints_device)
{ //printf("1111111111111\n");
	Vector2i imgSize = view->rgb->noDims;
	float voxelSize = scene->sceneParams->voxelSize;
	float oneOverVoxelSize = 1.0f / scene->sceneParams->voxelSize;

	Matrix4f invM = trackingState->pose_d->invM * view->calib->trafo_rgb_to_depth.calib;
	Vector4f projParams = view->calib->intrinsics_rgb.projectionParamsSimple.all;
	projParams.x = 1.0f / projParams.x;
	projParams.y = 1.0f / projParams.y;

	Vector4f *locations = trackingState->pointCloud->locations->GetData(true);
	Vector4f *colours = trackingState->pointCloud->colours->GetData(true);
	Vector4u *outRendering = trackingState->rendering->GetData(true);
	Vector2f *minmaxdata = trackingState->renderingRangeImage->GetData(true);

	float mu = scene->sceneParams->mu;
	Vector3f lightSource = -Vector3f(invM.getColumn(2));

	dim3 cudaBlockSize(16, 16);
	dim3 gridSize = getGridSize(imgSize, cudaBlockSize);

	ITMSafeCall(cudaMemset(noTotalPoints_device, 0, sizeof(uint)));

	RaycastRenderer_PointCloud<TVoxel,TIndex> renderer(outRendering, locations, colours, noTotalPoints_device, voxelSize, skipPoints, scene->localVBA.GetVoxelBlocks(), scene->index.getIndexData());

	genericRaycastAndRender_device<TVoxel,TIndex> << <gridSize, cudaBlockSize >> >(renderer, scene->localVBA.GetVoxelBlocks(), scene->index.getIndexData(), imgSize, invM, projParams, oneOverVoxelSize, minmaxdata, mu, lightSource, NULL, NULL);

	ITMSafeCall(cudaMemcpy(&trackingState->pointCloud->noTotalPoints, noTotalPoints_device, sizeof(uint), cudaMemcpyDeviceToHost));
}

//hao modified it
template<class TVoxel, class TIndex>
void CreateICPMaps_common(ITMScene<TVoxel,TIndex> *scene, const ITMView *view, ITMTrackingState *trackingState, float x0, float x1, float y0, float y1, float z)
{ //printf("222222222222222\n");
	Vector2i imgSize = view->depth->noDims;
	float voxelSize = scene->sceneParams->voxelSize;
	float oneOverVoxelSize = 1.0f / scene->sceneParams->voxelSize;

	Matrix4f invM = trackingState->pose_d->invM;
	Vector4f projParams = view->calib->intrinsics_d.projectionParamsSimple.all;
	projParams.x = 1.0f / projParams.x;
	projParams.y = 1.0f / projParams.y;

	float mu = scene->sceneParams->mu;
	Vector3f lightSource = -Vector3f(invM.getColumn(2));

	Vector4f *pointsMap = trackingState->pointCloud->locations->GetData(true);
	Vector4f *normalsMap = trackingState->pointCloud->colours->GetData(true);
	Vector4u *outRendering = trackingState->rendering->GetData(true);
	Vector2f *minmaxdata = trackingState->renderingRangeImage->GetData(true);

	dim3 cudaBlockSize(8, 8);
	dim3 gridSize((int)ceil((float)imgSize.x / (float)cudaBlockSize.x), (int)ceil((float)imgSize.y / (float)cudaBlockSize.y));

	RaycastRenderer_ICPMaps renderer(outRendering, pointsMap, normalsMap, voxelSize);

  Vector3f *points;
  Vector3f *normals;
  Vector3f *colors;
  ushort *objectIds;
  Vector3f *table_sum_normal;
  ITMSafeCall(cudaMalloc((void**)&points, view->depth->noDims.x*view->depth->noDims.y*sizeof(Vector3f)));
  ITMSafeCall(cudaMemset(points, 0, view->depth->noDims.x*view->depth->noDims.y*sizeof(Vector3f)));
  ITMSafeCall(cudaMalloc((void**)&normals, view->depth->noDims.x*view->depth->noDims.y*sizeof(Vector3f)));
  ITMSafeCall(cudaMemset(normals, 0, view->depth->noDims.x*view->depth->noDims.y*sizeof(Vector3f)));
  ITMSafeCall(cudaMalloc((void**)&colors, view->depth->noDims.x*view->depth->noDims.y*sizeof(Vector3f)));
  ITMSafeCall(cudaMemset(colors, 0, view->depth->noDims.x*view->depth->noDims.y*sizeof(Vector3f)));
  ITMSafeCall(cudaMalloc((void**)&objectIds, view->depth->noDims.x*view->depth->noDims.y*sizeof(ushort)));
  ITMSafeCall(cudaMemset(objectIds, 0, view->depth->noDims.x*view->depth->noDims.y*sizeof(ushort)));
  ITMSafeCall(cudaMalloc((void**)&table_sum_normal, view->depth->noDims.x*view->depth->noDims.y*sizeof(Vector3f)));
  ITMSafeCall(cudaMemset(table_sum_normal, 0, view->depth->noDims.x*view->depth->noDims.y*sizeof(Vector3f)));
  
  //printf("000000\n");
  getRaycastImage_device<TVoxel,TIndex> << <gridSize, cudaBlockSize >> >(renderer, scene->localVBA.GetVoxelBlocks(), scene->index.getIndexData(), imgSize, invM, projParams, oneOverVoxelSize, minmaxdata, mu, lightSource, points, normals, colors, objectIds, table_sum_normal);
  ITMSafeCall(cudaThreadSynchronize());
  
  Vector3f* table_avg_normal;
  ITMSafeCall(cudaMalloc((void**)&table_avg_normal, sizeof(Vector3f)));
  ITMSafeCall(cudaMemset(table_avg_normal, 0, sizeof(Vector3f)));
  //printf("111111\n");
  copute_table_avg_normal_device << <dim3(1), dim3(1) >> >(table_sum_normal, view->depth->noDims.x*view->depth->noDims.y, table_avg_normal);
  ITMSafeCall(cudaThreadSynchronize());
  //printf("222222\n");

  for(int i=0; i<20; i++){
  genIdMaps1_device << <gridSize, cudaBlockSize >> >(imgSize, points, normals, colors, objectIds, table_avg_normal);
  ITMSafeCall(cudaThreadSynchronize());
  }

  genIdMaps2_device << <gridSize, cudaBlockSize >> >(imgSize, x0, x1, y0, y1, z, points, normals, colors, objectIds);
  ITMSafeCall(cudaThreadSynchronize());

  //printf("333333\n");
	genericRaycastAndRender_device<TVoxel,TIndex> << <gridSize, cudaBlockSize >> >(renderer, scene->localVBA.GetVoxelBlocks(), scene->index.getIndexData(), imgSize, invM, projParams, oneOverVoxelSize, minmaxdata, mu, lightSource, colors, objectIds);
	ITMSafeCall(cudaThreadSynchronize());
  //printf("444444\n");
  ITMSafeCall(cudaFree(points));
  ITMSafeCall(cudaFree(normals));
  ITMSafeCall(cudaFree(colors));
  ITMSafeCall(cudaFree(objectIds));
  ITMSafeCall(cudaFree(table_sum_normal));
  ITMSafeCall(cudaFree(table_avg_normal));
}

template<class TVoxel, class TIndex>
void ITMVisualisationEngine_CUDA<TVoxel,TIndex>::RenderImage(ITMScene<TVoxel,TIndex> *scene, const ITMPose *pose, const ITMIntrinsics *intrinsics, const ITMVisualisationState *state, ITMUChar4Image *outputImage, bool useColour)
{
	RenderImage_common(scene, pose, intrinsics, state, outputImage, useColour);
}

template<class TVoxel>
void ITMVisualisationEngine_CUDA<TVoxel,ITMVoxelBlockHash>::RenderImage(ITMScene<TVoxel,ITMVoxelBlockHash> *scene, const ITMPose *pose, const ITMIntrinsics *intrinsics, const ITMVisualisationState *state, ITMUChar4Image *outputImage, bool useColour)
{
	RenderImage_common(scene, pose, intrinsics, state, outputImage, useColour);
}

template<class TVoxel, class TIndex>
void ITMVisualisationEngine_CUDA<TVoxel,TIndex>::CreatePointCloud(ITMScene<TVoxel,TIndex> *scene, const ITMView *view, ITMTrackingState *trackingState, bool skipPoints)
{
	CreatePointCloud_common(scene, view, trackingState, skipPoints, noTotalPoints_device);
}

template<class TVoxel>
void ITMVisualisationEngine_CUDA<TVoxel,ITMVoxelBlockHash>::CreatePointCloud(ITMScene<TVoxel,ITMVoxelBlockHash> *scene, const ITMView *view, ITMTrackingState *trackingState, bool skipPoints)
{
	CreatePointCloud_common(scene, view, trackingState, skipPoints, noTotalPoints_device);
}

template<class TVoxel, class TIndex>
void ITMVisualisationEngine_CUDA<TVoxel,TIndex>::CreateICPMaps(ITMScene<TVoxel,TIndex> *scene, const ITMView *view, ITMTrackingState *trackingState, float x0, float x1, float y0, float y1, float z)
{
	CreateICPMaps_common(scene, view, trackingState, x0, x1, y0, y1, z);
}

template<class TVoxel>
void ITMVisualisationEngine_CUDA<TVoxel,ITMVoxelBlockHash>::CreateICPMaps(ITMScene<TVoxel,ITMVoxelBlockHash> *scene, const ITMView *view, ITMTrackingState *trackingState, float x0, float x1, float y0, float y1, float z)
{ 
	CreateICPMaps_common(scene, view, trackingState, x0, x1, y0, y1, z);
}

//hao modified it
template<class TVoxel, class TIndex>
void ITMVisualisationEngine_CUDA<TVoxel,TIndex>::NewCreateICPMaps(ITMScene<TVoxel,TIndex> *scene, const ITMView *view, ITMTrackingState *trackingState, Vector3f *colors, ushort *objectIds, bool flag){
  
	Vector2i imgSize = view->depth->noDims;
	float voxelSize = scene->sceneParams->voxelSize;
	float oneOverVoxelSize = 1.0f / scene->sceneParams->voxelSize;

	Matrix4f invM = trackingState->pose_d->invM;
	Vector4f projParams = view->calib->intrinsics_d.projectionParamsSimple.all;
	projParams.x = 1.0f / projParams.x;
	projParams.y = 1.0f / projParams.y;

	float mu = scene->sceneParams->mu;
	Vector3f lightSource = -Vector3f(invM.getColumn(2));

	Vector4f *pointsMap = trackingState->pointCloud->locations->GetData(true);
	Vector4f *normalsMap = trackingState->pointCloud->colours->GetData(true);
	Vector4u *outRendering = trackingState->rendering->GetData(true);
	Vector2f *minmaxdata = trackingState->renderingRangeImage->GetData(true);

	dim3 cudaBlockSize(8, 8);
	dim3 gridSize((int)ceil((float)imgSize.x / (float)cudaBlockSize.x), (int)ceil((float)imgSize.y / (float)cudaBlockSize.y));

	RaycastRenderer_ICPMaps renderer(outRendering, pointsMap, normalsMap, voxelSize);

	new_genericRaycastAndRender_device<TVoxel,TIndex> << <gridSize, cudaBlockSize >> >(renderer, scene->localVBA.GetVoxelBlocks(), scene->index.getIndexData(), imgSize, invM, projParams, oneOverVoxelSize, minmaxdata, mu, lightSource, colors, objectIds, flag);

	ITMSafeCall(cudaThreadSynchronize());
}


//hao modified it
template<class TVoxel>
void ITMVisualisationEngine_CUDA<TVoxel,ITMVoxelBlockHash>::NewCreateICPMaps(ITMScene<TVoxel,ITMVoxelBlockHash> *scene, const ITMView *view, ITMTrackingState *trackingState, Vector3f *colors, ushort *objectIds, bool flag){
  //printf("sizeof(id_array):%d\n", sizeof(id_array));
  
  /*for(int i=0; i<sizeof(id_array)/sizeof(int); i++){
      printf("id_array:%d\n", *(id_array+i));
  }*/

	Vector2i imgSize = view->depth->noDims;
	float voxelSize = scene->sceneParams->voxelSize;
	float oneOverVoxelSize = 1.0f / scene->sceneParams->voxelSize;

	Matrix4f invM = trackingState->pose_d->invM;
	Vector4f projParams = view->calib->intrinsics_d.projectionParamsSimple.all;
	projParams.x = 1.0f / projParams.x;
	projParams.y = 1.0f / projParams.y;

	float mu = scene->sceneParams->mu;
	Vector3f lightSource = -Vector3f(invM.getColumn(2));

	Vector4f *pointsMap = trackingState->pointCloud->locations->GetData(true);
	Vector4f *normalsMap = trackingState->pointCloud->colours->GetData(true);
	Vector4u *outRendering = trackingState->rendering->GetData(true);
	Vector2f *minmaxdata = trackingState->renderingRangeImage->GetData(true);

	dim3 cudaBlockSize(8, 8);
	dim3 gridSize((int)ceil((float)imgSize.x / (float)cudaBlockSize.x), (int)ceil((float)imgSize.y / (float)cudaBlockSize.y));

	RaycastRenderer_ICPMaps renderer(outRendering, pointsMap, normalsMap, voxelSize);

	new_genericRaycastAndRender_device<TVoxel,ITMVoxelBlockHash> << <gridSize, cudaBlockSize >> >(renderer, scene->localVBA.GetVoxelBlocks(), scene->index.getIndexData(), imgSize, invM, projParams, oneOverVoxelSize, minmaxdata, mu, lightSource, colors, objectIds, flag);

	ITMSafeCall(cudaThreadSynchronize());
}

//hao modified it
template<class TVoxel, class TIndex>
void ITMVisualisationEngine_CUDA<TVoxel,TIndex>::GetRaycastImage(ITMScene<TVoxel,TIndex> *scene, const ITMView *view, ITMTrackingState *trackingState, Vector3f *points, Vector3f *normals, Vector3f *colors, ushort *objectId){
  Vector2i imgSize = view->depth->noDims;
	float voxelSize = scene->sceneParams->voxelSize;
	float oneOverVoxelSize = 1.0f / scene->sceneParams->voxelSize;

	Matrix4f invM = trackingState->pose_d->invM;
	Vector4f projParams = view->calib->intrinsics_d.projectionParamsSimple.all;
	projParams.x = 1.0f / projParams.x;
	projParams.y = 1.0f / projParams.y;

	float mu = scene->sceneParams->mu;
  Vector3f lightSource = -Vector3f(invM.getColumn(2));
	//Vector3f lightSource = -Vector3f(invM.getColumn(2));
  //Vector3f *points;
  //ITMSafeCall(cudaMalloc((void**)&points, imgSize.x*imgSize.y*sizeof(Vector3f)));

	Vector4f *pointsMap = trackingState->pointCloud->locations->GetData(true);
	Vector4f *normalsMap = trackingState->pointCloud->colours->GetData(true);
	Vector4u *outRendering = trackingState->rendering->GetData(true);
	Vector2f *minmaxdata = trackingState->renderingRangeImage->GetData(true);

	dim3 cudaBlockSize(8, 8);
	dim3 gridSize((int)ceil((float)imgSize.x / (float)cudaBlockSize.x), (int)ceil((float)imgSize.y / (float)cudaBlockSize.y));

	RaycastRenderer_ICPMaps renderer(outRendering, pointsMap, normalsMap, voxelSize);

	getRaycastImage_device<TVoxel,TIndex> << <gridSize, cudaBlockSize >> >(renderer, scene->localVBA.GetVoxelBlocks(), scene->index.getIndexData(), imgSize, invM, projParams, oneOverVoxelSize, minmaxdata, mu, lightSource, points, normals, colors, objectId, NULL);

	ITMSafeCall(cudaThreadSynchronize());
}

//hao modified it
template<class TVoxel>
void ITMVisualisationEngine_CUDA<TVoxel,ITMVoxelBlockHash>::GetRaycastImage(ITMScene<TVoxel,ITMVoxelBlockHash> *scene, const ITMView *view, ITMTrackingState *trackingState, Vector3f *points, Vector3f *normals, Vector3f *colors, ushort *objectId){
  Vector2i imgSize = view->depth->noDims;
	float voxelSize = scene->sceneParams->voxelSize;
	float oneOverVoxelSize = 1.0f / scene->sceneParams->voxelSize;

	Matrix4f invM = trackingState->pose_d->invM;
	Vector4f projParams = view->calib->intrinsics_d.projectionParamsSimple.all;
	projParams.x = 1.0f / projParams.x;
	projParams.y = 1.0f / projParams.y;

	float mu = scene->sceneParams->mu;
  Vector3f lightSource = -Vector3f(invM.getColumn(2));
	//Vector3f lightSource = -Vector3f(invM.getColumn(2));
  //Vector3f *points;
  //ITMSafeCall(cudaMalloc((void**)&points, imgSize.x*imgSize.y*sizeof(Vector3f)));

	Vector4f *pointsMap = trackingState->pointCloud->locations->GetData(true);
	Vector4f *normalsMap = trackingState->pointCloud->colours->GetData(true);
	Vector4u *outRendering = trackingState->rendering->GetData(true);
	Vector2f *minmaxdata = trackingState->renderingRangeImage->GetData(true);

	dim3 cudaBlockSize(8, 8);
	dim3 gridSize((int)ceil((float)imgSize.x / (float)cudaBlockSize.x), (int)ceil((float)imgSize.y / (float)cudaBlockSize.y));

	RaycastRenderer_ICPMaps renderer(outRendering, pointsMap, normalsMap, voxelSize);

	getRaycastImage_device<TVoxel,ITMVoxelBlockHash> << <gridSize, cudaBlockSize >> >(renderer, scene->localVBA.GetVoxelBlocks(), scene->index.getIndexData(), imgSize, invM, projParams, oneOverVoxelSize, minmaxdata, mu, lightSource, points, normals, colors, objectId, NULL);

	ITMSafeCall(cudaThreadSynchronize());
}

//hao modified it
template<class TVoxel, class TIndex>
void ITMVisualisationEngine_CUDA<TVoxel,TIndex>::GetAllPoints(const ITMScene<TVoxel,TIndex> *scene, Vector3f *points, Vector3f *normals, Vector3f *colors, ushort *objectId, int &num){
  dim3 cudaBlockSize(1);
	dim3 gridSize(1);
  getAllPoints_device<TVoxel,ITMVoxelBlockHash> << <gridSize, cudaBlockSize >> >(scene->localVBA.GetVoxelBlocks(), scene->index.getIndexData(), scene->sceneParams->mu, points, normals, colors, objectId, num);
  ITMSafeCall(cudaThreadSynchronize()); 
}

//hao modified it
template<class TVoxel>
void ITMVisualisationEngine_CUDA<TVoxel,ITMVoxelBlockHash>::GetAllPoints(const ITMScene<TVoxel,ITMVoxelBlockHash> *scene, Vector3f *points, Vector3f *normals, Vector3f *colors, ushort *objectId, int &num){
  dim3 cudaBlockSize(1);
	dim3 gridSize(1);
  getAllPoints_device<TVoxel,ITMVoxelBlockHash> << <gridSize, cudaBlockSize >> >(scene->localVBA.GetVoxelBlocks(), scene->index.getIndexData(), scene->sceneParams->mu, points, normals, colors, objectId, num);
  ITMSafeCall(cudaThreadSynchronize()); 
}

//hao modified it
template<class TVoxel, class TIndex>
void ITMVisualisationEngine_CUDA<TVoxel,TIndex>::computePointNormal(ITMScene<TVoxel,TIndex> *scene, Vector3f pt, Vector3f *normal){
  dim3 cudaBlockSize(1);
	dim3 gridSize(1);
  computePointNormal_device<TVoxel,TIndex> << <gridSize, cudaBlockSize >> >(scene->localVBA.GetVoxelBlocks(), scene->index.getIndexData(), pt, normal);
  ITMSafeCall(cudaThreadSynchronize()); 
}

//hao modified it
template<class TVoxel>
void ITMVisualisationEngine_CUDA<TVoxel,ITMVoxelBlockHash>::computePointNormal(ITMScene<TVoxel,ITMVoxelBlockHash> *scene, Vector3f pt, Vector3f *normal){
  dim3 cudaBlockSize(1);
	dim3 gridSize(1);
  computePointNormal_device<TVoxel,ITMVoxelBlockHash> << <gridSize, cudaBlockSize >> >(scene->localVBA.GetVoxelBlocks(), scene->index.getIndexData(), pt, normal);
  ITMSafeCall(cudaThreadSynchronize()); 
}

//device implementations

__global__ void buildVisibleList_device(const ITMHashEntry *hashTable, /*ITMHashCacheState *cacheStates, bool useSwapping,*/ int noTotalEntries,
	int *liveEntryIDs, int *noLiveEntries, uchar *entriesVisibleType, Matrix4f M, Vector4f projParams, Vector2i imgSize, float voxelSize)
{
	int targetIdx = threadIdx.x + blockIdx.x * blockDim.x;
	if (targetIdx > noTotalEntries - 1) return;

	__shared__ bool shouldPrefix;

	unsigned char hashVisibleType = 0; //entriesVisibleType[targetIdx];
	const ITMHashEntry &hashEntry = hashTable[targetIdx];

	shouldPrefix = false;
	__syncthreads();

	if (hashEntry.ptr >= 0)
	{
		shouldPrefix = true;

		Vector3f pt_image, buff3f;

		int noInvisible = 0;//, noInvisibleEnlarged = 0;

/*		pt_image = hashEntry.pos.toFloat() * (float)SDF_BLOCK_SIZE * voxelSize;
		buff3f = M * pt_image;

		if (buff3f.z > 1e-10f)
		{
			shouldPrefix = true;*/

		for (int x = 0; x <= 1; x++) for (int y = 0; y <= 1; y++) for (int z = 0; z <= 1; z++)
		{
			Vector3f off((float)x, (float)y, (float)z);

			pt_image = (hashEntry.pos.toFloat() + off) * (float)SDF_BLOCK_SIZE * voxelSize;

			buff3f = M * pt_image;

			if (buff3f.z < 1e-10f) continue;

			pt_image.x = projParams.x * buff3f.x / buff3f.z + projParams.z;
			pt_image.y = projParams.y * buff3f.y / buff3f.z + projParams.w;

			if (!(pt_image.x >= 0 && pt_image.x < imgSize.x && pt_image.y >= 0 && pt_image.y < imgSize.y)) noInvisible++;

/*			if (useSwapping)
			{
				Vector4i lims;
				lims.x = -imgSize.x / 8; lims.y = imgSize.x + imgSize.x / 8;
				lims.z = -imgSize.y / 8; lims.w = imgSize.y + imgSize.y / 8;

				if (!(pt_image.x >= lims.x && pt_image.x < lims.y && pt_image.y >= lims.z && pt_image.y < lims.w)) noInvisibleEnlarged++;
			}*/
		}

		hashVisibleType = noInvisible < 8;

		//if (useSwapping) entriesVisibleType[targetIdx] = noInvisibleEnlarged < 8;
	}

	/*if (useSwapping)
	{
		if (entriesVisibleType[targetIdx] > 0 && cacheStates[targetIdx].cacheFromHost != 2) cacheStates[targetIdx].cacheFromHost = 1;
	}*/

	if (hashVisibleType > 0) shouldPrefix = true;

	__syncthreads();

	if (shouldPrefix)
	{
		int offset = computePrefixSum_device<int>(hashVisibleType > 0, noLiveEntries, blockDim.x * blockDim.y, threadIdx.x);
		if (offset != -1) liveEntryIDs[offset] = targetIdx;
	}
}

template<typename T> __global__ void memsetKernel_device(T *devPtr, const T val, size_t nwords)
{
	size_t offset = threadIdx.x + blockDim.x * blockIdx.x;
	if (offset >= nwords) return;
	devPtr[offset] = val;
}


__global__ void projectAndSplitBlocks_device(const ITMHashEntry *hashEntries, const int *liveEntryIDs, int noLiveEntries,
	const Matrix4f pose_M, const Vector4f intrinsics, const Vector2i imgSize, float voxelSize, RenderingBlock *renderingBlocks,
	uint *noTotalBlocks)
{
	int in_offset = threadIdx.x + blockDim.x * blockIdx.x;

	const ITMHashEntry & blockData(hashEntries[liveEntryIDs[in_offset]]);

	Vector2i upperLeft, lowerRight;
	Vector2f zRange;
	bool validProjection = false;
	if (in_offset < noLiveEntries) if (blockData.ptr>=0) {
		validProjection = ProjectSingleBlock(blockData.pos, pose_M, intrinsics, imgSize, voxelSize, upperLeft, lowerRight, zRange);
	}

	Vector2i requiredRenderingBlocks(ceilf((float)(lowerRight.x - upperLeft.x + 1) / renderingBlockSizeX),
		ceilf((float)(lowerRight.y - upperLeft.y + 1) / renderingBlockSizeY));

	size_t requiredNumBlocks = requiredRenderingBlocks.x * requiredRenderingBlocks.y;
	if (!validProjection) requiredNumBlocks = 0;

	int out_offset = computePrefixSum_device<uint>(requiredNumBlocks, noTotalBlocks, blockDim.x, threadIdx.x);
	if (!validProjection) return;
	if ((out_offset == -1) || (out_offset + requiredNumBlocks > MAX_RENDERING_BLOCKS)) return;

	CreateRenderingBlocks(renderingBlocks, out_offset, upperLeft, lowerRight, zRange);
}

__device__ static inline void atomicMin(float* address, float val)
{
	int* address_as_i = (int*)address;
	int old = *address_as_i, assumed;
	do {
		assumed = old;
		old = ::atomicCAS(address_as_i, assumed,
			__float_as_int(::fminf(val, __int_as_float(assumed))));
	} while (assumed != old);
}

__device__ static inline void atomicMax(float* address, float val)
{
	int* address_as_i = (int*)address;
	int old = *address_as_i, assumed;
	do {
		assumed = old;
		old = ::atomicCAS(address_as_i, assumed,
			__float_as_int(::fmaxf(val, __int_as_float(assumed))));
	} while (assumed != old);
}

__global__ void fillBlocks_device(const uint *noTotalBlocks, const RenderingBlock *renderingBlocks,
	Vector2i imgSize, Vector2f *minmaxData)
{
	int x = threadIdx.x;
	int y = threadIdx.y;
	int block = blockIdx.x*4 + blockIdx.y;
	if (block >= *noTotalBlocks) return;

	const RenderingBlock & b(renderingBlocks[block]);
	int xpos = b.upperLeft.x + x;
	if (xpos > b.lowerRight.x) return;
	int ypos = b.upperLeft.y + y;
	if (ypos > b.lowerRight.y) return;

	Vector2f & pixel(minmaxData[xpos + ypos*imgSize.x]);
	atomicMin(&pixel.x, b.zRange.x);
	atomicMax(&pixel.y, b.zRange.y);
}

//hao modified it
template<class TVoxel, class TIndex, class TRaycastRenderer>
__global__ void genericRaycastAndRender_device(TRaycastRenderer renderer,
	TVoxel *voxelData, const typename TIndex::IndexData *voxelIndex, Vector2i imgSize, Matrix4f invM, Vector4f projParams, 
	float oneOverVoxelSize, const Vector2f *minmaxdata, float mu, 
	Vector3f lightSource, Vector3f* colors, ushort* objectIds)
{
	int x = (threadIdx.x + blockIdx.x * blockDim.x), y = (threadIdx.y + blockIdx.y * blockDim.y);

	if (x >= imgSize.x || y >= imgSize.y) return;

  genericRaycastAndRender<TVoxel,TIndex>(x,y, renderer, voxelData, voxelIndex, imgSize, invM, projParams, oneOverVoxelSize, minmaxdata, mu, lightSource, colors, objectIds);
	//genericRaycastAndRender<TVoxel,TIndex>(x,y, renderer, voxelData, voxelIndex, imgSize, invM, projParams, oneOverVoxelSize, minmaxdata, mu, lightSource);
}

//hao modified it
template<class TVoxel, class TIndex, class TRaycastRenderer>
__global__ void new_genericRaycastAndRender_device(TRaycastRenderer renderer,
	TVoxel *voxelData, const typename TIndex::IndexData *voxelIndex, Vector2i imgSize, Matrix4f invM, Vector4f projParams, 
	float oneOverVoxelSize, const Vector2f *minmaxdata, float mu, 
	Vector3f lightSource, Vector3f *colors, ushort *objectIds, bool flag)
{
	int x = (threadIdx.x + blockIdx.x * blockDim.x), y = (threadIdx.y + blockIdx.y * blockDim.y);

	if (x >= imgSize.x || y >= imgSize.y) return;

	new_genericRaycastAndRender<TVoxel,TIndex>(x,y, renderer, voxelData, voxelIndex, imgSize, invM, projParams, oneOverVoxelSize, minmaxdata, mu, lightSource, colors, objectIds, flag);
}

//hao modified it
template<class TVoxel, class TIndex, class TRaycastRenderer>
__global__ void getRaycastImage_device(TRaycastRenderer renderer,
	TVoxel *voxelData, const typename TIndex::IndexData *voxelIndex, Vector2i imgSize, Matrix4f invM, Vector4f projParams, 
	float oneOverVoxelSize, const Vector2f *minmaxdata, float mu, Vector3f lightSource, Vector3f *points, Vector3f *normals, 
  Vector3f *colors, ushort *objectIds, Vector3f *table_sum_normal)
{
	int x = (threadIdx.x + blockIdx.x * blockDim.x), y = (threadIdx.y + blockIdx.y * blockDim.y);

	if (x >= imgSize.x || y >= imgSize.y) return;

  getRaycastImage<TVoxel,TIndex>(x,y, renderer, voxelData, voxelIndex, imgSize, invM, projParams, oneOverVoxelSize, minmaxdata, mu, lightSource, points, normals, colors, objectIds, table_sum_normal);
	//getDepthValue<TVoxel,TIndex>(x,y, renderer, voxelData, voxelIndex, imgSize, invM, projParams, oneOverVoxelSize, minmaxdata, mu, lightSource, points, normals);
}

//hao modified it
__global__ void genIdMaps1_device(Vector2i imgSize, Vector3f *points, Vector3f *normals, Vector3f *colors, ushort *objectIds, Vector3f *table_avg_normal)
{
	int x = (threadIdx.x + blockIdx.x * blockDim.x), y = (threadIdx.y + blockIdx.y * blockDim.y);

	if (x >= imgSize.x || y >= imgSize.y) return;

  genIdMaps1(x, y, imgSize, points, normals, colors, objectIds, table_avg_normal);
}

//hao modified it
__global__ void genIdMaps2_device(Vector2i imgSize, float x0, float x1, float y0, float y1, float z, Vector3f *points, Vector3f *normals, Vector3f *colors, ushort *objectIds)
{
	int x = (threadIdx.x + blockIdx.x * blockDim.x), y = (threadIdx.y + blockIdx.y * blockDim.y);

	if (x >= imgSize.x || y >= imgSize.y) return;

  genIdMaps2(x, y, imgSize, x0, x1, y0, y1, z, points, normals, colors, objectIds);
}

//hao modified it
__global__ void copute_table_avg_normal_device(Vector3f *table_sum_normal, int length, Vector3f* table_avg_normal){
  Vector3f tem(0,0,0);

  for(int i=0; i<length; i+=10){
    tem+=table_sum_normal[i];
  }

  if(!(tem.x == 0&&tem.y == 0&&tem.z == 0)){
  (*table_avg_normal) = tem/sqrt(tem.x*tem.x+tem.y*tem.y+tem.z*tem.z);
  }
 
  //printf("length:%d\n", length);
  //printf("table_avg_normal[0].x:%f\n", table_avg_normal[0].x);
  //printf("table_avg_normal[0].y:%f\n", table_avg_normal[0].y);
  //printf("table_avg_normal[0].z:%f\n", table_avg_normal[0].z);
}

//hao modified it
template<class TVoxel, class TIndex>
__global__ void getAllPoints_device(const TVoxel *voxelData, const typename TIndex::IndexData *voxelIndex, float mu, Vector3f *points, Vector3f *normals, Vector3f *colors, ushort *objectId, int &num){

  //ITMVoxelIndex::IndexData *hashData_host = new ITMVoxelIndex::IndexData;voxelIndex
  //ITMVoxel *voxels = (ITMVoxel*)malloc(SDF_LOCAL_BLOCK_NUM*SDF_BLOCK_SIZE3 * sizeof(ITMVoxel));voxelData

  const ITMHashEntry *hashTable = voxelIndex->entries_all;
  int cout = 0;

  for(int i=0; i<SDF_BUCKET_NUM * SDF_ENTRY_NUM_PER_BUCKET + SDF_EXCESS_LIST_SIZE; i++){
    const ITMHashEntry &hashEntry = hashTable[i];

    if(hashEntry.ptr >= 0){
      for(int j=0; j<SDF_BLOCK_SIZE3; j++){
        ITMVoxel res = voxelData[(hashEntry.ptr * SDF_BLOCK_SIZE3) + j];

        float value = ITMVoxel::SDF_valueToFloat(res.sdf);
        if(value<10*mu&&value>-10*mu){

          float voxelSize = 0.125f;
          float blockSizeWorld = 0.005*SDF_BLOCK_SIZE; // = 0.005*8;

          (*(points+cout))[2] = (hashEntry.pos.z+(j/(SDF_BLOCK_SIZE*SDF_BLOCK_SIZE)+0.5f)*voxelSize)*blockSizeWorld;
          (*(points+cout))[1] = (hashEntry.pos.y+((j%(SDF_BLOCK_SIZE*SDF_BLOCK_SIZE))/SDF_BLOCK_SIZE+0.5f)*voxelSize)*blockSizeWorld;
          (*(points+cout))[0] = (hashEntry.pos.x+((j%(SDF_BLOCK_SIZE*SDF_BLOCK_SIZE))%SDF_BLOCK_SIZE+0.5f)*voxelSize)*blockSizeWorld;
          (*(colors+cout))[0] = res.r;
          (*(colors+cout))[1] = res.g;
          (*(colors+cout))[2] = res.b;

          Vector3f pt((hashEntry.pos.x*SDF_BLOCK_SIZE+((j%(SDF_BLOCK_SIZE*SDF_BLOCK_SIZE))%SDF_BLOCK_SIZE+0.5f)), (hashEntry.pos.y*SDF_BLOCK_SIZE+((j%(SDF_BLOCK_SIZE*SDF_BLOCK_SIZE))/SDF_BLOCK_SIZE+0.5f)), (hashEntry.pos.z*SDF_BLOCK_SIZE+(j/(SDF_BLOCK_SIZE*SDF_BLOCK_SIZE)+0.5f)));

          (*(normals+cout)) = computeSingleNormalFromSDF(voxelData, voxelIndex, pt);

          cout++;
        }
      }
    }
  }
}

//hao modified it
template<class TVoxel, class TIndex>
__global__ void computePointNormal_device(const TVoxel *voxelData, const typename TIndex::IndexData *voxelIndex, Vector3f pt, Vector3f *normal){
  (*normal) = computeSingleNormalFromSDF(voxelData, voxelIndex, pt);
}


////hao modified it
//template<class TVoxel, class TIndex>
//__global__ void genericRaycast_device(const TVoxel *voxelData, const typename TIndex::IndexData *voxelIndex, Vector2i imgSize, Matrix4f invM, Vector4f projParams, 
//	float oneOverVoxelSize, const Vector2f *minmaxdata, float mu, 
//	Vector3f lightSource)
//{
//	int x = (threadIdx.x + blockIdx.x * blockDim.x), y = (threadIdx.y + blockIdx.y * blockDim.y);
//
//	if (x >= imgSize.x || y >= imgSize.y) return;
//
//  Vector3f pt_ray;
//	Vector3f outNormal;
//	float angle;
//
//	int locId = x + y * imgSize.x;
//
//	float viewFrustum_min = minmaxdata[locId].x;
//	float viewFrustum_max = minmaxdata[locId].y;
//
//	bool foundPoint = castRay<TVoxel,TIndex>(pt_ray, x, y, voxelData, voxelIndex, invM, projParams, oneOverVoxelSize, mu, viewFrustum_min, viewFrustum_max);
//
//	computeNormalAndAngle<TVoxel,TIndex>(foundPoint, pt_ray, voxelData, voxelIndex, lightSource, outNormal, angle);
//}

template class ITMLib::Engine::ITMVisualisationEngine_CUDA<ITMVoxel, ITMVoxelIndex>;
