// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../ITMVisualisationEngine.h"

namespace ITMLib
{
	namespace Engine
	{
		template<class TVoxel, class TIndex>
		class ITMVisualisationEngine_CPU : public ITMVisualisationEngine<TVoxel,TIndex>
		{
		public:
			ITMVisualisationState* allocateInternalState(const Vector2i & imgSize)
			{ return new ITMVisualisationState(imgSize, false); }

			void FindVisibleBlocks(const ITMScene<TVoxel,TIndex> *scene, const ITMPose *pose, const ITMIntrinsics *intrinsics, ITMVisualisationState *state);
			void CreateExpectedDepths(const ITMScene<TVoxel,TIndex> *scene, const ITMPose *pose, const ITMIntrinsics *intrinsics, ITMFloat2Image *minmaxImg, const ITMVisualisationState *state = NULL);
			void RenderImage(ITMScene<TVoxel,TIndex> *scene, const ITMPose *pose, const ITMIntrinsics *intrinsics, const ITMVisualisationState *state, ITMUChar4Image *outputImage, bool useColour);
			void CreatePointCloud(ITMScene<TVoxel,TIndex> *scene, const ITMView *view, ITMTrackingState *trackingState, bool skipPoints);
			void CreateICPMaps(ITMScene<TVoxel,TIndex> *scene, const ITMView *view, ITMTrackingState *trackingState, float x0, float x1, float y0, float y1, float z);
      void GetAllPoints(const ITMScene<TVoxel,TIndex> *scene, Vector3f *points, Vector3f *normals, Vector3f *colors, ushort *objectId, int &num);//hao modified it
      void GetRaycastImage(ITMScene<TVoxel,TIndex> *scene, const ITMView *view, ITMTrackingState *trackingState, Vector3f *points, Vector3f *normals, Vector3f *colors, ushort *objectId);//hao modified it
		  void NewCreateICPMaps(ITMScene<TVoxel,TIndex> *scene, const ITMView *view, ITMTrackingState *trackingState, Vector3f *colors, ushort *objectIds, bool flag);//hao modified it
      void computePointNormal(ITMScene<TVoxel,TIndex> *scene, Vector3f pt, Vector3f *normal);
    };

		template<class TVoxel>
		class ITMVisualisationEngine_CPU<TVoxel,ITMVoxelBlockHash> : public ITMVisualisationEngine<TVoxel,ITMVoxelBlockHash>
		{
		public:
			class State : public ITMVisualisationState {
				public:
				State(const Vector2i & imgSize);
				~State(void);

				uchar *entriesVisibleType;
				int *visibleEntryIDs;
				int visibleEntriesNum;
			};

			ITMVisualisationState* allocateInternalState(const Vector2i & imgSize)
			{ return new State(imgSize); }

			void FindVisibleBlocks(const ITMScene<TVoxel,ITMVoxelBlockHash> *scene, const ITMPose *pose, const ITMIntrinsics *intrinsics, ITMVisualisationState *state);
			void CreateExpectedDepths(const ITMScene<TVoxel,ITMVoxelBlockHash> *scene, const ITMPose *pose, const ITMIntrinsics *intrinsics, ITMFloat2Image *minmaxImg, const ITMVisualisationState *state = NULL);
			void RenderImage(ITMScene<TVoxel,ITMVoxelBlockHash> *scene, const ITMPose *pose, const ITMIntrinsics *intrinsics, const ITMVisualisationState *state, ITMUChar4Image *outputImage, bool useColour);
			void CreatePointCloud(ITMScene<TVoxel,ITMVoxelBlockHash> *scene, const ITMView *view, ITMTrackingState *trackingState, bool skipPoints);
			void CreateICPMaps(ITMScene<TVoxel,ITMVoxelBlockHash> *scene, const ITMView *view, ITMTrackingState *trackingState, float x0, float x1, float y0, float y1, float z);
      void GetAllPoints(const ITMScene<TVoxel,ITMVoxelBlockHash> *scene, Vector3f *points, Vector3f *normals, Vector3f *colors, ushort *objectId, int &num);//hao modified it
      void GetRaycastImage(ITMScene<TVoxel,ITMVoxelBlockHash> *scene, const ITMView *view, ITMTrackingState *trackingState, Vector3f *points, Vector3f *normals, Vector3f *colors, ushort *objectId);//hao modified it
		  void NewCreateICPMaps(ITMScene<TVoxel,ITMVoxelBlockHash> *scene, const ITMView *view, ITMTrackingState *trackingState, Vector3f *colors, ushort *objectIds, bool flag);//hao modified it
      void computePointNormal(ITMScene<TVoxel,ITMVoxelBlockHash> *scene, Vector3f pt, Vector3f *normal);
    };
	}
}
