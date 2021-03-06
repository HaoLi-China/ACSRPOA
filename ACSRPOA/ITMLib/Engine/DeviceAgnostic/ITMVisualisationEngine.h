// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../Utils/ITMLibDefines.h"
#include <vector>//hao modified it
#include "math.h"//hao modified it

using namespace std;//hao modified it

struct RenderingBlock {
  Vector2s upperLeft;
  Vector2s lowerRight;
  Vector2f zRange;
};

#ifndef FAR_AWAY
#define FAR_AWAY 999999.9f
#endif

#ifndef VERY_CLOSE
#define VERY_CLOSE 0.05f
#endif

static const int renderingBlockSizeX = 16;
static const int renderingBlockSizeY = 16;

static const int MAX_RENDERING_BLOCKS = 65536*4;
//static const int MAX_RENDERING_BLOCKS = 16384;

_CPU_AND_GPU_CODE_ inline bool ProjectSingleBlock(const Vector3s & blockPos, const Matrix4f & pose, const Vector4f & intrinsics, const Vector2i & imgSize, float voxelSize, Vector2i & upperLeft, Vector2i & lowerRight, Vector2f & zRange)
{
  upperLeft = imgSize;
  lowerRight = Vector2i(-1, -1);
  zRange = Vector2f(FAR_AWAY, VERY_CLOSE);
  for (int corner = 0; corner < 8; ++corner)
  {
    // project all 8 corners down to 2D image
    Vector3s tmp = blockPos;
    tmp.x += (corner & 1) ? 1 : 0;
    tmp.y += (corner & 2) ? 1 : 0;
    tmp.z += (corner & 4) ? 1 : 0;
    Vector4f pt3d(tmp.toFloat() * (float)SDF_BLOCK_SIZE * voxelSize, 1.0f);
    pt3d = pose * pt3d;
    if (pt3d.z < 1e-6) continue;

    Vector2f pt2d;
    pt2d.x = intrinsics.x * pt3d.x / pt3d.z + intrinsics.z;
    pt2d.y = intrinsics.y * pt3d.y / pt3d.z + intrinsics.w;

    // remember bounding box, zmin and zmax
    if (upperLeft.x > floorf(pt2d.x)) upperLeft.x = (int)floorf(pt2d.x);
    if (lowerRight.x < ceilf(pt2d.x)) lowerRight.x = (int)ceilf(pt2d.x);
    if (upperLeft.y > floorf(pt2d.y)) upperLeft.y = (int)floorf(pt2d.y);
    if (lowerRight.y < ceilf(pt2d.y)) lowerRight.y = (int)ceilf(pt2d.y);
    if (zRange.x > pt3d.z) zRange.x = pt3d.z;
    if (zRange.y < pt3d.z) zRange.y = pt3d.z;
  }

  // do some sanity checks and respect image bounds
  if (upperLeft.x < 0) upperLeft.x = 0;
  if (upperLeft.y < 0) upperLeft.y = 0;
  if (lowerRight.x >= imgSize.x) lowerRight.x = imgSize.x - 1;
  if (lowerRight.y >= imgSize.y) lowerRight.y = imgSize.y - 1;
  if (upperLeft.x > lowerRight.x) return false;
  if (upperLeft.y > lowerRight.y) return false;
  //if (zRange.y <= VERY_CLOSE) return false; never seems to happen
  if (zRange.x < VERY_CLOSE) zRange.x = VERY_CLOSE;
  if (zRange.y < VERY_CLOSE) return false;

  return true;
}

_CPU_AND_GPU_CODE_ inline void CreateRenderingBlocks(RenderingBlock *renderingBlockList, int offset, const Vector2i & upperLeft, const Vector2i & lowerRight, const Vector2f & zRange)
{
  // split bounding box into 16x16 pixel rendering blocks
  for (int by = 0; by < ceilf((float)(1 + lowerRight.y - upperLeft.y) / renderingBlockSizeY); ++by) {
    for (int bx = 0; bx < ceilf((float)(1 + lowerRight.x - upperLeft.x) / renderingBlockSizeX); ++bx) {
      if (offset >= MAX_RENDERING_BLOCKS) return;
      //for each rendering block: add it to the list
      RenderingBlock & b(renderingBlockList[offset++]);
      b.upperLeft.x = upperLeft.x + bx*renderingBlockSizeX;
      b.upperLeft.y = upperLeft.y + by*renderingBlockSizeY;
      b.lowerRight.x = upperLeft.x + (bx + 1)*renderingBlockSizeX - 1;
      b.lowerRight.y = upperLeft.y + (by + 1)*renderingBlockSizeY - 1;
      if (b.lowerRight.x>lowerRight.x) b.lowerRight.x = lowerRight.x;
      if (b.lowerRight.y>lowerRight.y) b.lowerRight.y = lowerRight.y;
      b.zRange = zRange;
    }
  }
}

//hao modified it
template<class TVoxel, class TIndex>
_CPU_AND_GPU_CODE_ inline bool castRay(Vector3f &pt_out, int x, int y, const TVoxel *voxelData, const typename TIndex::IndexData *voxelIndex, Matrix4f invM,
  Vector4f projParams, float oneOverVoxelSize, float mu, float viewFrustum_min, float viewFrustum_max)
{
  Vector3f pt_camera_f, pt_block_s, pt_block_e, rayDirection, pt_result;
  bool pt_found, hash_found;
  float sdfValue = 1.0f;
  float totalLength, stepLength, totalLengthMax, stepScale;

  stepScale = mu * oneOverVoxelSize;

  pt_camera_f.z = viewFrustum_min;
  pt_camera_f.x = pt_camera_f.z * ((float(x) - projParams.z) * projParams.x);
  pt_camera_f.y = pt_camera_f.z * ((float(y) - projParams.w) * projParams.y);
  totalLength = length(pt_camera_f)*oneOverVoxelSize;
  pt_block_s = (invM * pt_camera_f) * oneOverVoxelSize;

  pt_camera_f.z = viewFrustum_max;
  pt_camera_f.x = pt_camera_f.z * ((float(x) - projParams.z) * projParams.x);
  pt_camera_f.y = pt_camera_f.z * ((float(y) - projParams.w) * projParams.y);
  totalLengthMax = length(pt_camera_f)*oneOverVoxelSize;
  pt_block_e = (invM * pt_camera_f) * oneOverVoxelSize;

  rayDirection = (pt_block_e - pt_block_s).normalised();
  pt_result = pt_block_s;

  enum { SEARCH_BLOCK_COARSE, SEARCH_BLOCK_FINE, SEARCH_SURFACE, BEHIND_SURFACE, WRONG_SIDE } state;

  sdfValue = readFromSDF_float_uninterpolated(voxelData, voxelIndex, pt_result, hash_found);

  if (!hash_found) state = SEARCH_BLOCK_COARSE;
  else if (sdfValue <= 0.0f) state = WRONG_SIDE;
  else state = SEARCH_SURFACE;

  typename TIndex::IndexCache cache;

  pt_found = false;
  while (state != BEHIND_SURFACE)
  {
    if (!hash_found)
    {
      switch (state)
      {
      case SEARCH_BLOCK_COARSE: stepLength = SDF_BLOCK_SIZE; break;
      case SEARCH_BLOCK_FINE: stepLength = stepScale; break;
      default:
      case WRONG_SIDE:
      case SEARCH_SURFACE:
        state = SEARCH_BLOCK_COARSE;
        stepLength = SDF_BLOCK_SIZE;
        break;
      }
    }
    else {
      switch (state)
      {
      case SEARCH_BLOCK_COARSE:
        state = SEARCH_BLOCK_FINE;
        stepLength = stepScale - SDF_BLOCK_SIZE;
        break;
      case WRONG_SIDE: stepLength = MIN(sdfValue * stepScale, -1.0f); break;
      case SEARCH_BLOCK_FINE: state = SEARCH_SURFACE;
      default:
      case SEARCH_SURFACE: stepLength = MAX(sdfValue * stepScale, 1.0f);
      }
    }

    pt_result += stepLength * rayDirection; totalLength += stepLength;
    if (totalLength > totalLengthMax) break;

    sdfValue = readFromSDF_float_maybe_interpolate(voxelData, voxelIndex, pt_result, hash_found, cache);

    if (sdfValue <= 0.0f) if (state == SEARCH_BLOCK_FINE) state = WRONG_SIDE; else state = BEHIND_SURFACE;
    else if (state == WRONG_SIDE) state = SEARCH_SURFACE;
  }

  if (state == BEHIND_SURFACE)
  {
    stepLength = MIN(sdfValue * stepScale, -0.5f);

    pt_result += stepLength * rayDirection;

    sdfValue = readFromSDF_float_interpolated(voxelData, voxelIndex, pt_result, hash_found);

    stepLength = sdfValue * stepScale;

    pt_result += stepLength * rayDirection;
    pt_found = true;
  }

  pt_out = pt_result;
  return pt_found;
}

//hao modified it
template<class TVoxel, class TIndex>
_CPU_AND_GPU_CODE_ inline bool newCastRay(vector<Vector3f> &pt_out, int x, int y, const TVoxel *voxelData, const typename TIndex::IndexData *voxelIndex, Matrix4f invM,
  Vector4f projParams, float oneOverVoxelSize, float mu, float viewFrustum_min, float viewFrustum_max)
{
  Vector3f pt_camera_f, pt_block_s, pt_block_e, rayDirection, pt_result;
  bool pt_found, hash_found;
  float sdfValue = 1.0f;
  float totalLength, stepLength, totalLengthMax, stepScale;

  stepScale = mu * oneOverVoxelSize;

  pt_camera_f.z = viewFrustum_min;
  pt_camera_f.x = pt_camera_f.z * ((float(x) - projParams.z) * projParams.x);
  pt_camera_f.y = pt_camera_f.z * ((float(y) - projParams.w) * projParams.y);
  totalLength = length(pt_camera_f)*oneOverVoxelSize;
  pt_block_s = (invM * pt_camera_f) * oneOverVoxelSize;

  pt_camera_f.z = viewFrustum_max;
  pt_camera_f.x = pt_camera_f.z * ((float(x) - projParams.z) * projParams.x);
  pt_camera_f.y = pt_camera_f.z * ((float(y) - projParams.w) * projParams.y);
  totalLengthMax = length(pt_camera_f)*oneOverVoxelSize;
  pt_block_e = (invM * pt_camera_f) * oneOverVoxelSize;

  rayDirection = (pt_block_e - pt_block_s).normalised();
  pt_result = pt_block_s;

  enum { SEARCH_BLOCK_COARSE, SEARCH_BLOCK_FINE, SEARCH_SURFACE, BEHIND_SURFACE, WRONG_SIDE } state;

  sdfValue = readFromSDF_float_uninterpolated(voxelData, voxelIndex, pt_result, hash_found);
  if (!hash_found) state = SEARCH_BLOCK_COARSE;
  else if (sdfValue <= 0.0f) state = WRONG_SIDE;
  else state = SEARCH_SURFACE;

  typename TIndex::IndexCache cache;

  pt_found = false;
  while (state != BEHIND_SURFACE)
  {
    if (!hash_found)
    {
      switch (state)
      {
      case SEARCH_BLOCK_COARSE: stepLength = SDF_BLOCK_SIZE; break;
      case SEARCH_BLOCK_FINE: stepLength = stepScale; break;
      default:
      case WRONG_SIDE:
      case SEARCH_SURFACE:
        state = SEARCH_BLOCK_COARSE;
        stepLength = SDF_BLOCK_SIZE;
        break;
      }
    }
    else {
      switch (state)
      {
      case SEARCH_BLOCK_COARSE:
        pt_out.push_back(pt_result);
        state = SEARCH_BLOCK_FINE;
        stepLength = stepScale - SDF_BLOCK_SIZE;
        break;
      case WRONG_SIDE: stepLength = MIN(sdfValue * stepScale, -1.0f); break;
      case SEARCH_BLOCK_FINE: 
        pt_out.push_back(pt_result);
        state = SEARCH_SURFACE;
      default:
      case SEARCH_SURFACE: stepLength = MAX(sdfValue * stepScale, 1.0f);
      }
    }

    pt_result += stepLength * rayDirection; totalLength += stepLength;
    if (totalLength > totalLengthMax) break;

    sdfValue = readFromSDF_float_maybe_interpolate(voxelData, voxelIndex, pt_result, hash_found, cache);

    if (sdfValue <= 0.0f) if (state == SEARCH_BLOCK_FINE) state = WRONG_SIDE; else state = BEHIND_SURFACE;
    else if (state == WRONG_SIDE) state = SEARCH_SURFACE;
  }

  if (state == BEHIND_SURFACE)
  {
    stepLength = MIN(sdfValue * stepScale, -0.5f);

    pt_result += stepLength * rayDirection;

    sdfValue = readFromSDF_float_interpolated(voxelData, voxelIndex, pt_result, hash_found);

    stepLength = sdfValue * stepScale;

    pt_result += stepLength * rayDirection;
    pt_found = true;
  }

  pt_out.push_back(pt_result);
  return pt_found;
}

template<class TVoxel, class TIndex>
_CPU_AND_GPU_CODE_ inline void computeNormalAndAngle(bool & foundPoint, const Vector3f & point, const TVoxel *voxelBlockData, const typename TIndex::IndexData *indexData, const Vector3f & lightSource, Vector3f & outNormal, float & angle)
{
  if (!foundPoint) return;

  outNormal = computeSingleNormalFromSDF(voxelBlockData, indexData, point);

  float normScale = 1.0f / sqrtf(outNormal.x * outNormal.x + outNormal.y * outNormal.y + outNormal.z * outNormal.z);
  outNormal *= normScale;

  angle = outNormal.x * lightSource.x + outNormal.y * lightSource.y + outNormal.z * lightSource.z;
  if (!(angle > 0.0)) foundPoint = false;
}

_CPU_AND_GPU_CODE_ inline void drawRendering(const bool & foundPoint, const float & angle, Vector4u & dest)
{
  if (!foundPoint)
  {
    dest = Vector4u((const uchar&)0);
    return;
  }

  float outRes = (0.8f * angle + 0.2f) * 255.0f;
  dest = Vector4u((uchar)outRes);
}

//hao modified it
_CPU_AND_GPU_CODE_ inline void newDrawRendering(const bool & foundPoint, const float & angle, Vector4u & dest, Vector3f color)
{
  if (!foundPoint)
  {
    dest = Vector4u((const uchar&)0);
    return;
  }

  float outRes = (0.8f * angle + 0.2f);

  if(!(color[0]==-1&&color[1]==-1&&color[2]==-1)){
    dest.x = (uchar)(color[0]*outRes);
    dest.y = (uchar)(color[1]*outRes);
    dest.z = (uchar)(color[2]*outRes);
    dest.w = outRes;
  }
  else{
    dest = Vector4u((uchar)(outRes*255));
  }
}

template<class TVoxel, class TIndex>
_CPU_AND_GPU_CODE_ inline void drawColourRendering(const bool & foundPoint, const Vector3f & point, const TVoxel *voxelBlockData, const typename TIndex::IndexData *indexData, Vector4u & dest)
{
  if (!foundPoint)
  {
    dest = Vector4u((const uchar&)0);
    return;
  }

  Vector4f clr = VoxelColorReader<TVoxel::hasColorInformation,TVoxel,typename TIndex::IndexData>::interpolate(voxelBlockData, indexData, point);

  dest.x = (uchar)(clr.r * 255.0f);
  dest.y = (uchar)(clr.g * 255.0f);
  dest.z = (uchar)(clr.b * 255.0f);
  dest.w = 255;
}

class RaycastRenderer_GrayImage {
private:
  Vector4u *outRendering;
public:
  RaycastRenderer_GrayImage(Vector4u *out)
  { outRendering = out; }

  _CPU_AND_GPU_CODE_ inline void processPixel(int x, int y, int locId, bool foundPoint, const Vector3f & point, const Vector3f & outNormal, float angle)
  {
    drawRendering(foundPoint, angle, outRendering[locId]);
  }

  //hao modified it
  _CPU_AND_GPU_CODE_ inline void new_processPixel(int x, int y, int locId, bool foundPoint, const Vector3f & point, const Vector3f & outNormal, float angle, Vector3f color){
    drawRendering(foundPoint, angle, outRendering[locId]);
  }
};

template<class TVoxel, class TIndex>
class RaycastRenderer_ColourImage {
private:
  const TVoxel *voxelData;
  const typename TIndex::IndexData *voxelIndex;

  Vector4u *outRendering;

public:
  RaycastRenderer_ColourImage(Vector4u *out, const TVoxel *_voxelData, const typename TIndex::IndexData *_voxelIndex)
  { outRendering = out; voxelData = _voxelData; voxelIndex = _voxelIndex; }

  _CPU_AND_GPU_CODE_ inline void processPixel(int x, int y, int locId, bool foundPoint, const Vector3f & point, const Vector3f & outNormal, float angle)
  {
    drawColourRendering<TVoxel,TIndex>(foundPoint, point, voxelData, voxelIndex, outRendering[locId]);
  }

  //hao modified it
  _CPU_AND_GPU_CODE_ inline void new_processPixel(int x, int y, int locId, bool foundPoint, const Vector3f & point, const Vector3f & outNormal, float angle, Vector3f color){
    drawColourRendering<TVoxel,TIndex>(foundPoint, point, voxelData, voxelIndex, outRendering[locId]);
  }

};

class RaycastRenderer_ICPMaps {
private:
  Vector4u *outRendering;
  Vector4f *pointsMap;
  Vector4f *normalsMap;
  float voxelSize;

public:
  RaycastRenderer_ICPMaps(Vector4u *_outRendering, Vector4f *_pointsMap, Vector4f *_normalsMap, float _voxelSize)
    : outRendering(_outRendering), pointsMap(_pointsMap), normalsMap(_normalsMap), voxelSize(_voxelSize)
  {}

  //hao modified it
  _CPU_AND_GPU_CODE_ inline void processPixel(int x, int y, int locId, bool foundPoint, const Vector3f & point, const Vector3f & outNormal, float angle)
  {
    drawRendering(foundPoint, angle, outRendering[locId]);

    if (foundPoint)
    {
      Vector4f outPoint4;
      outPoint4.x = point.x * voxelSize; outPoint4.y = point.y * voxelSize;
      outPoint4.z = point.z * voxelSize; outPoint4.w = 1.0f;
      pointsMap[locId] = outPoint4;

      Vector4f outNormal4;
      outNormal4.x = outNormal.x; outNormal4.y = outNormal.y; outNormal4.z = outNormal.z; outNormal4.w = 0.0f;
      normalsMap[locId] = outNormal4;
    }
    else
    {
      Vector4f out4;
      out4.x = 0.0f; out4.y = 0.0f; out4.z = 0.0f; out4.w = -1.0f;

      pointsMap[locId] = out4;
      normalsMap[locId] = out4;
    }
  }

  //hao modified it
  _CPU_AND_GPU_CODE_ inline void new_processPixel(int x, int y, int locId, bool foundPoint, const Vector3f & point, const Vector3f & outNormal, float angle, Vector3f color)
  {
    newDrawRendering(foundPoint, angle, outRendering[locId], color);

    if (foundPoint)
    {
      Vector4f outPoint4;
      outPoint4.x = point.x * voxelSize; outPoint4.y = point.y * voxelSize;
      outPoint4.z = point.z * voxelSize; outPoint4.w = 1.0f;
      pointsMap[locId] = outPoint4;

      Vector4f outNormal4;
      outNormal4.x = outNormal.x; outNormal4.y = outNormal.y; outNormal4.z = outNormal.z; outNormal4.w = 0.0f;
      normalsMap[locId] = outNormal4;
    }
    else
    {
      Vector4f out4;
      out4.x = 0.0f; out4.y = 0.0f; out4.z = 0.0f; out4.w = -1.0f;

      pointsMap[locId] = out4;
      normalsMap[locId] = out4;
    }
  }
};


//hao modified it
template<class TVoxel, class TIndex, class TRaycastRenderer>
_CPU_AND_GPU_CODE_ inline void genericRaycastAndRender(int x, int y, TRaycastRenderer & renderer,
  TVoxel *voxelData, const typename TIndex::IndexData *voxelIndex, Vector2i imgSize, Matrix4f invM, Vector4f projParams,
  float oneOverVoxelSize, const Vector2f *minmaxdata, float mu, Vector3f lightSource, Vector3f* colors, ushort* objectIds)
{
  Vector3f pt_ray;
  Vector3f outNormal;
  float angle;

  //ushort voxelId = 0;
  //uchar voxelR;
  //uchar voxelG;
  //uchar voxelB;
  bool hash_found;

  int locId = x + y * imgSize.x;

  float viewFrustum_min = minmaxdata[locId].x;
  float viewFrustum_max = minmaxdata[locId].y;

  bool foundPoint = castRay<TVoxel,TIndex>(pt_ray, x, y, voxelData, voxelIndex, invM, projParams, oneOverVoxelSize, mu, viewFrustum_min, viewFrustum_max);

  if(foundPoint){
    if(objectIds[locId] != 0){
      setVoxelIdAndRGB(voxelData, voxelIndex, pt_ray, hash_found, objectIds[locId], (uchar)(colors[locId].x), (uchar)(colors[locId].y), (uchar)(colors[locId].z));
    }
  }
  //if(foundPoint){
  //  readVoxelIdAndRGB(voxelData, voxelIndex, pt_ray, hash_found, voxelId, voxelR, voxelG, voxelB);
  //  //test
  //  printf("voxelG:%d\n",voxelG);
  //}

  computeNormalAndAngle<TVoxel,TIndex>(foundPoint, pt_ray, voxelData, voxelIndex, lightSource, outNormal, angle);

  if(objectIds[locId] == 0){
    renderer.processPixel(x,y, locId, foundPoint, pt_ray, outNormal, angle);
  }
  else{
    //Vector3f color;
    //color[0] = voxelR;
    //color[1] = voxelG;
    //color[2] = voxelB;
    renderer.new_processPixel(x,y, locId, foundPoint, pt_ray, outNormal, angle, colors[locId]);
  }
}

//hao modified it
template<class TVoxel, class TIndex, class TRaycastRenderer>
_CPU_AND_GPU_CODE_ inline void new_genericRaycastAndRender(int x, int y, TRaycastRenderer & renderer,
  TVoxel *voxelData, const typename TIndex::IndexData *voxelIndex, Vector2i imgSize, Matrix4f invM, Vector4f projParams,
  float oneOverVoxelSize, const Vector2f *minmaxdata, float mu, Vector3f lightSource, Vector3f *colors, ushort *objectIds, bool flag)
{
  Vector3f pt_ray;
  Vector3f outNormal;
  float angle;
  bool hash_found;

  int locId = x + y * imgSize.x;

  float viewFrustum_min = minmaxdata[locId].x;
  float viewFrustum_max = minmaxdata[locId].y;

  bool foundPoint = castRay<TVoxel,TIndex>(pt_ray, x, y, voxelData, voxelIndex, invM, projParams, oneOverVoxelSize, mu, viewFrustum_min, viewFrustum_max);

  computeNormalAndAngle<TVoxel,TIndex>(foundPoint, pt_ray, voxelData, voxelIndex, lightSource, outNormal, angle);

  Vector3f color;
  color[0]=(*(colors+locId))[0];
  color[1]=(*(colors+locId))[1];
  color[2]=(*(colors+locId))[2];

  if(flag && objectIds!= NULL && foundPoint){
    if(!(color[0]==-1&&color[1]==-1&&color[2]==-1)){
      //test
      //printf("testtesttest\n");
      setVoxelIdAndRGB(voxelData, voxelIndex, pt_ray, hash_found, objectIds[locId], (uchar)color[0], (uchar)color[1], (uchar)color[2]);
    }
  }

  ////test
  //if(objectIds[locId]!=0){
  //  printf("objectIds[locId]!=0\n");
  //}

  renderer.new_processPixel(x,y, locId, foundPoint, pt_ray, outNormal, angle, color);
}

//hao modified it
template<class TVoxel, class TIndex, class TRaycastRenderer>
_CPU_AND_GPU_CODE_ inline void getRaycastImage(int x, int y, TRaycastRenderer & renderer,
  TVoxel *voxelData, const typename TIndex::IndexData *voxelIndex, Vector2i imgSize, Matrix4f invM, Vector4f projParams,
  float oneOverVoxelSize, const Vector2f *minmaxdata, float mu, Vector3f lightSource, Vector3f *points, Vector3f *normals, 
  Vector3f *colors, ushort *objectIds, Vector3f* table_sum_normal)
{
  Vector3f pt_ray;
  Vector3f pt_ray_world;
  Vector3f outNormal;
  float angle;
  bool hash_found;

  int locId = x + y * imgSize.x;

  float viewFrustum_min = minmaxdata[locId].x;
  float viewFrustum_max = minmaxdata[locId].y;

  bool foundPoint = castRay<TVoxel,TIndex>(pt_ray, x, y, voxelData, voxelIndex, invM, projParams, oneOverVoxelSize, mu, viewFrustum_min, viewFrustum_max);

  computeNormalAndAngle<TVoxel,TIndex>(foundPoint, pt_ray, voxelData, voxelIndex, lightSource, outNormal, angle);

  if(foundPoint){
    pt_ray_world = pt_ray*(1.0f/oneOverVoxelSize);
    (*(points+locId))[0]=pt_ray_world[0];
    (*(points+locId))[1]=pt_ray_world[1];
    (*(points+locId))[2]=pt_ray_world[2];
    (*(normals+locId))[0]=outNormal[0];
    (*(normals+locId))[1]=outNormal[1];
    (*(normals+locId))[2]=outNormal[2];
    if(colors != NULL && objectIds != NULL){
      uchar voxelR;
      uchar voxelG;
      uchar voxelB;
      readVoxelIdAndRGB(voxelData, voxelIndex, pt_ray, hash_found, objectIds[locId], voxelR, voxelG, voxelB);

      if(table_sum_normal!=NULL&&objectIds[locId] == 1){
        table_sum_normal[locId] = outNormal;
      }

      //test
      //if(voxelG !=0 ){
      //  printf("voxelG:%d\n",voxelG);
      //}

      //test
      //if(objectId[locId]!=0){
      //  printf("objectId[locId]:%d\n",objectId[locId]);
      //  printf("(*(objectId+locId)):%d\n",(*(objectId+locId)));
      //}

      colors[locId].x = voxelR;
      colors[locId].y = voxelG;
      colors[locId].z = voxelB;
    }
  }
  else{
    (*(points+locId))[0]=0;
    (*(points+locId))[1]=0;
    (*(points+locId))[2]=0;
    (*(normals+locId))[0]=outNormal[0];
    (*(normals+locId))[1]=outNormal[1];
    (*(normals+locId))[2]=outNormal[2];
  }
}

//hao modified it
_CPU_AND_GPU_CODE_ inline void genNeighborPoints(int x, int y, int *neighboorX, int *neighboorY){
  neighboorX[0] = x;
  neighboorX[1] = x+1;
  neighboorX[2] = x+1;
  neighboorX[3] = x+1;
  neighboorX[4] = x;
  neighboorX[5] = x-1;
  neighboorX[6] = x-1;
  neighboorX[7] = x-1;

  neighboorY[0] = y;
  neighboorY[1] = y+1;
  neighboorY[2] = y+1;
  neighboorY[3] = y+1;
  neighboorY[4] = y;
  neighboorY[5] = y-1;
  neighboorY[6] = y-1;
  neighboorY[7] = y-1;
}

////hao modified it
//_CPU_AND_GPU_CODE_ inline void genIdMaps(int x, int y, Vector2i imgSize, Vector3f *points, Vector3f *normals, Vector3f *colors, ushort *objectId, Vector3f* table_avg_normal)
//{
//  int cenIndex = x + y * imgSize.x;
//
//  if(objectId[cenIndex] != 0){
//    //test
//    //printf("hahahaha\n");
//
//    int upIndex = cenIndex - imgSize.x;
//    int downIndex = cenIndex + imgSize.x;
//    int leftIndex = cenIndex - 1;
//    int rightIndex = cenIndex + 1;
//
//    if(y - 1 >= 0){
//      if(objectId[upIndex]==0&&points[upIndex]!=0){
//        double dis = sqrt(pow(points[cenIndex].x - points[upIndex].x, 2) + pow(points[cenIndex].y - points[upIndex].y, 2) + pow(points[cenIndex].z - points[upIndex].z, 2));
//
//        int neighboorX[8];
//        int neighboorY[8];
//        genNeighborPoints(x, y-1, neighboorX, neighboorY);
//
//        int count0=0;
//        int count1=0;
//
//        for(int i=0; i<8; i++){
//          if(neighboorX[i]>=0&&neighboorX[i]<imgSize.x&&neighboorY[i]>0&&neighboorY[i]<imgSize.y){
//            int tem_index = neighboorX[i] + neighboorY[i] * imgSize.x;; 
//            double tem_dis = sqrt(pow(points[tem_index].x - points[upIndex].x, 2) + pow(points[tem_index].y - points[upIndex].y, 2) + pow(points[tem_index].z - points[upIndex].z, 2));
//
//            if(tem_dis < 0.005 && objectId[tem_index] == objectId[cenIndex]){
//              count0++;
//            }
//            else if(tem_dis < 0.005 && objectId[tem_index] != 0){
//              count1++;
//            }
//          }
//        }
//
//        if(count0>=count1){
//          if(dis < 0.005){
//            if(objectId[cenIndex] != 1){
//              objectId[upIndex] = objectId[cenIndex];
//              colors[upIndex] = colors[cenIndex];
//              //genIdMaps(x, y-1, imgSize, points, colors, objectId);
//            }
//            else{
//              double dotvalue = (normals[upIndex].x * (*table_avg_normal).x) + (normals[upIndex].y * (*table_avg_normal).y) + (normals[upIndex].z * (*table_avg_normal).z);
//              if(dotvalue>0.9){
//                objectId[upIndex] = objectId[cenIndex];
//                colors[upIndex] = colors[cenIndex];
//              }
//            }
//          }
//        }
//      }
//    }
//
//    if(y + 1 < imgSize.y){
//      if(objectId[downIndex]==0&&points[downIndex]!=0){
//        double dis = sqrt(pow(points[cenIndex].x - points[downIndex].x, 2) + pow(points[cenIndex].y - points[downIndex].y, 2) + pow(points[cenIndex].z - points[downIndex].z, 2));
//
//        int neighboorX[8];
//        int neighboorY[8];
//        genNeighborPoints(x, y-1, neighboorX, neighboorY);
//
//        int count0=0;
//        int count1=0;
//
//        for(int i=0; i<8; i++){
//          if(neighboorX[i]>=0&&neighboorX[i]<imgSize.x&&neighboorY[i]>0&&neighboorY[i]<imgSize.y){
//            int tem_index = neighboorX[i] + neighboorY[i] * imgSize.x;; 
//            double tem_dis = sqrt(pow(points[tem_index].x - points[downIndex].x, 2) + pow(points[tem_index].y - points[downIndex].y, 2) + pow(points[tem_index].z - points[downIndex].z, 2));
//
//            if(tem_dis < 0.005 && objectId[tem_index] == objectId[cenIndex]){
//              count0++;
//            }
//            else if(tem_dis < 0.005 && objectId[tem_index] != 0){
//              count1++;
//            }
//          }
//        }
//
//        if(count0>=count1){
//          if(dis < 0.005){
//            if(objectId[cenIndex] != 1){
//              objectId[downIndex] = objectId[cenIndex];
//              colors[downIndex] = colors[cenIndex];
//            }
//            else{
//              double dotvalue = (normals[downIndex].x * (*table_avg_normal).x) + (normals[downIndex].y * (*table_avg_normal).y) + (normals[downIndex].z * (*table_avg_normal).z);
//              if(dotvalue>0.9){
//                objectId[downIndex] = objectId[cenIndex];
//                colors[downIndex] = colors[cenIndex];
//              }
//            }
//          }
//        }
//      }
//    }
//
//    if(x - 1 >= 0){
//      if(objectId[leftIndex]==0&&points[leftIndex]!=0){
//        double dis = sqrt(pow(points[cenIndex].x - points[leftIndex].x, 2) + pow(points[cenIndex].y - points[leftIndex].y, 2) + pow(points[cenIndex].z - points[leftIndex].z, 2));
//
//        int neighboorX[8];
//        int neighboorY[8];
//        genNeighborPoints(x, y-1, neighboorX, neighboorY);
//
//        int count0=0;
//        int count1=0;
//
//        for(int i=0; i<8; i++){
//          if(neighboorX[i]>=0&&neighboorX[i]<imgSize.x&&neighboorY[i]>0&&neighboorY[i]<imgSize.y){
//            int tem_index = neighboorX[i] + neighboorY[i] * imgSize.x;; 
//            double tem_dis = sqrt(pow(points[tem_index].x - points[leftIndex].x, 2) + pow(points[tem_index].y - points[leftIndex].y, 2) + pow(points[tem_index].z - points[leftIndex].z, 2));
//
//            if(tem_dis < 0.005 && objectId[tem_index] == objectId[cenIndex]){
//              count0++;
//            }
//            else if(tem_dis < 0.005 && objectId[tem_index] != 0){
//              count1++;
//            }
//          }
//        }
//
//        if(count0>=count1){
//          if(dis < 0.005){
//            if(objectId[cenIndex] != 1){
//              objectId[leftIndex] = objectId[cenIndex];
//              colors[leftIndex] = colors[cenIndex];
//            }
//            else{
//              double dotvalue = (normals[leftIndex].x * (*table_avg_normal).x) + (normals[leftIndex].y * (*table_avg_normal).y) + (normals[leftIndex].z * (*table_avg_normal).z);
//              if(dotvalue>0.9){
//                objectId[leftIndex] = objectId[cenIndex];
//                colors[leftIndex] = colors[cenIndex];
//              }
//            }
//          }
//        }
//      }
//    }
//
//    if(x + 1 < imgSize.x){
//      if(objectId[rightIndex]==0&&points[rightIndex]!=0){
//        double dis = sqrt(pow(points[cenIndex].x - points[rightIndex].x, 2) + pow(points[cenIndex].y - points[rightIndex].y, 2) + pow(points[cenIndex].z - points[rightIndex].z, 2));
//
//        int neighboorX[8];
//        int neighboorY[8];
//        genNeighborPoints(x, y-1, neighboorX, neighboorY);
//
//        int count0=0;
//        int count1=0;
//
//        for(int i=0; i<8; i++){
//          if(neighboorX[i]>=0&&neighboorX[i]<imgSize.x&&neighboorY[i]>0&&neighboorY[i]<imgSize.y){
//            int tem_index = neighboorX[i] + neighboorY[i] * imgSize.x;; 
//            double tem_dis = sqrt(pow(points[tem_index].x - points[rightIndex].x, 2) + pow(points[tem_index].y - points[rightIndex].y, 2) + pow(points[tem_index].z - points[rightIndex].z, 2));
//
//            if(tem_dis < 0.005 && objectId[tem_index] == objectId[cenIndex]){
//              count0++;
//            }
//            else if(tem_dis < 0.005 && objectId[tem_index] != 0){
//              count1++;
//            }
//          }
//        }
//
//        if(count0>=count1){
//          if(dis < 0.005){
//            if(objectId[cenIndex] != 1){
//              objectId[rightIndex] = objectId[cenIndex];
//              colors[rightIndex] = colors[cenIndex];
//            }
//            else{
//              double dotvalue = (normals[rightIndex].x * (*table_avg_normal).x) + (normals[rightIndex].y * (*table_avg_normal).y) + (normals[rightIndex].z * (*table_avg_normal).z);
//              if(dotvalue>0.9){
//                objectId[rightIndex] = objectId[cenIndex];
//                colors[rightIndex] = colors[cenIndex];
//              }
//            }
//          }
//        }
//      }
//    }
//  }
//}


//hao modified it
_CPU_AND_GPU_CODE_ inline void genIdMaps1(int x, int y, Vector2i imgSize, Vector3f *points, Vector3f *normals, Vector3f *colors, ushort *objectId, Vector3f* table_avg_normal)
{
  int cenIndex = x + y * imgSize.x;

  if(objectId[cenIndex]==1){
    double dotvalue = (normals[cenIndex].x * (*table_avg_normal).x) + (normals[cenIndex].y * (*table_avg_normal).y) + (normals[cenIndex].z * (*table_avg_normal).z);
    if(dotvalue<0.87){
      objectId[cenIndex] = 0;
      colors[cenIndex].x = 255;
      colors[cenIndex].y = 255;
      colors[cenIndex].z = 255;
    }
  }

  if(objectId[cenIndex] != 0){

    int upIndex = cenIndex - imgSize.x;
    int downIndex = cenIndex + imgSize.x;
    int leftIndex = cenIndex - 1;
    int rightIndex = cenIndex + 1;

    if(y - 1 >= 0){

      if(points[upIndex]!=0){
        double dis = sqrt(pow(points[cenIndex].x - points[upIndex].x, 2) + pow(points[cenIndex].y - points[upIndex].y, 2) + pow(points[cenIndex].z - points[upIndex].z, 2));

        //if(count0>=count1){
          if(dis < 0.005){
            if(objectId[cenIndex] == 1){
              double dotvalue = (normals[upIndex].x * (*table_avg_normal).x) + (normals[upIndex].y * (*table_avg_normal).y) + (normals[upIndex].z * (*table_avg_normal).z);
              if(dotvalue>0.87){
                objectId[upIndex] = objectId[cenIndex];
                colors[upIndex] = colors[cenIndex];
              }
            }
          }
        //}
      }
    }

    if(y + 1 < imgSize.y){
      if(points[downIndex]!=0){
        double dis = sqrt(pow(points[cenIndex].x - points[downIndex].x, 2) + pow(points[cenIndex].y - points[downIndex].y, 2) + pow(points[cenIndex].z - points[downIndex].z, 2));

        //if(count0>=count1){
          if(dis < 0.005){
            if(objectId[cenIndex] == 1){
              double dotvalue = (normals[downIndex].x * (*table_avg_normal).x) + (normals[downIndex].y * (*table_avg_normal).y) + (normals[downIndex].z * (*table_avg_normal).z);
              if(dotvalue>0.87){
                objectId[downIndex] = objectId[cenIndex];
                colors[downIndex] = colors[cenIndex];
              }
            }
          }
        //}
      }
    }

    if(x - 1 >= 0){
      if(points[leftIndex]!=0){
        double dis = sqrt(pow(points[cenIndex].x - points[leftIndex].x, 2) + pow(points[cenIndex].y - points[leftIndex].y, 2) + pow(points[cenIndex].z - points[leftIndex].z, 2));

       // if(count0>=count1){
          if(dis < 0.005){
            if(objectId[cenIndex] == 1){
              double dotvalue = (normals[leftIndex].x * (*table_avg_normal).x) + (normals[leftIndex].y * (*table_avg_normal).y) + (normals[leftIndex].z * (*table_avg_normal).z);
              if(dotvalue>0.87){
                objectId[leftIndex] = objectId[cenIndex];
                colors[leftIndex] = colors[cenIndex];
              }
            }
          }
        //}
      }
    }

    if(x + 1 < imgSize.x){
      if(points[rightIndex]!=0){
        double dis = sqrt(pow(points[cenIndex].x - points[rightIndex].x, 2) + pow(points[cenIndex].y - points[rightIndex].y, 2) + pow(points[cenIndex].z - points[rightIndex].z, 2));

        //if(count0>=count1){
          if(dis < 0.005){
            if(objectId[cenIndex] == 1){
              double dotvalue = (normals[rightIndex].x * (*table_avg_normal).x) + (normals[rightIndex].y * (*table_avg_normal).y) + (normals[rightIndex].z * (*table_avg_normal).z);
              if(dotvalue>0.87){
                objectId[rightIndex] = objectId[cenIndex];
                colors[rightIndex] = colors[cenIndex];
              }
            }
          }
        //}
      }
    }
  }
}

//hao modified it
_CPU_AND_GPU_CODE_ inline void genIdMaps2(int x, int y, Vector2i imgSize, float x0, float x1, float y0, float y1, float z, Vector3f *points, Vector3f *normals, Vector3f *colors, ushort *objectId)
{
  int cenIndex = x + y * imgSize.x;

  if(objectId[cenIndex] != 0){
    //test
    //printf("hahahaha\n");

    int upIndex = cenIndex - imgSize.x;
    int downIndex = cenIndex + imgSize.x;
    int leftIndex = cenIndex - 1;
    int rightIndex = cenIndex + 1;

    if(y - 1 >= 0&&points[upIndex].z<z&&points[upIndex].x<x1&&points[upIndex].x>x0&&points[upIndex].y<y1&&points[upIndex].y>y0){
      if(objectId[upIndex]==0&&points[upIndex]!=0){
        double dis = sqrt(pow(points[cenIndex].x - points[upIndex].x, 2) + pow(points[cenIndex].y - points[upIndex].y, 2) + pow(points[cenIndex].z - points[upIndex].z, 2));

        int neighboorX[8];
        int neighboorY[8];
        genNeighborPoints(x, y-1, neighboorX, neighboorY);

        int count0=0;
        int count1=0;

        for(int i=0; i<8; i++){
          if(neighboorX[i]>=0&&neighboorX[i]<imgSize.x&&neighboorY[i]>0&&neighboorY[i]<imgSize.y){
            int tem_index = neighboorX[i] + neighboorY[i] * imgSize.x;; 
            double tem_dis = sqrt(pow(points[tem_index].x - points[upIndex].x, 2) + pow(points[tem_index].y - points[upIndex].y, 2) + pow(points[tem_index].z - points[upIndex].z, 2));

            if(tem_dis < 0.005 && objectId[tem_index] == objectId[cenIndex]){
              count0++;
            }
            else if(tem_dis < 0.005 && objectId[tem_index] != 0){
              count1++;
            }
          }
        }

        if(count0>=count1){
          if(dis < 0.005){
            //if(objectId[cenIndex] != 1){
              objectId[upIndex] = objectId[cenIndex];
              colors[upIndex] = colors[cenIndex];
              //genIdMaps(x, y-1, imgSize, points, colors, objectId);
            //}
          }
        }
      }
    }

    if(y + 1 < imgSize.y&&points[downIndex].z<z&&points[downIndex].x<x1&&points[downIndex].x>x0&&points[downIndex].y<y1&&points[downIndex].y>y0){
      if(objectId[downIndex]==0&&points[downIndex]!=0){
        double dis = sqrt(pow(points[cenIndex].x - points[downIndex].x, 2) + pow(points[cenIndex].y - points[downIndex].y, 2) + pow(points[cenIndex].z - points[downIndex].z, 2));

        int neighboorX[8];
        int neighboorY[8];
        genNeighborPoints(x, y-1, neighboorX, neighboorY);

        int count0=0;
        int count1=0;

        for(int i=0; i<8; i++){
          if(neighboorX[i]>=0&&neighboorX[i]<imgSize.x&&neighboorY[i]>0&&neighboorY[i]<imgSize.y){
            int tem_index = neighboorX[i] + neighboorY[i] * imgSize.x;; 
            double tem_dis = sqrt(pow(points[tem_index].x - points[downIndex].x, 2) + pow(points[tem_index].y - points[downIndex].y, 2) + pow(points[tem_index].z - points[downIndex].z, 2));

            if(tem_dis < 0.005 && objectId[tem_index] == objectId[cenIndex]){
              count0++;
            }
            else if(tem_dis < 0.005 && objectId[tem_index] != 0){
              count1++;
            }
          }
        }

        if(count0>=count1){
          if(dis < 0.005){
            if(objectId[cenIndex] != 1){
              objectId[downIndex] = objectId[cenIndex];
              colors[downIndex] = colors[cenIndex];
            }
          }
        }
      }
    }

    if(x - 1 >= 0&&points[leftIndex].z<z&&points[leftIndex].x<x1&&points[leftIndex].x>x0&&points[leftIndex].y<y1&&points[leftIndex].y>y0){
      if(objectId[leftIndex]==0&&points[leftIndex]!=0){
        double dis = sqrt(pow(points[cenIndex].x - points[leftIndex].x, 2) + pow(points[cenIndex].y - points[leftIndex].y, 2) + pow(points[cenIndex].z - points[leftIndex].z, 2));

        int neighboorX[8];
        int neighboorY[8];
        genNeighborPoints(x, y-1, neighboorX, neighboorY);

        int count0=0;
        int count1=0;

        for(int i=0; i<8; i++){
          if(neighboorX[i]>=0&&neighboorX[i]<imgSize.x&&neighboorY[i]>0&&neighboorY[i]<imgSize.y){
            int tem_index = neighboorX[i] + neighboorY[i] * imgSize.x;; 
            double tem_dis = sqrt(pow(points[tem_index].x - points[leftIndex].x, 2) + pow(points[tem_index].y - points[leftIndex].y, 2) + pow(points[tem_index].z - points[leftIndex].z, 2));

            if(tem_dis < 0.005 && objectId[tem_index] == objectId[cenIndex]){
              count0++;
            }
            else if(tem_dis < 0.005 && objectId[tem_index] != 0){
              count1++;
            }
          }
        }

        if(count0>=count1){
          if(dis < 0.005){
            if(objectId[cenIndex] != 1){
              objectId[leftIndex] = objectId[cenIndex];
              colors[leftIndex] = colors[cenIndex];
            }
          }
        }
      }
    }

    if(x + 1 < imgSize.x&&points[rightIndex].z<z&&points[rightIndex].x<x1&&points[rightIndex].x>x0&&points[rightIndex].y<y1&&points[rightIndex].y>y0){
      if(objectId[rightIndex]==0&&points[rightIndex]!=0){
        double dis = sqrt(pow(points[cenIndex].x - points[rightIndex].x, 2) + pow(points[cenIndex].y - points[rightIndex].y, 2) + pow(points[cenIndex].z - points[rightIndex].z, 2));

        int neighboorX[8];
        int neighboorY[8];
        genNeighborPoints(x, y-1, neighboorX, neighboorY);

        int count0=0;
        int count1=0;

        for(int i=0; i<8; i++){
          if(neighboorX[i]>=0&&neighboorX[i]<imgSize.x&&neighboorY[i]>0&&neighboorY[i]<imgSize.y){
            int tem_index = neighboorX[i] + neighboorY[i] * imgSize.x;; 
            double tem_dis = sqrt(pow(points[tem_index].x - points[rightIndex].x, 2) + pow(points[tem_index].y - points[rightIndex].y, 2) + pow(points[tem_index].z - points[rightIndex].z, 2));

            if(tem_dis < 0.005 && objectId[tem_index] == objectId[cenIndex]){
              count0++;
            }
            else if(tem_dis < 0.005 && objectId[tem_index] != 0){
              count1++;
            }
          }
        }

        if(count0>=count1){
          if(dis < 0.005){
            if(objectId[cenIndex] != 1){
              objectId[rightIndex] = objectId[cenIndex];
              colors[rightIndex] = colors[cenIndex];
            }
          }
        }
      }
    }
  }
}