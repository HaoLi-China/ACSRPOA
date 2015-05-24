// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMMainEngine.h"
#include "../Engine/DeviceAgnostic/ITMSceneReconstructionEngine.h"

#include "ITMTrackerFactory.h"
using namespace ITMLib::Engine;

ITMMainEngine::ITMMainEngine(const ITMLibSettings *settings, const ITMRGBDCalib *calib, Vector2i imgSize_rgb, Vector2i imgSize_d)
{
  if ((imgSize_d.x == -1) || (imgSize_d.y == -1)) imgSize_d = imgSize_rgb;

  this->settings = new ITMLibSettings(*settings);

  this->scene = new ITMScene<ITMVoxel,ITMVoxelIndex>(&(settings->sceneParams), settings->useSwapping, settings->useGPU);

  this->trackingState = ITMTrackerFactory::MakeTrackingState(*settings, imgSize_rgb, imgSize_d);
  trackingState->pose_d->SetFrom(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f); 

  this->view = new ITMView(*calib, imgSize_rgb, imgSize_d, settings->useGPU);

  if (settings->useGPU)
  {
#ifndef COMPILE_WITHOUT_CUDA
    lowLevelEngine = new ITMLowLevelEngine_CUDA();
    sceneRecoEngine = new ITMSceneReconstructionEngine_CUDA<ITMVoxel,ITMVoxelIndex>();
    if (settings->useSwapping) swappingEngine = new ITMSwappingEngine_CUDA<ITMVoxel,ITMVoxelIndex>();
    visualisationEngine = new ITMVisualisationEngine_CUDA<ITMVoxel,ITMVoxelIndex>();
#endif
  }
  else
  {
    lowLevelEngine = new ITMLowLevelEngine_CPU();
    sceneRecoEngine = new ITMSceneReconstructionEngine_CPU<ITMVoxel,ITMVoxelIndex>();
    if (settings->useSwapping) swappingEngine = new ITMSwappingEngine_CPU<ITMVoxel,ITMVoxelIndex>();
    visualisationEngine = new ITMVisualisationEngine_CPU<ITMVoxel,ITMVoxelIndex>();
  }

  trackerPrimary = ITMTrackerFactory::MakePrimaryTracker(*settings, imgSize_rgb, imgSize_d, lowLevelEngine);
  trackerSecondary = ITMTrackerFactory::MakeSecondaryTracker<ITMVoxel,ITMVoxelIndex>(*settings, imgSize_rgb, imgSize_d, lowLevelEngine, scene);

  visualisationState = NULL;

  hasStartedObjectReconstruction = false;
  fusionActive = true;

  idCount = 1;
}

ITMMainEngine::~ITMMainEngine()
{
  delete sceneRecoEngine;
  if (trackerPrimary != NULL) delete trackerPrimary;
  if (trackerSecondary != NULL) delete trackerSecondary;
  delete visualisationEngine;
  delete lowLevelEngine;

  if (settings->useSwapping) delete swappingEngine;

  delete trackingState;
  delete scene;
  delete view;

  delete settings;
}

//hao modified it
void ITMMainEngine::ProcessFrame(short segFlag)
{
  bool useGPU = settings->useGPU;
  bool useSwapping = settings->useSwapping;

  // prepare image and move it to GPU, if required
  if (useGPU)
  {
    view->rgb->UpdateDeviceFromHost();

    switch (view->inputImageType)
    {
    case ITMView::InfiniTAM_FLOAT_DEPTH_IMAGE: view->depth->UpdateDeviceFromHost(); break;
    case ITMView::InfiniTAM_SHORT_DEPTH_IMAGE:
    case ITMView::InfiniTAM_DISPARITY_IMAGE: view->rawDepth->UpdateDeviceFromHost(); break;
    }
  }

  // prepare image and turn it into a depth image
  if (view->inputImageType == ITMView::InfiniTAM_DISPARITY_IMAGE)
  {
    lowLevelEngine->ConvertDisparityToDepth(view->depth, view->rawDepth, &(view->calib->intrinsics_d), &(view->calib->disparityCalib));
  }
  else if (view->inputImageType == ITMView::InfiniTAM_SHORT_DEPTH_IMAGE)
  {
    lowLevelEngine->ConvertDepthMMToFloat(view->depth, view->rawDepth);
  }

  // tracking
  if (hasStartedObjectReconstruction)
  {
    if (trackerPrimary != NULL) trackerPrimary->TrackCamera(trackingState, view);
    if (trackerSecondary != NULL) trackerSecondary->TrackCamera(trackingState, view);

  }

  // allocation
  sceneRecoEngine->AllocateSceneFromDepth(scene, view, trackingState->pose_d);

  // integration
  if (fusionActive) sceneRecoEngine->IntegrateIntoScene(scene, view, trackingState->pose_d);


  if (useSwapping) {
    // swapping: CPU -> GPU
    swappingEngine->IntegrateGlobalIntoLocal(scene, view);
    // swapping: GPU -> CPU
    swappingEngine->SaveToGlobalMemory(scene, view);
  }

  switch (settings->trackerType)
  {
  case ITMLibSettings::TRACKER_ICP:
  case ITMLibSettings::TRACKER_REN:
    // raycasting
    visualisationEngine->CreateExpectedDepths(scene, trackingState->pose_d, &(view->calib->intrinsics_d), trackingState->renderingRangeImage);

    if(segFlag==0){
      visualisationEngine->CreateICPMaps(scene, view, trackingState);
    }
    else if(segFlag==1){
      overSegmentView();
    }
    else if(segFlag==2){
      segmentView();
    }
    else if(segFlag==3){
      interactedSegment();
    }

    break;
  case ITMLibSettings::TRACKER_COLOR:
    // raycasting
    ITMPose pose_rgb(view->calib->trafo_rgb_to_depth.calib_inv * trackingState->pose_d->M);
    visualisationEngine->CreateExpectedDepths(scene, &pose_rgb, &(view->calib->intrinsics_rgb), trackingState->renderingRangeImage);
    visualisationEngine->CreatePointCloud(scene, view, trackingState, settings->skipPoints);
    break;
  }

  hasStartedObjectReconstruction = true;
}

void ITMMainEngine::GetImage(ITMUChar4Image *out, GetImageType getImageType, bool useColour, ITMPose *pose, ITMIntrinsics *intrinsics)
{
  out->Clear();

  switch (getImageType)
  {
  case ITMMainEngine::InfiniTAM_IMAGE_ORIGINAL_RGB:
    if (settings->useGPU) view->rgb->UpdateHostFromDevice();
    out->ChangeDims(view->rgb->noDims);
    out->SetFrom(view->rgb);
    break;
  case ITMMainEngine::InfiniTAM_IMAGE_ORIGINAL_DEPTH:
    if (settings->useGPU) view->depth->UpdateHostFromDevice();
    out->ChangeDims(view->depth->noDims);
    ITMVisualisationEngine<ITMVoxel,ITMVoxelIndex>::DepthToUchar4(out, view->depth);
    break;
  case ITMMainEngine::InfiniTAM_IMAGE_SCENERAYCAST:
    if (settings->useGPU) trackingState->rendering->UpdateHostFromDevice();
    out->ChangeDims(trackingState->rendering->noDims);
    out->SetFrom(trackingState->rendering);
    break;
  case ITMMainEngine::InfiniTAM_IMAGE_SCENERAYCAST_FREECAMERA:
    if (pose != NULL && intrinsics != NULL)
    {
      if (visualisationState == NULL) visualisationState = visualisationEngine->allocateInternalState(out->noDims);

      visualisationEngine->FindVisibleBlocks(scene, pose, intrinsics, visualisationState);
      visualisationEngine->CreateExpectedDepths(scene, pose, intrinsics, visualisationState->minmaxImage, visualisationState);
      visualisationEngine->RenderImage(scene, pose, intrinsics, visualisationState, visualisationState->outputImage, useColour);

      if (settings->useGPU) visualisationState->outputImage->UpdateHostFromDevice();
      out->SetFrom(visualisationState->outputImage);
    }
    break;
  };
}

//hao modified it
void ITMMainEngine::GetDepthAndColorImageForPause(ITMUChar4Image *out, GetImageType getImageType, bool useColour, ITMPose *pose, ITMIntrinsics *intrinsics)
{
  out->Clear();

  switch (getImageType)
  {
  case ITMMainEngine::InfiniTAM_IMAGE_ORIGINAL_RGB:
    out->ChangeDims(view->rgb->noDims);
    out->SetFrom(view->rgb);
    break;
  case ITMMainEngine::InfiniTAM_IMAGE_ORIGINAL_DEPTH:
    out->ChangeDims(view->depth->noDims);
    ITMVisualisationEngine<ITMVoxel,ITMVoxelIndex>::DepthToUchar4(out, view->depth);
    break;
  case ITMMainEngine::InfiniTAM_IMAGE_SCENERAYCAST:
    if (settings->useGPU) trackingState->rendering->UpdateHostFromDevice();
    out->ChangeDims(trackingState->rendering->noDims);
    out->SetFrom(trackingState->rendering);
    break;
  case ITMMainEngine::InfiniTAM_IMAGE_SCENERAYCAST_FREECAMERA:
    if (pose != NULL && intrinsics != NULL)
    {
      if (visualisationState == NULL) visualisationState = visualisationEngine->allocateInternalState(out->noDims);

      visualisationEngine->FindVisibleBlocks(scene, pose, intrinsics, visualisationState);
      visualisationEngine->CreateExpectedDepths(scene, pose, intrinsics, visualisationState->minmaxImage, visualisationState);
      visualisationEngine->RenderImage(scene, pose, intrinsics, visualisationState, visualisationState->outputImage, useColour);

      if (settings->useGPU) visualisationState->outputImage->UpdateHostFromDevice();
      out->SetFrom(visualisationState->outputImage);
    }
    break;
  };
}

void ITMMainEngine::turnOnIntegration()
{
  fusionActive = true;
}

void ITMMainEngine::turnOffIntegration()
{
  fusionActive = false;
}

//hao modified it
void ITMMainEngine::overSegmentView()
{
  {
#ifndef COMPILE_WITHOUT_CUDA
    cout<<"view->depth->noDims.x:"<<view->depth->noDims.x<<endl;
    cout<<"view->depth->noDims.y:"<<view->depth->noDims.y<<endl;
    Vector3f *points_device;
    Vector3f *points_host = (Vector3f*)malloc(view->depth->noDims.x*view->depth->noDims.y*sizeof(Vector3f));
    Vector3f *normals_device;
    Vector3f *normals_host = (Vector3f*)malloc(view->depth->noDims.x*view->depth->noDims.y*sizeof(Vector3f));
    ITMSafeCall(cudaMalloc((void**)&points_device, view->depth->noDims.x*view->depth->noDims.y*sizeof(Vector3f)));
    ITMSafeCall(cudaMemset(points_device, 0, view->depth->noDims.x*view->depth->noDims.y*sizeof(Vector3f)));
    ITMSafeCall(cudaMalloc((void**)&normals_device, view->depth->noDims.x*view->depth->noDims.y*sizeof(Vector3f)));
    ITMSafeCall(cudaMemset(normals_device, 0, view->depth->noDims.x*view->depth->noDims.y*sizeof(Vector3f)));
    visualisationEngine->GetRaycastImage(scene, view, trackingState, points_device, normals_device, NULL, NULL);
    ITMSafeCall(cudaMemcpy(points_host, points_device, view->depth->noDims.x*view->depth->noDims.y*sizeof(Vector3f), cudaMemcpyDeviceToHost));
    ITMSafeCall(cudaMemcpy(normals_host, normals_device, view->depth->noDims.x*view->depth->noDims.y*sizeof(Vector3f), cudaMemcpyDeviceToHost));

    int *id_array_host = (int*)malloc(view->depth->noDims.x*view->depth->noDims.y*sizeof(int));
    PointCloudPtr_RGB_NORMAL cloud(new PointCloud_RGB_NORMAL);

    for(int i=0; i<view->depth->noDims.x*view->depth->noDims.y; i++){
      if(!((*(points_host+i))[0]==0&&(*(points_host+i))[1]==0&&(*(points_host+i))[2]==0)){
        Point_RGB_NORMAL pt;
        pt.x=(*(points_host+i))[0];
        pt.y=(*(points_host+i))[1];
        pt.z=(*(points_host+i))[2];
        pt.r=255;
        pt.g=255;
        pt.b=255;
        pt.normal_x=(*(normals_host+i))[0];
        pt.normal_y=(*(normals_host+i))[1];
        pt.normal_z=(*(normals_host+i))[2];
        cloud->points.push_back(pt);
        *(id_array_host+i)=i;
      }else{
        *(id_array_host+i)=-1;
      }
    }

    PointCloudPtr_RGB segmented_cloud(new PointCloud_RGB);
    overSegmentObject(cloud, segmented_cloud);
    //showPointCloud(segmented_cloud, "segmented_cloud");

    KDtree tree;
    CUDA_KDTree GPU_tree;
    int max_tree_levels = 13; // play around with this value to get the best result

    vector<KDPoint> data(cloud->size());
    vector<KDPoint> queries(segmented_cloud->size());

    for(int i=0; i<cloud->size(); i++){
      data[i].coords[0] = cloud->points[i].x;
      data[i].coords[1] = cloud->points[i].y;
      data[i].coords[2] = cloud->points[i].z;
    }

    for(int i=0; i<segmented_cloud->size(); i++){
      queries[i].coords[0] = segmented_cloud->points[i].x;
      queries[i].coords[1] = segmented_cloud->points[i].y;
      queries[i].coords[2] = segmented_cloud->points[i].z;
    }

    vector <int> gpu_indexes;
    vector <float> gpu_dists;

    tree.Create(data, max_tree_levels);
    GPU_tree.CreateKDTree(tree.GetRoot(), tree.GetNumNodes(), data);
    GPU_tree.Search(queries, gpu_indexes, gpu_dists);

    PointCloudPtr_RGB colored_cloud(new PointCloud_RGB);
    for(int i=0; i<cloud->size(); i++){
      Point_RGB p_tem;
      p_tem.x=0;
      p_tem.y=0;
      p_tem.z=0;
      colored_cloud->push_back(p_tem);
    }

    for(int i=0; i<segmented_cloud->size(); i++){
      colored_cloud->points[gpu_indexes[i]].x = segmented_cloud->points[i].x;
      colored_cloud->points[gpu_indexes[i]].y = segmented_cloud->points[i].y;
      colored_cloud->points[gpu_indexes[i]].z = segmented_cloud->points[i].z;
      colored_cloud->points[gpu_indexes[i]].r = segmented_cloud->points[i].r;
      colored_cloud->points[gpu_indexes[i]].g = segmented_cloud->points[i].g;
      colored_cloud->points[gpu_indexes[i]].b = segmented_cloud->points[i].b;
    }

    //showPointCloud(colored_cloud, "colored_cloud");

    Vector3f *colors_host = (Vector3f*)malloc(view->depth->noDims.x*view->depth->noDims.y*sizeof(Vector3f));

    int count=0;
    for(int i=0; i<view->depth->noDims.x*view->depth->noDims.y; i++){
      if(*(id_array_host+i)!=-1){
        if(!(colored_cloud->points[count].x==0&&colored_cloud->points[count].y==0&&colored_cloud->points[count].z==0)){
          (*(colors_host+i))[0] = colored_cloud->points[count].r;
          (*(colors_host+i))[1] = colored_cloud->points[count].g;
          (*(colors_host+i))[2] = colored_cloud->points[count].b;
        }
        else{
          (*(colors_host+i))[0] = -1;
          (*(colors_host+i))[1] = -1;
          (*(colors_host+i))[2] = -1;
        }

        count++;
      }
      else{
        (*(colors_host+i))[0] = -1;
        (*(colors_host+i))[1] = -1;
        (*(colors_host+i))[2] = -1;
      }
    }

    Vector3f *colors_device;
    ITMSafeCall(cudaMalloc((void**)&colors_device, view->depth->noDims.x*view->depth->noDims.y*sizeof(Vector3f)));
    ITMSafeCall(cudaMemcpy(colors_device, colors_host, view->depth->noDims.x*view->depth->noDims.y*sizeof(Vector3f), cudaMemcpyHostToDevice));
    //pcl::io::savePLYFileBinary("Data/scan1.ply", *colored_cloud);

    visualisationEngine->NewCreateICPMaps(scene, view, trackingState, colors_device, NULL, 0);

    free(points_host);
    free(normals_host);
    free(colors_host);
    free(id_array_host);
    points_host=NULL;
    normals_host=NULL;
    colors_host=NULL;
    id_array_host=NULL;

    ITMSafeCall(cudaFree(points_device));
    ITMSafeCall(cudaFree(normals_device));
    ITMSafeCall(cudaFree(colors_device));
#endif
  }
}

//hao modified it
void ITMMainEngine::segmentView()
{
  {
#ifndef COMPILE_WITHOUT_CUDA
    Vector3f *points_device;
    Vector3f *points_host = (Vector3f*)malloc(view->depth->noDims.x*view->depth->noDims.y*sizeof(Vector3f));
    Vector3f *normals_device;
    Vector3f *normals_host = (Vector3f*)malloc(view->depth->noDims.x*view->depth->noDims.y*sizeof(Vector3f));
    ITMSafeCall(cudaMalloc((void**)&points_device, view->depth->noDims.x*view->depth->noDims.y*sizeof(Vector3f)));
    ITMSafeCall(cudaMemset(points_device, 0, view->depth->noDims.x*view->depth->noDims.y*sizeof(Vector3f)));
    ITMSafeCall(cudaMalloc((void**)&normals_device, view->depth->noDims.x*view->depth->noDims.y*sizeof(Vector3f)));
    ITMSafeCall(cudaMemset(normals_device, 0, view->depth->noDims.x*view->depth->noDims.y*sizeof(Vector3f)));
    visualisationEngine->GetRaycastImage(scene, view, trackingState, points_device, normals_device, NULL, NULL);
    ITMSafeCall(cudaMemcpy(points_host, points_device, view->depth->noDims.x*view->depth->noDims.y*sizeof(Vector3f), cudaMemcpyDeviceToHost));
    ITMSafeCall(cudaMemcpy(normals_host, normals_device, view->depth->noDims.x*view->depth->noDims.y*sizeof(Vector3f), cudaMemcpyDeviceToHost));

    int *id_array_host = (int*)malloc(view->depth->noDims.x*view->depth->noDims.y*sizeof(int));
    PointCloudPtr_RGB_NORMAL cloud(new PointCloud_RGB_NORMAL);

    for(int i=0; i<view->depth->noDims.x*view->depth->noDims.y; i++){
      if(!((*(points_host+i))[0]==0&&(*(points_host+i))[1]==0&&(*(points_host+i))[2]==0)){
        Point_RGB_NORMAL pt;
        pt.x=(*(points_host+i))[0];
        pt.y=(*(points_host+i))[1];
        pt.z=(*(points_host+i))[2];
        pt.r=255;
        pt.g=255;
        pt.b=255;
        pt.normal_x=(*(normals_host+i))[0];
        pt.normal_y=(*(normals_host+i))[1];
        pt.normal_z=(*(normals_host+i))[2];
        cloud->points.push_back(pt);
        *(id_array_host+i)=i;
      }else{
        *(id_array_host+i)=-1;
      }
    }

    //currentViewCloud.mypoints.clear();
    //PointCloud_RGB_NORMAL2MyPointCloud_RGB_NORMAL(cloud, currentViewCloud);

    PointCloudPtr_RGB object_cloud(new PointCloud_RGB);
    PointCloudPtr_RGB confidence_cloud(new PointCloud_RGB);
    vector<ushort> objectIndexs;
    Eigen::Vector3f range(0.5, 0.5, 1.0);
    int objectNum;
    vector<ObjectAttri> obas;
    segmentObject(cloud, range, obas, object_cloud, confidence_cloud, objectIndexs, objectNum);
    //showPointCloud(segmented_cloud, "segmented_cloud");

    for(int i=0; i<obas.size(); i++){
      objectMap[idCount+i+1] = obas[i];
    }

    KDtree tree;
    CUDA_KDTree GPU_tree;
    int max_tree_levels = 13; // play around with this value to get the best result

    vector<KDPoint> data(cloud->size());
    vector<KDPoint> queries(object_cloud->size());

    for(int i=0; i<cloud->size(); i++){
      data[i].coords[0] = cloud->points[i].x;
      data[i].coords[1] = cloud->points[i].y;
      data[i].coords[2] = cloud->points[i].z;
    }

    for(int i=0; i<object_cloud->size(); i++){
      queries[i].coords[0] = object_cloud->points[i].x;
      queries[i].coords[1] = object_cloud->points[i].y;
      queries[i].coords[2] = object_cloud->points[i].z;
    }

    vector <int> gpu_indexes;
    vector <float> gpu_dists;

    tree.Create(data, max_tree_levels);
    GPU_tree.CreateKDTree(tree.GetRoot(), tree.GetNumNodes(), data);
    GPU_tree.Search(queries, gpu_indexes, gpu_dists);

    PointCloudPtr_RGB colored_cloud0(new PointCloud_RGB);
    PointCloudPtr_RGB colored_cloud1(new PointCloud_RGB);
    vector<ushort> objectIndexs_new;
    for(int i=0; i<cloud->size(); i++){
      Point_RGB p_tem;
      p_tem.x=0;
      p_tem.y=0;
      p_tem.z=0;
      colored_cloud0->push_back(p_tem);
      colored_cloud1->push_back(p_tem);
      objectIndexs_new.push_back(9999);
    }

    for(int i=0; i<object_cloud->size(); i++){
      colored_cloud0->points[gpu_indexes[i]].x = object_cloud->points[i].x;
      colored_cloud0->points[gpu_indexes[i]].y = object_cloud->points[i].y;
      colored_cloud0->points[gpu_indexes[i]].z = object_cloud->points[i].z;
      colored_cloud0->points[gpu_indexes[i]].r = object_cloud->points[i].r;
      colored_cloud0->points[gpu_indexes[i]].g = object_cloud->points[i].g;
      colored_cloud0->points[gpu_indexes[i]].b = object_cloud->points[i].b;

      colored_cloud1->points[gpu_indexes[i]].x = confidence_cloud->points[i].x;
      colored_cloud1->points[gpu_indexes[i]].y = confidence_cloud->points[i].y;
      colored_cloud1->points[gpu_indexes[i]].z = confidence_cloud->points[i].z;
      colored_cloud1->points[gpu_indexes[i]].r = confidence_cloud->points[i].r;
      colored_cloud1->points[gpu_indexes[i]].g = confidence_cloud->points[i].g;
      colored_cloud1->points[gpu_indexes[i]].b = confidence_cloud->points[i].b;

      objectIndexs_new[gpu_indexes[i]] = objectIndexs[i];
    }

    //showPointCloud(colored_cloud, "colored_cloud");

    Vector3f *objectColors = (Vector3f*)malloc(view->depth->noDims.x*view->depth->noDims.y*sizeof(Vector3f));
    ushort *objectIds_host = (ushort*)malloc(view->depth->noDims.x*view->depth->noDims.y*sizeof(ushort));
    memset(objectIds_host, 0, view->depth->noDims.x*view->depth->noDims.y*sizeof(ushort));

    int count=0;
    for(int i=0; i<view->depth->noDims.x*view->depth->noDims.y; i++){
      if(*(id_array_host+i)!=-1){
        if(!(colored_cloud0->points[count].x==0&&colored_cloud0->points[count].y==0&&colored_cloud0->points[count].z==0)){
          (*(objectColors+i))[0] = colored_cloud0->points[count].r;
          (*(objectColors+i))[1] = colored_cloud0->points[count].g;
          (*(objectColors+i))[2] = colored_cloud0->points[count].b;

          if(objectIndexs_new[count] == 0){
            (*(objectIds_host+i)) = 1;
          }else{
            (*(objectIds_host+i)) = idCount + objectIndexs_new[count];
          }
        }
        else{
          (*(objectColors+i))[0] = -1;
          (*(objectColors+i))[1] = -1;
          (*(objectColors+i))[2] = -1;
        }

        count++;
      }
      else{
        (*(objectColors+i))[0] = -1;
        (*(objectColors+i))[1] = -1;
        (*(objectColors+i))[2] = -1;
      }
    }

    idCount += objectNum;

    Vector3f *colors_device;
    ITMSafeCall(cudaMalloc((void**)&colors_device, view->depth->noDims.x*view->depth->noDims.y*sizeof(Vector3f)));
    ITMSafeCall(cudaMemcpy(colors_device, objectColors, view->depth->noDims.x*view->depth->noDims.y*sizeof(Vector3f), cudaMemcpyHostToDevice));
    //pcl::io::savePLYFileBinary("Data/scan1.ply", *colored_cloud);

    ushort *objectIds_device;
    ITMSafeCall(cudaMalloc((void**)&objectIds_device, view->depth->noDims.x*view->depth->noDims.y*sizeof(ushort)));
    ITMSafeCall(cudaMemcpy(objectIds_device, objectIds_host, view->depth->noDims.x*view->depth->noDims.y*sizeof(ushort), cudaMemcpyHostToDevice));

    visualisationEngine->NewCreateICPMaps(scene, view, trackingState, colors_device, objectIds_device, 1);

    free(points_host);
    free(normals_host);
    free(objectIds_host);
    free(id_array_host);
    free(objectColors);
    points_host=NULL;
    normals_host=NULL;
    objectIds_host=NULL;
    id_array_host=NULL;
    objectColors=NULL;

    ITMSafeCall(cudaFree(points_device));
    ITMSafeCall(cudaFree(normals_device));
    ITMSafeCall(cudaFree(colors_device));
    ITMSafeCall(cudaFree(objectIds_device));
#endif
  }
}

//hao modified it
void ITMMainEngine::saveViewPoints(){
  {
    trackingStateTem=trackingState;
#ifndef COMPILE_WITHOUT_CUDA
    Vector3f *points_device;
    Vector3f *points_host = (Vector3f*)malloc(view->depth->noDims.x*view->depth->noDims.y*sizeof(Vector3f));
    Vector3f *normals_device;
    Vector3f *normals_host = (Vector3f*)malloc(view->depth->noDims.x*view->depth->noDims.y*sizeof(Vector3f));
    ITMSafeCall(cudaMalloc((void**)&points_device, view->depth->noDims.x*view->depth->noDims.y*sizeof(Vector3f)));
    ITMSafeCall(cudaMemset(points_device, 0, view->depth->noDims.x*view->depth->noDims.y*sizeof(Vector3f)));
    ITMSafeCall(cudaMalloc((void**)&normals_device, view->depth->noDims.x*view->depth->noDims.y*sizeof(Vector3f)));
    ITMSafeCall(cudaMemset(normals_device, 0, view->depth->noDims.x*view->depth->noDims.y*sizeof(Vector3f)));
    visualisationEngine->GetRaycastImage(scene, view, trackingState, points_device, normals_device, NULL, NULL);
    ITMSafeCall(cudaMemcpy(points_host, points_device, view->depth->noDims.x*view->depth->noDims.y*sizeof(Vector3f), cudaMemcpyDeviceToHost));
    ITMSafeCall(cudaMemcpy(normals_host, normals_device, view->depth->noDims.x*view->depth->noDims.y*sizeof(Vector3f), cudaMemcpyDeviceToHost));

    PointCloudPtr_RGB_NORMAL cloud(new PointCloud_RGB_NORMAL);

    for(int i=0; i<view->depth->noDims.x*view->depth->noDims.y; i++){
      if(!((*(points_host+i))[0]==0&&(*(points_host+i))[1]==0&&(*(points_host+i))[2]==0)){
        Point_RGB_NORMAL pt;
        pt.x=(*(points_host+i))[0];
        pt.y=(*(points_host+i))[1];
        pt.z=(*(points_host+i))[2];
        pt.r=255;
        pt.g=255;
        pt.b=255;
        pt.normal_x=(*(normals_host+i))[0];
        pt.normal_y=(*(normals_host+i))[1];
        pt.normal_z=(*(normals_host+i))[2];
        cloud->points.push_back(pt);
      }
    }

    myCloudOne.mypoints.clear();
    PointCloud_RGB_NORMAL2MyPointCloud_RGB_NORMAL(cloud, myCloudOne);
    pcl::io::savePLYFileASCII("Data/currentView0.ply", *cloud);

    free(points_host);
    free(normals_host);
    points_host=NULL;
    normals_host=NULL;

    ITMSafeCall(cudaFree(points_device));
    ITMSafeCall(cudaFree(normals_device));
#endif
  }
}

//hao modified it
void ITMMainEngine::saveViewPoints(ITMTrackingState *itmtrackingState){
  {
#ifndef COMPILE_WITHOUT_CUDA
    Vector3f *points_device;
    Vector3f *points_host = (Vector3f*)malloc(view->depth->noDims.x*view->depth->noDims.y*sizeof(Vector3f));
    Vector3f *normals_device;
    Vector3f *normals_host = (Vector3f*)malloc(view->depth->noDims.x*view->depth->noDims.y*sizeof(Vector3f));
    ITMSafeCall(cudaMalloc((void**)&points_device, view->depth->noDims.x*view->depth->noDims.y*sizeof(Vector3f)));
    ITMSafeCall(cudaMemset(points_device, 0, view->depth->noDims.x*view->depth->noDims.y*sizeof(Vector3f)));
    ITMSafeCall(cudaMalloc((void**)&normals_device, view->depth->noDims.x*view->depth->noDims.y*sizeof(Vector3f)));
    ITMSafeCall(cudaMemset(normals_device, 0, view->depth->noDims.x*view->depth->noDims.y*sizeof(Vector3f)));
    visualisationEngine->GetRaycastImage(scene, view, trackingState, points_device, normals_device, NULL, NULL);
    ITMSafeCall(cudaMemcpy(points_host, points_device, view->depth->noDims.x*view->depth->noDims.y*sizeof(Vector3f), cudaMemcpyDeviceToHost));
    ITMSafeCall(cudaMemcpy(normals_host, normals_device, view->depth->noDims.x*view->depth->noDims.y*sizeof(Vector3f), cudaMemcpyDeviceToHost));

    PointCloudPtr_RGB_NORMAL cloud(new PointCloud_RGB_NORMAL);

    for(int i=0; i<view->depth->noDims.x*view->depth->noDims.y; i++){
      if(!((*(points_host+i))[0]==0&&(*(points_host+i))[1]==0&&(*(points_host+i))[2]==0)){
        Point_RGB_NORMAL pt;
        pt.x=(*(points_host+i))[0];
        pt.y=(*(points_host+i))[1];
        pt.z=(*(points_host+i))[2];
        pt.r=255;
        pt.g=255;
        pt.b=255;
        pt.normal_x=(*(normals_host+i))[0];
        pt.normal_y=(*(normals_host+i))[1];
        pt.normal_z=(*(normals_host+i))[2];
        cloud->points.push_back(pt);
      }
    }

    myCloudTwo.mypoints.clear();
    PointCloud_RGB_NORMAL2MyPointCloud_RGB_NORMAL(cloud, myCloudTwo);
    pcl::io::savePLYFileASCII("Data/currentView1.ply", *cloud);

    free(points_host);
    free(normals_host);
    points_host=NULL;
    normals_host=NULL;

    ITMSafeCall(cudaFree(points_device));
    ITMSafeCall(cudaFree(normals_device));
#endif
  }
}

//hao modified it
void ITMMainEngine::detectChange(){
  {
#ifndef COMPILE_WITHOUT_CUDA
    Vector3f *points_device;
    Vector3f *points_host = (Vector3f*)malloc(view->depth->noDims.x*view->depth->noDims.y*sizeof(Vector3f));
    Vector3f *normals_device;
    Vector3f *normals_host = (Vector3f*)malloc(view->depth->noDims.x*view->depth->noDims.y*sizeof(Vector3f));
    ITMSafeCall(cudaMalloc((void**)&points_device, view->depth->noDims.x*view->depth->noDims.y*sizeof(Vector3f)));
    ITMSafeCall(cudaMemset(points_device, 0, view->depth->noDims.x*view->depth->noDims.y*sizeof(Vector3f)));
    ITMSafeCall(cudaMalloc((void**)&normals_device, view->depth->noDims.x*view->depth->noDims.y*sizeof(Vector3f)));
    ITMSafeCall(cudaMemset(normals_device, 0, view->depth->noDims.x*view->depth->noDims.y*sizeof(Vector3f)));
    visualisationEngine->GetRaycastImage(scene, view, trackingState, points_device, normals_device, NULL, NULL);
    ITMSafeCall(cudaMemcpy(points_host, points_device, view->depth->noDims.x*view->depth->noDims.y*sizeof(Vector3f), cudaMemcpyDeviceToHost));
    ITMSafeCall(cudaMemcpy(normals_host, normals_device, view->depth->noDims.x*view->depth->noDims.y*sizeof(Vector3f), cudaMemcpyDeviceToHost));

    int *id_array_host = (int*)malloc(view->depth->noDims.x*view->depth->noDims.y*sizeof(int));
    PointCloudPtr_RGB_NORMAL cloud(new PointCloud_RGB_NORMAL);

    for(int i=0; i<view->depth->noDims.x*view->depth->noDims.y; i++){
      if(!((*(points_host+i))[0]==0&&(*(points_host+i))[1]==0&&(*(points_host+i))[2]==0)){
        Point_RGB_NORMAL pt;
        pt.x=(*(points_host+i))[0];
        pt.y=(*(points_host+i))[1];
        pt.z=(*(points_host+i))[2];
        pt.normal_x=(*(normals_host+i))[0];
        pt.normal_y=(*(normals_host+i))[1];
        pt.normal_z=(*(normals_host+i))[2];
        cloud->points.push_back(pt);
        *(id_array_host+i)=i;
      }else{
        *(id_array_host+i)=-1;
      }
    }

    PointCloudPtr_RGB_NORMAL cloudA (new PointCloud_RGB_NORMAL);
    PointCloudPtr_RGB_NORMAL cloudB (new PointCloud_RGB_NORMAL);

    MyPointCloud_RGB_NORMAL2PointCloud_RGB_NORMAL(myCloudOne, cloudA);
    MyPointCloud_RGB_NORMAL2PointCloud_RGB_NORMAL(myCloudTwo, cloudB);

    //showPointCloud(cloudA, "cloudA");
    //showPointCloud(cloudB, "cloudB");

    PointCloudPtr_RGB_NORMAL planeCloudA(new PointCloud_RGB_NORMAL);
    PointCloudPtr rect_cloudA(new PointCloud());
    PointCloudPtr_RGB_NORMAL remainingCloudA(new PointCloud_RGB_NORMAL);
    pcl::ModelCoefficients coefficientsA;
    PointCloudPtr_RGB_NORMAL planeCloudB(new PointCloud_RGB_NORMAL);
    PointCloudPtr rect_cloudB(new PointCloud());
    PointCloudPtr_RGB_NORMAL remainingCloudB(new PointCloud_RGB_NORMAL);
    pcl::ModelCoefficients coefficientsB;
    detect_table(cloudA, coefficientsA, planeCloudA, rect_cloudA, remainingCloudA);
    detect_table(cloudB, coefficientsB, planeCloudB, rect_cloudB, remainingCloudB);

    /* showPointCloud(planeCloudA, "planeCloudA");
    showPointCloud(remainingCloudA, "remainingCloudA");
    showPointCloud(planeCloudB, "planeCloudB");
    showPointCloud(remainingCloudB, "remainingCloudB");*/

    Eigen::Matrix4f matrix_transformA;
    Eigen::Matrix4f matrix_transformA_r;
    Eigen::Matrix4f matrix_transformB;
    Eigen::Matrix4f matrix_transformB_r;

    getTemTransformMatrix(coefficientsA, matrix_transformA, matrix_transformA_r);
    getTemTransformMatrix(coefficientsB, matrix_transformB, matrix_transformB_r);

    PointCloudPtr_RGB_NORMAL tabletopCloudA_tem(new PointCloud_RGB_NORMAL());
    PointCloudPtr_RGB_NORMAL tabletopCloudB_tem(new PointCloud_RGB_NORMAL());
    getCloudOnTable(remainingCloudA, rect_cloudA, matrix_transformA, matrix_transformA_r, tabletopCloudA_tem);
    getCloudOnTable(remainingCloudB, rect_cloudB, matrix_transformB, matrix_transformB_r, tabletopCloudB_tem);

    PointCloudPtr_RGB_NORMAL tabletopCloudA(new PointCloud_RGB_NORMAL());
    PointCloudPtr_RGB_NORMAL tabletopCloudB(new PointCloud_RGB_NORMAL());
    for(int i=0; i<tabletopCloudA_tem->size(); i++){
      if(!(tabletopCloudA_tem->points[i].z>1||tabletopCloudA_tem->points[i].x<-0.5||tabletopCloudA_tem->points[i].x>0.5||tabletopCloudA_tem->points[i].y<-0.5||tabletopCloudA_tem->points[i].y>0.5)){
        tabletopCloudA->push_back(tabletopCloudA_tem->points[i]);
      }
    }

    for(int i=0; i<tabletopCloudB_tem->size(); i++){
      if(!(tabletopCloudB_tem->points[i].z>1||tabletopCloudB_tem->points[i].x<-0.5||tabletopCloudB_tem->points[i].x>0.5||tabletopCloudB_tem->points[i].y<-0.5||tabletopCloudB_tem->points[i].y>0.5)){
        tabletopCloudB->push_back(tabletopCloudB_tem->points[i]);
      }
    }
    //showPointCloud(tabletopCloudA, "tabletopCloudA");
    //showPointCloud(tabletopCloudB, "tabletopCloudB");

    PointCloudPtr_RGB_NORMAL resultA (new PointCloud_RGB_NORMAL);
    PointCloudPtr_RGB_NORMAL resultB (new PointCloud_RGB_NORMAL);
    detect_change(tabletopCloudA, tabletopCloudB, resultA, resultB);

    //showPointCloud3(resultA, "resultA");
    //showPointCloud3(resultB, "resultB");

    KDtree tree;
    CUDA_KDTree GPU_tree;
    int max_tree_levels = 13; // play around with this value to get the best result

    vector<KDPoint> data(cloud->size());
    vector<KDPoint> queries(resultB->size());

    for(int i=0; i<cloud->size(); i++){
      data[i].coords[0] = cloud->points[i].x;
      data[i].coords[1] = cloud->points[i].y;
      data[i].coords[2] = cloud->points[i].z;
    }

    for(int i=0; i<resultB->size(); i++){
      queries[i].coords[0] = resultB->points[i].x;
      queries[i].coords[1] = resultB->points[i].y;
      queries[i].coords[2] = resultB->points[i].z;
    }

    vector <int> gpu_indexes;
    vector <float> gpu_dists;

    tree.Create(data, max_tree_levels);
    GPU_tree.CreateKDTree(tree.GetRoot(), tree.GetNumNodes(), data);
    GPU_tree.Search(queries, gpu_indexes, gpu_dists);

    PointCloudPtr_RGB colored_cloud(new PointCloud_RGB);
    for(int i=0; i<cloud->size(); i++){
      Point_RGB p_tem;
      p_tem.x=0;
      p_tem.y=0;
      p_tem.z=0;
      colored_cloud->push_back(p_tem);
    }

    for(int i=0; i<resultB->size(); i++){
      colored_cloud->points[gpu_indexes[i]].x = resultB->points[i].x;
      colored_cloud->points[gpu_indexes[i]].y = resultB->points[i].y;
      colored_cloud->points[gpu_indexes[i]].z = resultB->points[i].z;
      colored_cloud->points[gpu_indexes[i]].r = 255;
      colored_cloud->points[gpu_indexes[i]].g = 0;
      colored_cloud->points[gpu_indexes[i]].b = 0;
    }

    Vector3f *colors_host = (Vector3f*)malloc(view->depth->noDims.x*view->depth->noDims.y*sizeof(Vector3f));

    int count=0;
    for(int i=0; i<view->depth->noDims.x*view->depth->noDims.y; i++){
      if(*(id_array_host+i)!=-1){
        if(!(colored_cloud->points[count].x==0&&colored_cloud->points[count].y==0&&colored_cloud->points[count].z==0)){
          (*(colors_host+i))[0] = colored_cloud->points[count].r;
          (*(colors_host+i))[1] = colored_cloud->points[count].g;
          (*(colors_host+i))[2] = colored_cloud->points[count].b;
        }
        else{
          (*(colors_host+i))[0] = -1;
          (*(colors_host+i))[1] = -1;
          (*(colors_host+i))[2] = -1;
        }

        count++;
      }
      else{
        (*(colors_host+i))[0] = -1;
        (*(colors_host+i))[1] = -1;
        (*(colors_host+i))[2] = -1;
      }
    }

    Vector3f *colors_device;
    ITMSafeCall(cudaMalloc((void**)&colors_device, view->depth->noDims.x*view->depth->noDims.y*sizeof(Vector3f)));
    ITMSafeCall(cudaMemcpy(colors_device, colors_host, view->depth->noDims.x*view->depth->noDims.y*sizeof(Vector3f), cudaMemcpyHostToDevice));

    visualisationEngine->NewCreateICPMaps(scene, view, trackingStateTem, colors_device, NULL, 0);

    free(points_host);
    free(normals_host);
    free(colors_host);
    free(id_array_host);
    points_host=NULL;
    normals_host=NULL;
    colors_host=NULL;
    id_array_host=NULL;

    ITMSafeCall(cudaFree(points_device));
    ITMSafeCall(cudaFree(normals_device));
    ITMSafeCall(cudaFree(colors_device));
#endif
  }
}

//hao modified it
void ITMMainEngine::changeObjectIds(const vector<ushort> &oldIDs, const vector<ushort> &newIds, const vector<uchar> &newRs, const vector<uchar> &newGs, const vector<uchar> &newBs){

  for(int i=0; i<oldIDs.size(); i++){
    objectMap.erase(oldIDs[i]);
  }

  ITMVoxelIndex::IndexData *hashData_host = new ITMVoxelIndex::IndexData;
  ITMVoxel *voxels = (ITMVoxel*)malloc(SDF_LOCAL_BLOCK_NUM*SDF_BLOCK_SIZE3 * sizeof(ITMVoxel));

  //printf("aaaaaa\n");

  bool flag = false;
#ifndef COMPILE_WITHOUT_CUDA
  flag=true;
#endif
  if(flag){
    ITMSafeCall(cudaMemcpy(hashData_host, scene->index.getIndexData(), sizeof(ITMVoxelIndex::IndexData), cudaMemcpyDeviceToHost));
    ITMSafeCall(cudaMemcpy(voxels, scene->localVBA.GetVoxelBlocks(), SDF_LOCAL_BLOCK_NUM*SDF_BLOCK_SIZE3*sizeof(ITMVoxel), cudaMemcpyDeviceToHost));
  }
  else{
    memcpy(hashData_host, scene->index.getIndexData(), sizeof(ITMVoxelIndex::IndexData));
    memcpy(voxels, scene->localVBA.GetVoxelBlocks(), SDF_LOCAL_BLOCK_NUM*SDF_BLOCK_SIZE3 * sizeof(ITMVoxel));
  }
  //printf("bbbbbb\n");
  const ITMHashEntry *hashTable = hashData_host->entries_all;

  for(int i=0; i<SDF_BUCKET_NUM * SDF_ENTRY_NUM_PER_BUCKET + SDF_EXCESS_LIST_SIZE; i++){
    const ITMHashEntry &hashEntry = hashTable[i];

    if(hashEntry.ptr >= 0){
      for(int j=0; j<SDF_BLOCK_SIZE3; j++){
        ITMVoxel res = voxels[(hashEntry.ptr * SDF_BLOCK_SIZE3) + j];

        //if(res.id == 3){
        //  printf("dddddd\n");
        //}

        for(int k=0; k<oldIDs.size(); k++){
          if(res.id == oldIDs[k]){
            voxels[(hashEntry.ptr * SDF_BLOCK_SIZE3) + j].id = newIds[k];
            voxels[(hashEntry.ptr * SDF_BLOCK_SIZE3) + j].r = newRs[k];
            voxels[(hashEntry.ptr * SDF_BLOCK_SIZE3) + j].g = newGs[k];
            voxels[(hashEntry.ptr * SDF_BLOCK_SIZE3) + j].b = newBs[k];
          }
        }
      }
    }
  }

  if(flag){
    ITMSafeCall(cudaMemcpy(scene->localVBA.GetVoxelBlocks(), voxels, SDF_LOCAL_BLOCK_NUM*SDF_BLOCK_SIZE3*sizeof(ITMVoxel), cudaMemcpyHostToDevice));
  }
  else{
    memcpy(scene->localVBA.GetVoxelBlocks(), voxels, SDF_LOCAL_BLOCK_NUM*SDF_BLOCK_SIZE3 * sizeof(ITMVoxel));
  }

  //printf("cccccc\n");
  free(voxels);
  free(hashData_host);
  voxels=NULL;
  hashData_host=NULL;
}

//hao modified it
void ITMMainEngine::savePoints()
{
  //visualisationEngine->GetAllPoints(scene);
  PointCloudPtr_RGB_NORMAL cloud(new PointCloud_RGB_NORMAL);
  //int num=0;

  ITMVoxelIndex::IndexData *hashData_host = new ITMVoxelIndex::IndexData;
  ITMVoxel *voxels = (ITMVoxel*)malloc(SDF_LOCAL_BLOCK_NUM*SDF_BLOCK_SIZE3 * sizeof(ITMVoxel));

  bool flag = false;
#ifndef COMPILE_WITHOUT_CUDA
  flag=true;
#endif
  if(flag){
    ITMSafeCall(cudaMemcpy(hashData_host, scene->index.getIndexData(), sizeof(ITMVoxelIndex::IndexData), cudaMemcpyDeviceToHost));
    ITMSafeCall(cudaMemcpy(voxels, scene->localVBA.GetVoxelBlocks(), SDF_LOCAL_BLOCK_NUM*SDF_BLOCK_SIZE3*sizeof(ITMVoxel), cudaMemcpyDeviceToHost));
  }
  else{
    memcpy(hashData_host, scene->index.getIndexData(), sizeof(ITMVoxelIndex::IndexData));
    memcpy(voxels, scene->localVBA.GetVoxelBlocks(), SDF_LOCAL_BLOCK_NUM*SDF_BLOCK_SIZE3 * sizeof(ITMVoxel));
  }

  const ITMHashEntry *hashTable = hashData_host->entries_all;
  float mu = scene->sceneParams->mu;

  for(int i=0; i<SDF_BUCKET_NUM * SDF_ENTRY_NUM_PER_BUCKET + SDF_EXCESS_LIST_SIZE; i++){
    const ITMHashEntry &hashEntry = hashTable[i];

    if(hashEntry.ptr >= 0){
      for(int j=0; j<SDF_BLOCK_SIZE3; j++){
        ITMVoxel res = voxels[(hashEntry.ptr * SDF_BLOCK_SIZE3) + j];

        float value = ITMVoxel::SDF_valueToFloat(res.sdf);
        if(value<10*mu&&value>-10*mu){
          //cout<<"value:"<<value<<endl;

          Point_RGB_NORMAL p;
          float voxelSize = 0.125f;
          float blockSizeWorld = scene->sceneParams->voxelSize*SDF_BLOCK_SIZE; // = 0.005*8;

          p.z = (hashEntry.pos.z+(j/(SDF_BLOCK_SIZE*SDF_BLOCK_SIZE)+0.5f)*voxelSize)*blockSizeWorld;
          p.y = (hashEntry.pos.y+((j%(SDF_BLOCK_SIZE*SDF_BLOCK_SIZE))/SDF_BLOCK_SIZE+0.5f)*voxelSize)*blockSizeWorld;
          p.x = (hashEntry.pos.x+((j%(SDF_BLOCK_SIZE*SDF_BLOCK_SIZE))%SDF_BLOCK_SIZE+0.5f)*voxelSize)*blockSizeWorld;
          p.r = res.r;
          p.g = res.g;
          p.b = res.b;

          Vector3f pt((hashEntry.pos.x*SDF_BLOCK_SIZE+((j%(SDF_BLOCK_SIZE*SDF_BLOCK_SIZE))%SDF_BLOCK_SIZE+0.5f)), (hashEntry.pos.y*SDF_BLOCK_SIZE+((j%(SDF_BLOCK_SIZE*SDF_BLOCK_SIZE))/SDF_BLOCK_SIZE+0.5f)), (hashEntry.pos.z*SDF_BLOCK_SIZE+(j/(SDF_BLOCK_SIZE*SDF_BLOCK_SIZE)+0.5f)));

          Vector3f normal_host = computeSingleNormalFromSDF(voxels, hashData_host, pt);

          float normScale = 1.0f / sqrtf(normal_host.x * normal_host.x + normal_host.y * normal_host.y + normal_host.z * normal_host.z);
          normal_host *= normScale;

          Vector3f pn(normal_host[0], normal_host[1], normal_host[2]);
          Vector3f tem(-p.x, -p.y, -p.z);

          double dotvalue = pn.x*tem.x + pn.y*tem.y + pn.z*tem.z;
          if(dotvalue<0){
            pn = -pn;
          }

          p.normal_x = pn.x;
          p.normal_y = pn.y;
          p.normal_z = pn.z;

          cloud->points.push_back(p);
        }
      }
    }
  }
  pcl::io::savePLYFileBinary("Data/scan_points.ply", *cloud);

  free(voxels);
  free(hashData_host);
  voxels=NULL;
  hashData_host=NULL;

  printf("savePoints finished.\n");
}

//ccjn modified it
int hashIndex(const Vector3s voxelPos, const int hashMask) {
  return ((uint)(((uint)voxelPos.x * 73856093) ^ ((uint)voxelPos.y * 19349669) ^ ((uint)voxelPos.z * 83492791)) & (uint)hashMask);
}

//ccjn modified it
int FindVBIndex(const Vector3s blockPos, const ITMHashEntry *hashTable)
{
  int offsetExcess = 0;
  int hashIdx = hashIndex(blockPos, SDF_HASH_MASK) * SDF_ENTRY_NUM_PER_BUCKET;


  //check ordered list
  for(int inBucketIdx = 0 ; inBucketIdx < SDF_ENTRY_NUM_PER_BUCKET ; inBucketIdx++)
  {
    const ITMHashEntry &hashEntry = hashTable[hashIdx + inBucketIdx];
    offsetExcess = hashEntry.offset - 1;

    if(hashEntry.ptr < 0 ) return -1;

    if(hashEntry.pos == blockPos && hashEntry.ptr >= 0)
      return hashIdx + inBucketIdx;
  }

  //check excess list
  while( offsetExcess >= 0)
  {
    const ITMHashEntry &hashEntry = hashTable[SDF_BUCKET_NUM  * SDF_ENTRY_NUM_PER_BUCKET + offsetExcess];

    if(hashEntry.pos == blockPos && hashEntry.ptr >= 0 )
      return SDF_BUCKET_NUM  * SDF_ENTRY_NUM_PER_BUCKET + offsetExcess;
    offsetExcess = hashEntry.offset - 1;
  }

  return -1;
}

//ccjn modified it
void ITMMainEngine::saveMesh()
{

  ITMVoxelIndex::IndexData *hashData_host = new ITMVoxelIndex::IndexData;
  ITMVoxel *voxels = (ITMVoxel*)malloc(SDF_LOCAL_BLOCK_NUM*SDF_BLOCK_SIZE3 * sizeof(ITMVoxel));

  bool flag = false;
#ifndef COMPILE_WITHOUT_CUDA
  flag=true;
#endif
  if(flag){
    ITMSafeCall(cudaMemcpy(hashData_host, scene->index.getIndexData(), sizeof(ITMVoxelIndex::IndexData), cudaMemcpyDeviceToHost));
    ITMSafeCall(cudaMemcpy(voxels, scene->localVBA.GetVoxelBlocks(), SDF_LOCAL_BLOCK_NUM*SDF_BLOCK_SIZE3*sizeof(ITMVoxel), cudaMemcpyDeviceToHost));
  }
  else{
    memcpy(hashData_host, scene->index.getIndexData(), sizeof(ITMVoxelIndex::IndexData));
    memcpy(voxels, scene->localVBA.GetVoxelBlocks(), SDF_LOCAL_BLOCK_NUM*SDF_BLOCK_SIZE3 * sizeof(ITMVoxel));
  }

  const ITMHashEntry *hashTable = hashData_host->entries_all;
  float mu = scene->sceneParams->mu;

  const float isolevel = 0.0f;

  for(int i=0; i<SDF_BUCKET_NUM * SDF_ENTRY_NUM_PER_BUCKET + SDF_EXCESS_LIST_SIZE; i++)
  {
    const ITMHashEntry &hashEntry = hashTable[i];

    if(hashEntry.ptr >= 0) // means there is a voxel block
    {
      int ExceedX = FindVBIndex(Vector3s(hashEntry.pos.x + 1 , hashEntry.pos.y , hashEntry.pos.z), hashTable);
      int ExceedY = FindVBIndex(Vector3s(hashEntry.pos.x , hashEntry.pos.y + 1 , hashEntry.pos.z), hashTable);
      int ExceedZ = FindVBIndex(Vector3s(hashEntry.pos.x , hashEntry.pos.y , hashEntry.pos.z + 1), hashTable);
      int ExceedXY = FindVBIndex(Vector3s(hashEntry.pos.x + 1 , hashEntry.pos.y + 1 , hashEntry.pos.z), hashTable);
      int ExceedXZ = FindVBIndex(Vector3s(hashEntry.pos.x + 1 , hashEntry.pos.y , hashEntry.pos.z + 1), hashTable);
      int ExceedYZ = FindVBIndex(Vector3s(hashEntry.pos.x , hashEntry.pos.y + 1 , hashEntry.pos.z + 1), hashTable);
      int ExceedXYZ = FindVBIndex(Vector3s(hashEntry.pos.x + 1 , hashEntry.pos.y + 1 , hashEntry.pos.z + 1), hashTable);

      //modified
      ITMHashEntry hashEntryEX;
      if(ExceedX != -1){
        hashEntryEX = hashTable[ExceedX];
      }

      ITMHashEntry hashEntryEY;
      if(ExceedY != -1){
        hashEntryEY = hashTable[ExceedY];
      }

      ITMHashEntry hashEntryEZ;
      if(ExceedZ != -1){
        hashEntryEZ = hashTable[ExceedZ];
      }

      ITMHashEntry hashEntryEXY;
      if(ExceedXY != -1){
        hashEntryEXY = hashTable[ExceedXY];
      }

      ITMHashEntry hashEntryEXZ;
      if(ExceedXZ != -1){
        hashEntryEXZ = hashTable[ExceedXZ];
      }

      ITMHashEntry hashEntryEYZ;
      if(ExceedYZ != -1){
        hashEntryEYZ = hashTable[ExceedYZ];
      }

      ITMHashEntry hashEntryEXYZ;
      if(ExceedXYZ != -1){
        hashEntryEXYZ = hashTable[ExceedXYZ];
      }

      float blockSizeWorld = scene->sceneParams->voxelSize*SDF_BLOCK_SIZE; // = 0.005*8;
      float voxelSize = 0.125f;

      for(int j=0 ; j<SDF_BLOCK_SIZE3 /*- SDF_BLOCK_SIZE*SDF_BLOCK_SIZE*/ ; j++)
      {

        int ZZ = (int)(j/(SDF_BLOCK_SIZE*SDF_BLOCK_SIZE));
        int YY = (int)((j%(SDF_BLOCK_SIZE*SDF_BLOCK_SIZE))/SDF_BLOCK_SIZE);
        int XX = (int)((j%(SDF_BLOCK_SIZE*SDF_BLOCK_SIZE))%SDF_BLOCK_SIZE);

        ITMVoxel v000,v100,v010,v001,v110,v011,v101,v111;

        if((XX == 7) && (YY == 7) && (ZZ == 7))
        {
          if(ExceedX == -1) continue;
          if(ExceedY == -1) continue;
          if(ExceedZ == -1) continue;
          if(ExceedXY == -1) continue;
          if(ExceedYZ == -1) continue;
          if(ExceedXZ == -1) continue;
          if(ExceedXYZ == -1) continue;

          v000 = voxels[(hashEntry.ptr * SDF_BLOCK_SIZE3) + XX + YY*SDF_BLOCK_SIZE + ZZ*SDF_BLOCK_SIZE*SDF_BLOCK_SIZE ];
          v100 = voxels[(hashEntryEX.ptr * SDF_BLOCK_SIZE3) + YY*SDF_BLOCK_SIZE + ZZ*SDF_BLOCK_SIZE*SDF_BLOCK_SIZE];
          v010 = voxels[(hashEntryEY.ptr * SDF_BLOCK_SIZE3) + XX + ZZ*SDF_BLOCK_SIZE*SDF_BLOCK_SIZE ];
          v001 = voxels[(hashEntryEZ.ptr * SDF_BLOCK_SIZE3) + XX + YY*SDF_BLOCK_SIZE];
          v110 = voxels[(hashEntryEXY.ptr * SDF_BLOCK_SIZE3) + ZZ*SDF_BLOCK_SIZE*SDF_BLOCK_SIZE];
          v011 = voxels[(hashEntryEYZ.ptr * SDF_BLOCK_SIZE3) + XX];
          v101 = voxels[(hashEntryEXZ.ptr * SDF_BLOCK_SIZE3) + YY*SDF_BLOCK_SIZE];
          v111 = voxels[(hashEntryEXYZ.ptr * SDF_BLOCK_SIZE3) ];
        }

        if((XX == 7) && (YY == 7) && (ZZ != 7))
        {
          if(ExceedX == -1) continue;
          if(ExceedY == -1) continue;
          if(ExceedXY == -1) continue;

          v000 = voxels[(hashEntry.ptr * SDF_BLOCK_SIZE3) + XX + YY*SDF_BLOCK_SIZE + ZZ*SDF_BLOCK_SIZE*SDF_BLOCK_SIZE ];
          v100 = voxels[(hashEntryEX.ptr * SDF_BLOCK_SIZE3) + YY*SDF_BLOCK_SIZE + ZZ*SDF_BLOCK_SIZE*SDF_BLOCK_SIZE];
          v010 = voxels[(hashEntryEY.ptr * SDF_BLOCK_SIZE3) + XX + ZZ*SDF_BLOCK_SIZE*SDF_BLOCK_SIZE ];
          v001 = voxels[(hashEntry.ptr * SDF_BLOCK_SIZE3) + XX + YY*SDF_BLOCK_SIZE + (ZZ + 1)*SDF_BLOCK_SIZE*SDF_BLOCK_SIZE];
          v110 = voxels[(hashEntryEXY.ptr * SDF_BLOCK_SIZE3) + ZZ*SDF_BLOCK_SIZE*SDF_BLOCK_SIZE ];
          v011 = voxels[(hashEntryEY.ptr * SDF_BLOCK_SIZE3) + XX + (ZZ + 1)*SDF_BLOCK_SIZE*SDF_BLOCK_SIZE];
          v101 = voxels[(hashEntryEX.ptr * SDF_BLOCK_SIZE3) + YY*SDF_BLOCK_SIZE + (ZZ + 1)*SDF_BLOCK_SIZE*SDF_BLOCK_SIZE];
          v111 = voxels[(hashEntryEXY.ptr * SDF_BLOCK_SIZE3) + (ZZ + 1)*SDF_BLOCK_SIZE*SDF_BLOCK_SIZE];
        }

        if((XX == 7) && (YY != 7) && (ZZ == 7))
        {
          if(ExceedX == -1) continue;
          if(ExceedZ == -1) continue;
          if(ExceedXZ == -1) continue;

          v000 = voxels[(hashEntry.ptr * SDF_BLOCK_SIZE3) + XX + YY*SDF_BLOCK_SIZE + ZZ*SDF_BLOCK_SIZE*SDF_BLOCK_SIZE ];
          v100 = voxels[(hashEntryEX.ptr * SDF_BLOCK_SIZE3) + YY*SDF_BLOCK_SIZE + ZZ*SDF_BLOCK_SIZE*SDF_BLOCK_SIZE];
          v010 = voxels[(hashEntry.ptr * SDF_BLOCK_SIZE3) + XX + (YY + 1)*SDF_BLOCK_SIZE + ZZ*SDF_BLOCK_SIZE*SDF_BLOCK_SIZE ];
          v001 = voxels[(hashEntryEZ.ptr * SDF_BLOCK_SIZE3) + XX + YY*SDF_BLOCK_SIZE];
          v110 = voxels[(hashEntryEX.ptr * SDF_BLOCK_SIZE3) + (YY + 1)*SDF_BLOCK_SIZE + ZZ*SDF_BLOCK_SIZE*SDF_BLOCK_SIZE ];
          v011 = voxels[(hashEntryEZ.ptr * SDF_BLOCK_SIZE3) + XX + (YY + 1)*SDF_BLOCK_SIZE];
          v101 = voxels[(hashEntryEXZ.ptr * SDF_BLOCK_SIZE3) + YY*SDF_BLOCK_SIZE];
          v111 = voxels[(hashEntryEXZ.ptr * SDF_BLOCK_SIZE3) + (YY + 1)*SDF_BLOCK_SIZE];

        }

        if((XX != 7) && (YY == 7) && (ZZ == 7))
        {
          if(ExceedY == -1) continue;
          if(ExceedZ == -1) continue;
          if(ExceedYZ == -1) continue;

          v000 = voxels[(hashEntry.ptr * SDF_BLOCK_SIZE3) + XX + YY*SDF_BLOCK_SIZE + ZZ*SDF_BLOCK_SIZE*SDF_BLOCK_SIZE ];
          v100 = voxels[(hashEntry.ptr * SDF_BLOCK_SIZE3) + (XX + 1) + YY*SDF_BLOCK_SIZE + ZZ*SDF_BLOCK_SIZE*SDF_BLOCK_SIZE];
          v010 = voxels[(hashEntryEY.ptr * SDF_BLOCK_SIZE3) + XX + ZZ*SDF_BLOCK_SIZE*SDF_BLOCK_SIZE ];
          v001 = voxels[(hashEntryEZ.ptr * SDF_BLOCK_SIZE3) + XX + YY*SDF_BLOCK_SIZE];
          v110 = voxels[(hashEntryEY.ptr * SDF_BLOCK_SIZE3) + (XX + 1) + ZZ*SDF_BLOCK_SIZE*SDF_BLOCK_SIZE ];
          v011 = voxels[(hashEntryEYZ.ptr * SDF_BLOCK_SIZE3) + XX];
          v101 = voxels[(hashEntryEZ.ptr * SDF_BLOCK_SIZE3) + (XX + 1) + YY*SDF_BLOCK_SIZE];
          v111 = voxels[(hashEntryEYZ.ptr * SDF_BLOCK_SIZE3) + (XX + 1)];
        }

        if((XX == 7) && (YY != 7) && (ZZ != 7))
        {
          if(ExceedX == -1) continue;

          v000 = voxels[(hashEntry.ptr * SDF_BLOCK_SIZE3) + XX + YY*SDF_BLOCK_SIZE + ZZ*SDF_BLOCK_SIZE*SDF_BLOCK_SIZE ];
          v100 = voxels[(hashEntryEX.ptr * SDF_BLOCK_SIZE3) + YY*SDF_BLOCK_SIZE + ZZ*SDF_BLOCK_SIZE*SDF_BLOCK_SIZE];
          v010 = voxels[(hashEntry.ptr * SDF_BLOCK_SIZE3) + XX + (YY + 1)*SDF_BLOCK_SIZE + ZZ*SDF_BLOCK_SIZE*SDF_BLOCK_SIZE ];
          v001 = voxels[(hashEntry.ptr * SDF_BLOCK_SIZE3) + XX + YY*SDF_BLOCK_SIZE + (ZZ + 1)*SDF_BLOCK_SIZE*SDF_BLOCK_SIZE];
          v110 = voxels[(hashEntryEX.ptr * SDF_BLOCK_SIZE3) + (YY + 1)*SDF_BLOCK_SIZE + ZZ*SDF_BLOCK_SIZE*SDF_BLOCK_SIZE];
          v011 = voxels[(hashEntry.ptr * SDF_BLOCK_SIZE3) + XX + (YY + 1)*SDF_BLOCK_SIZE + (ZZ + 1)*SDF_BLOCK_SIZE*SDF_BLOCK_SIZE];
          v101 = voxels[(hashEntryEX.ptr * SDF_BLOCK_SIZE3) + YY*SDF_BLOCK_SIZE + (ZZ + 1)*SDF_BLOCK_SIZE*SDF_BLOCK_SIZE];
          v111 = voxels[(hashEntryEX.ptr * SDF_BLOCK_SIZE3) + (YY + 1)*SDF_BLOCK_SIZE + (ZZ + 1)*SDF_BLOCK_SIZE*SDF_BLOCK_SIZE];
        }

        if((XX != 7) && (YY == 7) && (ZZ != 7))
        {
          if(ExceedY == -1) continue;

          v000 = voxels[(hashEntry.ptr * SDF_BLOCK_SIZE3) + XX + YY*SDF_BLOCK_SIZE + ZZ*SDF_BLOCK_SIZE*SDF_BLOCK_SIZE ];
          v100 = voxels[(hashEntry.ptr * SDF_BLOCK_SIZE3) + (XX + 1) + YY*SDF_BLOCK_SIZE + ZZ*SDF_BLOCK_SIZE*SDF_BLOCK_SIZE];
          v010 = voxels[(hashEntryEY.ptr * SDF_BLOCK_SIZE3) + XX + ZZ*SDF_BLOCK_SIZE*SDF_BLOCK_SIZE ];
          v001 = voxels[(hashEntry.ptr * SDF_BLOCK_SIZE3) + XX + YY*SDF_BLOCK_SIZE + (ZZ + 1)*SDF_BLOCK_SIZE*SDF_BLOCK_SIZE];
          v110 = voxels[(hashEntryEY.ptr * SDF_BLOCK_SIZE3) + (XX + 1) + ZZ*SDF_BLOCK_SIZE*SDF_BLOCK_SIZE];
          v011 = voxels[(hashEntryEY.ptr * SDF_BLOCK_SIZE3) + XX + (ZZ + 1)*SDF_BLOCK_SIZE*SDF_BLOCK_SIZE ];
          v101 = voxels[(hashEntry.ptr * SDF_BLOCK_SIZE3) + (XX + 1) + YY*SDF_BLOCK_SIZE + (ZZ + 1)*SDF_BLOCK_SIZE*SDF_BLOCK_SIZE];
          v111 = voxels[(hashEntryEY.ptr * SDF_BLOCK_SIZE3) + (XX + 1) + (ZZ + 1)*SDF_BLOCK_SIZE*SDF_BLOCK_SIZE];
        }

        if((XX != 7) && (YY != 7) && (ZZ == 7))
        {
          if(ExceedZ == -1) continue;

          v000 = voxels[(hashEntry.ptr * SDF_BLOCK_SIZE3) + XX + YY*SDF_BLOCK_SIZE + ZZ*SDF_BLOCK_SIZE*SDF_BLOCK_SIZE ];
          v100 = voxels[(hashEntry.ptr * SDF_BLOCK_SIZE3) + (XX + 1) + YY*SDF_BLOCK_SIZE + ZZ*SDF_BLOCK_SIZE*SDF_BLOCK_SIZE];
          v010 = voxels[(hashEntry.ptr * SDF_BLOCK_SIZE3) + XX + (YY + 1)*SDF_BLOCK_SIZE + ZZ*SDF_BLOCK_SIZE*SDF_BLOCK_SIZE ];
          v001 = voxels[(hashEntryEZ.ptr * SDF_BLOCK_SIZE3) + XX + YY*SDF_BLOCK_SIZE];
          v110 = voxels[(hashEntry.ptr * SDF_BLOCK_SIZE3) + (XX + 1) + (YY + 1)*SDF_BLOCK_SIZE + ZZ*SDF_BLOCK_SIZE*SDF_BLOCK_SIZE ];
          v011 = voxels[(hashEntryEZ.ptr * SDF_BLOCK_SIZE3) + XX + (YY + 1)*SDF_BLOCK_SIZE];
          v101 = voxels[(hashEntryEZ.ptr * SDF_BLOCK_SIZE3) + (XX + 1) + YY*SDF_BLOCK_SIZE];
          v111 = voxels[(hashEntryEZ.ptr * SDF_BLOCK_SIZE3) + (XX + 1) + (YY + 1)*SDF_BLOCK_SIZE];
        }

        if((XX != 7) && (YY != 7) && (ZZ != 7))
        {
          v000 = voxels[(hashEntry.ptr * SDF_BLOCK_SIZE3) + XX + YY*SDF_BLOCK_SIZE + ZZ*SDF_BLOCK_SIZE*SDF_BLOCK_SIZE ];
          v100 = voxels[(hashEntry.ptr * SDF_BLOCK_SIZE3) + (XX + 1) + YY*SDF_BLOCK_SIZE + ZZ*SDF_BLOCK_SIZE*SDF_BLOCK_SIZE];
          v010 = voxels[(hashEntry.ptr * SDF_BLOCK_SIZE3) + XX + (YY + 1)*SDF_BLOCK_SIZE + ZZ*SDF_BLOCK_SIZE*SDF_BLOCK_SIZE ];
          v001 = voxels[(hashEntry.ptr * SDF_BLOCK_SIZE3) + XX + YY*SDF_BLOCK_SIZE + (ZZ + 1)*SDF_BLOCK_SIZE*SDF_BLOCK_SIZE];
          v110 = voxels[(hashEntry.ptr * SDF_BLOCK_SIZE3) + (XX + 1) + (YY + 1)*SDF_BLOCK_SIZE + ZZ*SDF_BLOCK_SIZE*SDF_BLOCK_SIZE ];
          v011 = voxels[(hashEntry.ptr * SDF_BLOCK_SIZE3) + XX + (YY + 1)*SDF_BLOCK_SIZE + (ZZ + 1)*SDF_BLOCK_SIZE*SDF_BLOCK_SIZE];
          v101 = voxels[(hashEntry.ptr * SDF_BLOCK_SIZE3) + (XX + 1) + YY*SDF_BLOCK_SIZE + (ZZ + 1)*SDF_BLOCK_SIZE*SDF_BLOCK_SIZE];
          v111 = voxels[(hashEntry.ptr * SDF_BLOCK_SIZE3) + (XX + 1) + (YY + 1)*SDF_BLOCK_SIZE + (ZZ + 1)*SDF_BLOCK_SIZE*SDF_BLOCK_SIZE];
        }

        // if voxel not exist, jump this time
        if(v000.w_depth == 0) continue;
        if(v100.w_depth == 0) continue;
        if(v010.w_depth == 0) continue;
        if(v001.w_depth == 0) continue;
        if(v110.w_depth == 0) continue;
        if(v011.w_depth == 0) continue;
        if(v101.w_depth == 0) continue;
        if(v111.w_depth == 0) continue;

        uint cubeindex = 0;
        if(ITMVoxel::SDF_valueToFloat(v010.sdf) < isolevel) cubeindex += 1;
        if(ITMVoxel::SDF_valueToFloat(v110.sdf) < isolevel) cubeindex += 2;
        if(ITMVoxel::SDF_valueToFloat(v100.sdf) < isolevel) cubeindex += 4;
        if(ITMVoxel::SDF_valueToFloat(v000.sdf) < isolevel) cubeindex += 8;
        if(ITMVoxel::SDF_valueToFloat(v011.sdf) < isolevel) cubeindex += 16;
        if(ITMVoxel::SDF_valueToFloat(v111.sdf) < isolevel) cubeindex += 32;
        if(ITMVoxel::SDF_valueToFloat(v101.sdf) < isolevel) cubeindex += 64;
        if(ITMVoxel::SDF_valueToFloat(v001.sdf) < isolevel) cubeindex += 128;

        if((cubeindex == 0) || (cubeindex == 0xff)) continue;
        if(edgeTable[cubeindex] == 0 || edgeTable[cubeindex] == 255) continue;

        Vector3f p000 = Vector3f(hashEntry.pos.x + XX*voxelSize, hashEntry.pos.y + YY*voxelSize, hashEntry.pos.z + ZZ*voxelSize);
        Vector3f p100 = Vector3f(hashEntry.pos.x + XX*voxelSize + 1*voxelSize, hashEntry.pos.y + YY*voxelSize,hashEntry.pos.z + ZZ*voxelSize );
        Vector3f p010 = Vector3f(hashEntry.pos.x + XX*voxelSize, hashEntry.pos.y + YY*voxelSize + 1*voxelSize, hashEntry.pos.z + ZZ*voxelSize);
        Vector3f p001 = Vector3f(hashEntry.pos.x + XX*voxelSize, hashEntry.pos.y + YY*voxelSize, hashEntry.pos.z + ZZ*voxelSize + 1*voxelSize);
        Vector3f p110 = Vector3f(hashEntry.pos.x + XX*voxelSize + 1*voxelSize , hashEntry.pos.y + YY*voxelSize + 1*voxelSize, hashEntry.pos.z + ZZ*voxelSize);
        Vector3f p011 = Vector3f(hashEntry.pos.x + XX*voxelSize , hashEntry.pos.y + YY*voxelSize + 1*voxelSize, hashEntry.pos.z + ZZ*voxelSize + 1*voxelSize);
        Vector3f p101 = Vector3f(hashEntry.pos.x + XX*voxelSize + 1*voxelSize , hashEntry.pos.y + YY*voxelSize, hashEntry.pos.z + ZZ*voxelSize + 1*voxelSize);
        Vector3f p111 = Vector3f(hashEntry.pos.x + XX*voxelSize + 1*voxelSize , hashEntry.pos.y + YY*voxelSize + 1*voxelSize, hashEntry.pos.z + ZZ*voxelSize + 1*voxelSize);

        //hao modified it(the position in real world)
        p000 *= blockSizeWorld;
        p100 *= blockSizeWorld;
        p010 *= blockSizeWorld;
        p001 *= blockSizeWorld;
        p110 *= blockSizeWorld;
        p011 *= blockSizeWorld;
        p101 *= blockSizeWorld;
        p111 *= blockSizeWorld;

        if(edgeTable[cubeindex] == 0 || edgeTable[cubeindex] == 255) return;  // added by me edgeTable[cubeindex] == 255 !!!

        myVertex vertlist[12];
        if(edgeTable[cubeindex] & 1)	vertlist[0]  = VertexInterp(isolevel, p010, p110, v010, v110);
        if(edgeTable[cubeindex] & 2)	vertlist[1]  = VertexInterp(isolevel, p110, p100, v110, v100);
        if(edgeTable[cubeindex] & 4)	vertlist[2]  = VertexInterp(isolevel, p100, p000, v100, v000);
        if(edgeTable[cubeindex] & 8)	vertlist[3]  = VertexInterp(isolevel, p000, p010, v000, v010);
        if(edgeTable[cubeindex] & 16)	vertlist[4]  = VertexInterp(isolevel, p011, p111, v011, v111);
        if(edgeTable[cubeindex] & 32)	vertlist[5]  = VertexInterp(isolevel, p111, p101, v111, v101);
        if(edgeTable[cubeindex] & 64)	vertlist[6]  = VertexInterp(isolevel, p101, p001, v101, v001);
        if(edgeTable[cubeindex] & 128)	vertlist[7]  = VertexInterp(isolevel, p001, p011, v001, v011);
        if(edgeTable[cubeindex] & 256)	vertlist[8]	 = VertexInterp(isolevel, p010, p011, v010, v011);
        if(edgeTable[cubeindex] & 512)	vertlist[9]  = VertexInterp(isolevel, p110, p111, v110, v111);
        if(edgeTable[cubeindex] & 1024) vertlist[10] = VertexInterp(isolevel, p100, p101, v100, v101);
        if(edgeTable[cubeindex] & 2048) vertlist[11] = VertexInterp(isolevel, p000, p001, v000, v001);



        for(int i=0; triTable[cubeindex][i] != -1; i+=3)
        {
          myTriangle t;
          t.v0 = vertlist[triTable[cubeindex][i+0]];
          t.v1 = vertlist[triTable[cubeindex][i+1]];
          t.v2 = vertlist[triTable[cubeindex][i+2]];

          g_triangles.push_back(t);
        }
      }
    }
  }

  std::cout<<"start writing mesh to file ..."<<std::endl;
  //string filename = "Data/Mesh.off";
  //writeToFileOFF(filename, g_triangles);

  string filename = "Data/scan_mesh.ply";
  writeToFilePly(filename, g_triangles);
  std::cout<<"writing finished!"<<endl;

  g_triangles.clear();
  //writeToFile(g_triangles);
}

//hao modified it
void ITMMainEngine::getGlobalCloud(bool withNormals){
  globalCloud.mypoints.clear();

  //PointCloudPtr cloud(new PointCloud);

  ITMVoxelIndex::IndexData *hashData_host = new ITMVoxelIndex::IndexData;
  ITMVoxel *voxels = (ITMVoxel*)malloc(SDF_LOCAL_BLOCK_NUM*SDF_BLOCK_SIZE3 * sizeof(ITMVoxel));

  bool flag = false;
#ifndef COMPILE_WITHOUT_CUDA
  flag=true;
#endif
  if(flag){
    ITMSafeCall(cudaMemcpy(hashData_host, scene->index.getIndexData(), sizeof(ITMVoxelIndex::IndexData), cudaMemcpyDeviceToHost));
    ITMSafeCall(cudaMemcpy(voxels, scene->localVBA.GetVoxelBlocks(), SDF_LOCAL_BLOCK_NUM*SDF_BLOCK_SIZE3*sizeof(ITMVoxel), cudaMemcpyDeviceToHost));
  }
  else{
    memcpy(hashData_host, scene->index.getIndexData(), sizeof(ITMVoxelIndex::IndexData));
    memcpy(voxels, scene->localVBA.GetVoxelBlocks(), SDF_LOCAL_BLOCK_NUM*SDF_BLOCK_SIZE3 * sizeof(ITMVoxel));
  }

  const ITMHashEntry *hashTable = hashData_host->entries_all;
  float mu = scene->sceneParams->mu;

  for(int i=0; i<SDF_BUCKET_NUM * SDF_ENTRY_NUM_PER_BUCKET + SDF_EXCESS_LIST_SIZE; i++){
    const ITMHashEntry &hashEntry = hashTable[i];

    if(hashEntry.ptr >= 0){
      for(int j=0; j<SDF_BLOCK_SIZE3; j++){
        ITMVoxel res = voxels[(hashEntry.ptr * SDF_BLOCK_SIZE3) + j];

        float value = ITMVoxel::SDF_valueToFloat(res.sdf);
        if(value<10*mu&&value>-10*mu){
          //cout<<"value:"<<value<<endl;

          MyPoint_RGB_NORMAL_ID p;
          float voxelSize = 0.125f;
          float blockSizeWorld = scene->sceneParams->voxelSize*SDF_BLOCK_SIZE; // = 0.005*8;

          p.z = (hashEntry.pos.z+(j/(SDF_BLOCK_SIZE*SDF_BLOCK_SIZE)+0.5f)*voxelSize)*blockSizeWorld;
          p.y = (hashEntry.pos.y+((j%(SDF_BLOCK_SIZE*SDF_BLOCK_SIZE))/SDF_BLOCK_SIZE+0.5f)*voxelSize)*blockSizeWorld;
          p.x = (hashEntry.pos.x+((j%(SDF_BLOCK_SIZE*SDF_BLOCK_SIZE))%SDF_BLOCK_SIZE+0.5f)*voxelSize)*blockSizeWorld;
          p.r = res.r;
          p.g = res.g;
          p.b = res.b;
          p.id = res.id;

          if(withNormals){
            Vector3f pt((hashEntry.pos.x*SDF_BLOCK_SIZE+((j%(SDF_BLOCK_SIZE*SDF_BLOCK_SIZE))%SDF_BLOCK_SIZE+0.5f)), (hashEntry.pos.y*SDF_BLOCK_SIZE+((j%(SDF_BLOCK_SIZE*SDF_BLOCK_SIZE))/SDF_BLOCK_SIZE+0.5f)), (hashEntry.pos.z*SDF_BLOCK_SIZE+(j/(SDF_BLOCK_SIZE*SDF_BLOCK_SIZE)+0.5f)));

            Vector3f normal_host = computeSingleNormalFromSDF(voxels, hashData_host, pt);

            float normScale = 1.0f / sqrtf(normal_host.x * normal_host.x + normal_host.y * normal_host.y + normal_host.z * normal_host.z);
            normal_host *= normScale;

            Vector3f pn(normal_host[0], normal_host[1], normal_host[2]);
            Vector3f tem(-p.x, -p.y, -p.z);

            double dotvalue = pn.x*tem.x + pn.y*tem.y + pn.z*tem.z;
            if(dotvalue<0){
              pn = -pn;
            }

            p.normal_x = pn.x;
            p.normal_y = pn.y;
            p.normal_z = pn.z;
          }

          globalCloud.mypoints.push_back(p);
        }
      }
    }
  }

  free(voxels);
  free(hashData_host);
  voxels=NULL;
  hashData_host=NULL;
}

//hao modified it
void ITMMainEngine::changeVoxelValue(ITMVoxel *voxels, const ITMHashEntry *hashTable, Point_RGB pt, ushort objectId){

  float voxelSize = 0.125f;
  float blockSizeWorld = scene->sceneParams->voxelSize*SDF_BLOCK_SIZE; // = 0.005*8;

  Vector3s blockPos;
  blockPos.x = floor(pt.x/blockSizeWorld);
  blockPos.y = floor(pt.y/blockSizeWorld);
  blockPos.z = floor(pt.z/blockSizeWorld);

  Vector3s voxelPoseInBlock;
  voxelPoseInBlock.x = floor((pt.x/blockSizeWorld - blockPos.x)/voxelSize);
  voxelPoseInBlock.y = floor((pt.y/blockSizeWorld - blockPos.y)/voxelSize);
  voxelPoseInBlock.z = floor((pt.z/blockSizeWorld - blockPos.z)/voxelSize);

  int lineIndex = voxelPoseInBlock.x + voxelPoseInBlock.y*SDF_BLOCK_SIZE + voxelPoseInBlock.z*SDF_BLOCK_SIZE*SDF_BLOCK_SIZE;

  int res = FindVBIndex(blockPos, hashTable);

  if(res!=-1){
    const ITMHashEntry &hashEntry = hashTable[res];
    voxels[(hashEntry.ptr * SDF_BLOCK_SIZE3) + lineIndex].r = pt.r;
    voxels[(hashEntry.ptr * SDF_BLOCK_SIZE3) + lineIndex].g = pt.g;
    voxels[(hashEntry.ptr * SDF_BLOCK_SIZE3) + lineIndex].b = pt.b;
    voxels[(hashEntry.ptr * SDF_BLOCK_SIZE3) + lineIndex].id = objectId;
  }
}

//hao modified it
void ITMMainEngine::updateVoxelsValues(PointCloudPtr_RGB cloud, vector<ushort> objectIds){
  ITMVoxelIndex::IndexData *hashData_host = new ITMVoxelIndex::IndexData;
  ITMVoxel *voxels = (ITMVoxel*)malloc(SDF_LOCAL_BLOCK_NUM*SDF_BLOCK_SIZE3 * sizeof(ITMVoxel));

  bool flag = false;
#ifndef COMPILE_WITHOUT_CUDA
  flag=true;
#endif
  if(flag){
    ITMSafeCall(cudaMemcpy(hashData_host, scene->index.getIndexData(), sizeof(ITMVoxelIndex::IndexData), cudaMemcpyDeviceToHost));
    ITMSafeCall(cudaMemcpy(voxels, scene->localVBA.GetVoxelBlocks(), SDF_LOCAL_BLOCK_NUM*SDF_BLOCK_SIZE3*sizeof(ITMVoxel), cudaMemcpyDeviceToHost));
  }
  else{
    memcpy(hashData_host, scene->index.getIndexData(), sizeof(ITMVoxelIndex::IndexData));
    memcpy(voxels, scene->localVBA.GetVoxelBlocks(), SDF_LOCAL_BLOCK_NUM*SDF_BLOCK_SIZE3 * sizeof(ITMVoxel));
  }

  const ITMHashEntry *hashTable = hashData_host->entries_all;

  for(int i=0; i<cloud->size(); i++){
    ushort objectId = 0;
    if(objectIds[i]==0){
      objectId = 1;
    }
    else{
      objectId = objectIds[i]+idCount;
    }

    //if(objectId == 3){
    //  printf("aaaaaa\n");
    //}

    changeVoxelValue(voxels, hashTable, cloud->points[i], objectId);
  }

  if(flag){
    ITMSafeCall(cudaMemcpy(scene->localVBA.GetVoxelBlocks(), voxels, SDF_LOCAL_BLOCK_NUM*SDF_BLOCK_SIZE3*sizeof(ITMVoxel), cudaMemcpyHostToDevice));
  }
  else{
    memcpy(scene->localVBA.GetVoxelBlocks(), voxels, SDF_LOCAL_BLOCK_NUM*SDF_BLOCK_SIZE3 * sizeof(ITMVoxel));
  }

  free(voxels);
  free(hashData_host);
  voxels=NULL;
  hashData_host=NULL;
}

//hao modified it
void ITMMainEngine::segmentGlobal(){
  PointCloudPtr_RGB_NORMAL source_cloud(new PointCloud_RGB_NORMAL);

  getGlobalCloud();

  for(int i=0; i<globalCloud.mypoints.size(); i++){
    Point_RGB_NORMAL pt;
    pt.x = globalCloud.mypoints[i].x;
    pt.y = globalCloud.mypoints[i].y;
    pt.z = globalCloud.mypoints[i].z;
    pt.r = 255;
    pt.g = 255;
    pt.b = 255;
    pt.normal_x = globalCloud.mypoints[i].normal_x;
    pt.normal_y = globalCloud.mypoints[i].normal_y;
    pt.normal_z = globalCloud.mypoints[i].normal_z;
    source_cloud->push_back(pt);
  }

  PointCloudPtr_RGB object_cloud(new PointCloud_RGB);
  PointCloudPtr_RGB confidence_cloud(new PointCloud_RGB);
  vector<ushort> objectIndexs;
  Eigen::Vector3f range(0.5, 0.5, 1.0);
  int objectNum;
  vector<ObjectAttri> obas;
  //showPointCloud3(source_cloud, "source_cloud");

  segmentObject(source_cloud, range, obas, object_cloud, confidence_cloud, objectIndexs, objectNum);

  for(int i=0; i<obas.size(); i++){
    objectMap[idCount+i+1] = obas[i];
  }

  //showPointCloud(object_cloud, "object_cloud");

  updateVoxelsValues(object_cloud, objectIndexs);

  idCount += objectNum;

  printf("segmentGlobal finished.\n");
}

//hao modified it
void ITMMainEngine::segmentPortionInGlobal(const vector<ushort> &objectIds){
  vector<ushort> newIds;
  vector<uchar> newRs;
  vector<uchar> newGs;
  vector<uchar> newBs;
  for(int i=0; i<objectIds.size(); i++){
    newIds.push_back(0);
    newRs.push_back(255);
    newGs.push_back(255);
    newBs.push_back(255);
  }

  changeObjectIds(objectIds, newIds, newRs, newGs, newBs);

  getGlobalCloud();

  PointCloudPtr_RGB_NORMAL source_cloud(new PointCloud_RGB_NORMAL);

  for(int i=0; i<globalCloud.mypoints.size(); i++){
    Point_RGB_NORMAL pt;
    pt.x = globalCloud.mypoints[i].x;
    pt.y = globalCloud.mypoints[i].y;
    pt.z = globalCloud.mypoints[i].z;
    pt.r = globalCloud.mypoints[i].r;
    pt.g = globalCloud.mypoints[i].g;
    pt.b = globalCloud.mypoints[i].b;
    pt.normal_x = globalCloud.mypoints[i].normal_x;
    pt.normal_y = globalCloud.mypoints[i].normal_y;
    pt.normal_z = globalCloud.mypoints[i].normal_z;
    source_cloud->push_back(pt);
  }


  PointCloudPtr_RGB object_cloud(new PointCloud_RGB);
  PointCloudPtr_RGB confidence_cloud(new PointCloud_RGB);
  vector<ushort> objectIndexs;
  Eigen::Vector3f range(0.5, 0.5, 1.0);
  int objectNum;
  vector<ObjectAttri> obas;
  segmentSepcialObjects(source_cloud, range, obas, object_cloud, confidence_cloud, objectIndexs, objectNum);

  for(int i=0; i<obas.size(); i++){
    objectMap[idCount+i+1] = obas[i];
  }

  updateVoxelsValues(object_cloud, objectIndexs);

  idCount += objectNum;
}

//hao modified it
void ITMMainEngine::refineSegment(){
  vector<ushort> objectIds;
  objectIds.push_back(2);

  segmentPortionInGlobal(objectIds);

  printf("refineSegment finished.\n");
}

//hao modified it
void ITMMainEngine::interactedSegment(){
  saveViewPoints(this->trackingStateTem);
  {
#ifndef COMPILE_WITHOUT_CUDA
    Vector3f *points_device;
    Vector3f *points_host = (Vector3f*)malloc(view->depth->noDims.x*view->depth->noDims.y*sizeof(Vector3f));
    Vector3f *normals_device;
    Vector3f *normals_host = (Vector3f*)malloc(view->depth->noDims.x*view->depth->noDims.y*sizeof(Vector3f));
    Vector3f *colors_device;
    Vector3f *colors_host = (Vector3f*)malloc(view->depth->noDims.x*view->depth->noDims.y*sizeof(Vector3f));
    ushort *objectIds_device;
    ushort *objectIds_host = (ushort*)malloc(view->depth->noDims.x*view->depth->noDims.y*sizeof(ushort));
    ITMSafeCall(cudaMalloc((void**)&points_device, view->depth->noDims.x*view->depth->noDims.y*sizeof(Vector3f)));
    ITMSafeCall(cudaMemset(points_device, 0, view->depth->noDims.x*view->depth->noDims.y*sizeof(Vector3f)));
    ITMSafeCall(cudaMalloc((void**)&normals_device, view->depth->noDims.x*view->depth->noDims.y*sizeof(Vector3f)));
    ITMSafeCall(cudaMemset(normals_device, 0, view->depth->noDims.x*view->depth->noDims.y*sizeof(Vector3f)));
    ITMSafeCall(cudaMalloc((void**)&colors_device, view->depth->noDims.x*view->depth->noDims.y*sizeof(Vector3f)));
    ITMSafeCall(cudaMemset(colors_device, 0, view->depth->noDims.x*view->depth->noDims.y*sizeof(Vector3f)));
    ITMSafeCall(cudaMalloc((void**)&objectIds_device, view->depth->noDims.x*view->depth->noDims.y*sizeof(ushort)));
    ITMSafeCall(cudaMemset(objectIds_device, 0, view->depth->noDims.x*view->depth->noDims.y*sizeof(ushort)));
    visualisationEngine->GetRaycastImage(scene, view, trackingState, points_device, normals_device, colors_device, objectIds_device);
    ITMSafeCall(cudaMemcpy(points_host, points_device, view->depth->noDims.x*view->depth->noDims.y*sizeof(Vector3f), cudaMemcpyDeviceToHost));
    ITMSafeCall(cudaMemcpy(normals_host, normals_device, view->depth->noDims.x*view->depth->noDims.y*sizeof(Vector3f), cudaMemcpyDeviceToHost));
    ITMSafeCall(cudaMemcpy(colors_host, colors_device, view->depth->noDims.x*view->depth->noDims.y*sizeof(Vector3f), cudaMemcpyDeviceToHost));
    ITMSafeCall(cudaMemcpy(objectIds_host, objectIds_device, view->depth->noDims.x*view->depth->noDims.y*sizeof(ushort), cudaMemcpyDeviceToHost));

    int *id_array_host = (int*)malloc(view->depth->noDims.x*view->depth->noDims.y*sizeof(int));
    PointCloudPtr_RGB_NORMAL cloud(new PointCloud_RGB_NORMAL);

    for(int i=0; i<view->depth->noDims.x*view->depth->noDims.y; i++){
      if(!((*(points_host+i))[0]==0&&(*(points_host+i))[1]==0&&(*(points_host+i))[2]==0)){
        Point_RGB_NORMAL pt;
        pt.x=(*(points_host+i))[0];
        pt.y=(*(points_host+i))[1];
        pt.z=(*(points_host+i))[2];
        pt.r=(*(colors_host+i))[0];
        pt.g=(*(colors_host+i))[1];
        pt.b=(*(colors_host+i))[2];
        pt.normal_x=(*(normals_host+i))[0];
        pt.normal_y=(*(normals_host+i))[1];
        pt.normal_z=(*(normals_host+i))[2];
        cloud->points.push_back(pt);
        *(id_array_host+i)=i;
      }else{
        *(id_array_host+i)=-1;
      }
    }

    PointCloudPtr_RGB_NORMAL cloudA (new PointCloud_RGB_NORMAL);
    PointCloudPtr_RGB_NORMAL cloudB (new PointCloud_RGB_NORMAL);

    MyPointCloud_RGB_NORMAL2PointCloud_RGB_NORMAL(myCloudOne, cloudA);
    MyPointCloud_RGB_NORMAL2PointCloud_RGB_NORMAL(myCloudTwo, cloudB);

    PointCloudPtr_RGB_NORMAL planeCloudA(new PointCloud_RGB_NORMAL);
    PointCloudPtr rect_cloudA(new PointCloud());
    PointCloudPtr_RGB_NORMAL remainingCloudA(new PointCloud_RGB_NORMAL);
    pcl::ModelCoefficients coefficientsA;
    PointCloudPtr_RGB_NORMAL planeCloudB(new PointCloud_RGB_NORMAL);
    PointCloudPtr rect_cloudB(new PointCloud());
    PointCloudPtr_RGB_NORMAL remainingCloudB(new PointCloud_RGB_NORMAL);
    pcl::ModelCoefficients coefficientsB;
    detect_table(cloudA, coefficientsA, planeCloudA, rect_cloudA, remainingCloudA);
    detect_table(cloudB, coefficientsB, planeCloudB, rect_cloudB, remainingCloudB);

    Eigen::Matrix4f matrix_transformA;
    Eigen::Matrix4f matrix_transformA_r;
    Eigen::Matrix4f matrix_transformB;
    Eigen::Matrix4f matrix_transformB_r;

    getTemTransformMatrix(coefficientsA, matrix_transformA, matrix_transformA_r);
    getTemTransformMatrix(coefficientsB, matrix_transformB, matrix_transformB_r);

    PointCloudPtr_RGB_NORMAL tabletopCloudA_tem(new PointCloud_RGB_NORMAL());
    PointCloudPtr_RGB_NORMAL tabletopCloudB_tem(new PointCloud_RGB_NORMAL());
    getCloudOnTable(remainingCloudA, rect_cloudA, matrix_transformA, matrix_transformA_r, tabletopCloudA_tem);
    getCloudOnTable(remainingCloudB, rect_cloudB, matrix_transformB, matrix_transformB_r, tabletopCloudB_tem);

    PointCloudPtr_RGB_NORMAL tabletopCloudA(new PointCloud_RGB_NORMAL());
    PointCloudPtr_RGB_NORMAL tabletopCloudB(new PointCloud_RGB_NORMAL());
    for(int i=0; i<tabletopCloudA_tem->size(); i++){
      if(!(tabletopCloudA_tem->points[i].z>1||tabletopCloudA_tem->points[i].x<-0.5||tabletopCloudA_tem->points[i].x>0.5||tabletopCloudA_tem->points[i].y<-0.5||tabletopCloudA_tem->points[i].y>0.5)){
        tabletopCloudA->push_back(tabletopCloudA_tem->points[i]);
      }
    }

    for(int i=0; i<tabletopCloudB_tem->size(); i++){
      if(!(tabletopCloudB_tem->points[i].z>1||tabletopCloudB_tem->points[i].x<-0.5||tabletopCloudB_tem->points[i].x>0.5||tabletopCloudB_tem->points[i].y<-0.5||tabletopCloudB_tem->points[i].y>0.5)){
        tabletopCloudB->push_back(tabletopCloudB_tem->points[i]);
      }
    }

    PointCloudPtr_RGB_NORMAL resultA (new PointCloud_RGB_NORMAL);
    PointCloudPtr_RGB_NORMAL resultB (new PointCloud_RGB_NORMAL);
    detect_change(tabletopCloudA, tabletopCloudB, resultA, resultB);

    PointCloudPtr_RGB object_cloud(new PointCloud_RGB);
    PointCloudPtr_RGB confidence_cloud(new PointCloud_RGB);

    vector<ushort> objectIndexs;
    int objectNum;
    Eigen::Vector3f range(0.5, 0.5, 1.0);
    vector<ObjectAttri> obas;

    updateSegmentObject(cloud, range, resultA, resultB, obas, object_cloud, confidence_cloud, objectIndexs, objectNum);

    for(int i=0; i<obas.size(); i++){
      objectMap[idCount+i+1] = obas[i];
    }

    KDtree tree;
    CUDA_KDTree GPU_tree;
    int max_tree_levels = 13; // play around with this value to get the best result

    vector<KDPoint> data(cloud->size());
    vector<KDPoint> queries(object_cloud->size());

    for(int i=0; i<cloud->size(); i++){
      data[i].coords[0] = cloud->points[i].x;
      data[i].coords[1] = cloud->points[i].y;
      data[i].coords[2] = cloud->points[i].z;
    }

    for(int i=0; i<object_cloud->size(); i++){
      queries[i].coords[0] = object_cloud->points[i].x;
      queries[i].coords[1] = object_cloud->points[i].y;
      queries[i].coords[2] = object_cloud->points[i].z;
    }

    vector <int> gpu_indexes;
    vector <float> gpu_dists;

    tree.Create(data, max_tree_levels);
    GPU_tree.CreateKDTree(tree.GetRoot(), tree.GetNumNodes(), data);
    GPU_tree.Search(queries, gpu_indexes, gpu_dists);

    PointCloudPtr_RGB colored_cloud0(new PointCloud_RGB);
    PointCloudPtr_RGB colored_cloud1(new PointCloud_RGB);
    vector<ushort> objectIndexs_new;
    for(int i=0; i<cloud->size(); i++){
      Point_RGB p_tem;
      p_tem.x=0;
      p_tem.y=0;
      p_tem.z=0;
      colored_cloud0->push_back(p_tem);
      colored_cloud1->push_back(p_tem);
      objectIndexs_new.push_back(9999);
    }

    for(int i=0; i<object_cloud->size(); i++){
      colored_cloud0->points[gpu_indexes[i]].x = object_cloud->points[i].x;
      colored_cloud0->points[gpu_indexes[i]].y = object_cloud->points[i].y;
      colored_cloud0->points[gpu_indexes[i]].z = object_cloud->points[i].z;
      colored_cloud0->points[gpu_indexes[i]].r = object_cloud->points[i].r;
      colored_cloud0->points[gpu_indexes[i]].g = object_cloud->points[i].g;
      colored_cloud0->points[gpu_indexes[i]].b = object_cloud->points[i].b;

      colored_cloud1->points[gpu_indexes[i]].x = confidence_cloud->points[i].x;
      colored_cloud1->points[gpu_indexes[i]].y = confidence_cloud->points[i].y;
      colored_cloud1->points[gpu_indexes[i]].z = confidence_cloud->points[i].z;
      colored_cloud1->points[gpu_indexes[i]].r = confidence_cloud->points[i].r;
      colored_cloud1->points[gpu_indexes[i]].g = confidence_cloud->points[i].g;
      colored_cloud1->points[gpu_indexes[i]].b = confidence_cloud->points[i].b;

      objectIndexs_new[gpu_indexes[i]] = objectIndexs[i];
    }

    //showPointCloud(colored_cloud, "colored_cloud");

    memset(objectIds_host, 0, view->depth->noDims.x*view->depth->noDims.y*sizeof(ushort));

    int count=0;
    for(int i=0; i<view->depth->noDims.x*view->depth->noDims.y; i++){
      if(*(id_array_host+i)!=-1){
        if(!(colored_cloud0->points[count].x==0&&colored_cloud0->points[count].y==0&&colored_cloud0->points[count].z==0)){
          (*(colors_host+i))[0] = colored_cloud0->points[count].r;
          (*(colors_host+i))[1] = colored_cloud0->points[count].g;
          (*(colors_host+i))[2] = colored_cloud0->points[count].b;

          if(objectIndexs_new[count] == 0){
            (*(objectIds_host+i)) = 1;
          }else{
            (*(objectIds_host+i)) = idCount + objectIndexs_new[count];
          }
        }
        else{
          (*(colors_host+i))[0] = -1;
          (*(colors_host+i))[1] = -1;
          (*(colors_host+i))[2] = -1;
        }

        count++;
      }
      else{
        (*(colors_host+i))[0] = -1;
        (*(colors_host+i))[1] = -1;
        (*(colors_host+i))[2] = -1;
      }
    }

    idCount += objectNum;

    ITMSafeCall(cudaMemcpy(colors_device, colors_host, view->depth->noDims.x*view->depth->noDims.y*sizeof(Vector3f), cudaMemcpyHostToDevice));
    //pcl::io::savePLYFileBinary("Data/scan1.ply", *colored_cloud);

    ITMSafeCall(cudaMemcpy(objectIds_device, objectIds_host, view->depth->noDims.x*view->depth->noDims.y*sizeof(ushort), cudaMemcpyHostToDevice));

    visualisationEngine->NewCreateICPMaps(scene, view, trackingState, colors_device, objectIds_device, 1);

    free(points_host);
    free(normals_host);
    free(objectIds_host);
    free(colors_host);
    free(id_array_host);
    points_host=NULL;
    normals_host=NULL;
    objectIds_host=NULL;
    id_array_host=NULL;
    colors_host=NULL;

    ITMSafeCall(cudaFree(points_device));
    ITMSafeCall(cudaFree(normals_device));
    ITMSafeCall(cudaFree(colors_device));
    ITMSafeCall(cudaFree(objectIds_device));
#endif
  }

  printf("interactedSegment finished.\n");
}

//hao modified it
void ITMMainEngine::getIntsObjectsIds(const ushort targetObjectId, vector<ushort> &objectIds){
  objectIds.push_back(targetObjectId);

  Vector3u targetObjectRGB;
  int countForTargetObjectPoints = 0;

  for(map<ushort,ObjectAttri>::iterator it = objectMap.begin(); it != objectMap.end(); ++it) {
    if(targetObjectId == it->first){
      targetObjectRGB.x = it->second.oR;
      targetObjectRGB.y = it->second.oG;
      targetObjectRGB.z = it->second.oB;
    }
  }

  getGlobalCloud(false);

  PointCloudPtr_RGB pc(new PointCloud_RGB);

  for(int i=0; i<globalCloud.mypoints.size(); i++){
    if(globalCloud.mypoints[i].id!=0&&globalCloud.mypoints[i].id!=1){
      Point_RGB pt;
      pt.x = globalCloud.mypoints[i].x;
      pt.y = globalCloud.mypoints[i].y;
      pt.z = globalCloud.mypoints[i].z;
      pt.r = globalCloud.mypoints[i].r;
      pt.g = globalCloud.mypoints[i].g;
      pt.b = globalCloud.mypoints[i].b;
      pc->push_back(pt);
    }
  }

  pcl::search::KdTree<Point_RGB>::Ptr tree (new pcl::search::KdTree<Point_RGB>);
  tree->setInputCloud (pc);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<Point_RGB> ec;
  ec.setClusterTolerance (0.010); // 1cm
  ec.setMinClusterSize (100);
  ec.setMaxClusterSize (500000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (pc);
  ec.extract (cluster_indices);

  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    PointCloudPtr_RGB cloud_cluster (new PointCloud_RGB);

    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++){
      if(pc->points[*pit].r == targetObjectRGB.x && pc->points[*pit].g == targetObjectRGB.y && pc->points[*pit].b == targetObjectRGB.z){
        countForTargetObjectPoints++;
        continue;
      }
      cloud_cluster->points.push_back (pc->points[*pit]); 
    } 

    if(countForTargetObjectPoints>100){
      map<Vector3u, int> map_tem;
      if(cloud_cluster->size()>100){
        for(int i=0; i<cloud_cluster->size(); i++){
          Vector3u color(cloud_cluster->points[i].r, cloud_cluster->points[i].g, cloud_cluster->points[i].b);
          if(map_tem.find(color)==map_tem.end()){
            map_tem[color] = 1;
          }
          else{
            map_tem[color]++;
          }
        }
      }

      for(map<Vector3u,int>::iterator it = map_tem.begin(); it != map_tem.end(); ++it) {
        if(it->second > 100){
          for(map<ushort,ObjectAttri>::iterator it1 = objectMap.begin(); it1 != objectMap.end(); ++it1) {
            if(it->first.x == it1->second.oR && it->first.y == it1->second.oG && it->first.z == it1->second.oB){
              objectIds.push_back(it1->first);
            }
          }
        }
      }

      break;
    }
  }
}

//hao modified it
void ITMMainEngine::preWorkForIntSeg(){
  saveViewPoints();

  ushort targetObjectId = 2;
  vector<ushort> objectIds;
  getIntsObjectsIds(targetObjectId, objectIds);

  vector<ushort> newIds;
  vector<uchar> newRs;
  vector<uchar> newGs;
  vector<uchar> newBs;
  for(int i=0; i<objectIds.size(); i++){
    newIds.push_back(0);
    newRs.push_back(255);
    newGs.push_back(255);
    newBs.push_back(255);
  }

  changeObjectIds(objectIds, newIds, newRs, newGs, newBs);

  printf("preWorkForIntSeg finished.\n");
}

//hao modified it
void ITMMainEngine::switchShowModel(uchar mode){
  ITMVoxelIndex::IndexData *hashData_host = new ITMVoxelIndex::IndexData;
  ITMVoxel *voxels = (ITMVoxel*)malloc(SDF_LOCAL_BLOCK_NUM*SDF_BLOCK_SIZE3 * sizeof(ITMVoxel));

  //printf("aaaaaa\n");

  bool flag = false;
#ifndef COMPILE_WITHOUT_CUDA
  flag=true;
#endif
  if(flag){
    ITMSafeCall(cudaMemcpy(hashData_host, scene->index.getIndexData(), sizeof(ITMVoxelIndex::IndexData), cudaMemcpyDeviceToHost));
    ITMSafeCall(cudaMemcpy(voxels, scene->localVBA.GetVoxelBlocks(), SDF_LOCAL_BLOCK_NUM*SDF_BLOCK_SIZE3*sizeof(ITMVoxel), cudaMemcpyDeviceToHost));
  }
  else{
    memcpy(hashData_host, scene->index.getIndexData(), sizeof(ITMVoxelIndex::IndexData));
    memcpy(voxels, scene->localVBA.GetVoxelBlocks(), SDF_LOCAL_BLOCK_NUM*SDF_BLOCK_SIZE3 * sizeof(ITMVoxel));
  }
  //printf("bbbbbb\n");
  const ITMHashEntry *hashTable = hashData_host->entries_all;

  for(int i=0; i<SDF_BUCKET_NUM * SDF_ENTRY_NUM_PER_BUCKET + SDF_EXCESS_LIST_SIZE; i++){
    const ITMHashEntry &hashEntry = hashTable[i];

    if(hashEntry.ptr >= 0){
      for(int j=0; j<SDF_BLOCK_SIZE3; j++){
        ITMVoxel res = voxels[(hashEntry.ptr * SDF_BLOCK_SIZE3) + j];

        switch(mode){
        case 0://show object result
          {
            for(map<ushort,ObjectAttri>::iterator it = objectMap.begin(); it != objectMap.end(); ++it) {
              if(res.id == it->first){
                voxels[(hashEntry.ptr * SDF_BLOCK_SIZE3) + j].r = it->second.oR;
                voxels[(hashEntry.ptr * SDF_BLOCK_SIZE3) + j].g = it->second.oG;
                voxels[(hashEntry.ptr * SDF_BLOCK_SIZE3) + j].b = it->second.oB;
              }
            }
            break;
          }
        case 1://show confidence result
          {
            for(map<ushort,ObjectAttri>::iterator it = objectMap.begin(); it != objectMap.end(); ++it) {
              if(res.id == it->first){
                voxels[(hashEntry.ptr * SDF_BLOCK_SIZE3) + j].r = it->second.cR;
                voxels[(hashEntry.ptr * SDF_BLOCK_SIZE3) + j].g = it->second.cG;
                voxels[(hashEntry.ptr * SDF_BLOCK_SIZE3) + j].b = it->second.cB;
              }
            }
            break;
          }
        }
      }
    }
  }

  if(flag){
    ITMSafeCall(cudaMemcpy(scene->localVBA.GetVoxelBlocks(), voxels, SDF_LOCAL_BLOCK_NUM*SDF_BLOCK_SIZE3*sizeof(ITMVoxel), cudaMemcpyHostToDevice));
  }
  else{
    memcpy(scene->localVBA.GetVoxelBlocks(), voxels, SDF_LOCAL_BLOCK_NUM*SDF_BLOCK_SIZE3 * sizeof(ITMVoxel));
  }

  //printf("cccccc\n");
  free(voxels);
  free(hashData_host);
  voxels=NULL;
  hashData_host=NULL;
}