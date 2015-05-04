// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMMainEngine.h"

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
    visualisationEngine->RealTimeSegment(scene, view, trackingState, points_device, normals_device);
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

    float voxel_resolution = 0.006f;
    float seed_resolution = 0.06f;
    float color_importance = 0;//0.2f
    float spatial_importance = 0.4f;
    float normal_importance = 1.0f;

    PointCloudT::Ptr colored_cloud(new PointCloudT);
    vector<MyPointCloud_RGB_NORMAL> patch_clouds;
    PointNCloudT::Ptr normal_cloud(new PointNCloudT);
    VCCS_over_segmentation(cloud,voxel_resolution,seed_resolution,color_importance,spatial_importance,normal_importance,patch_clouds,colored_cloud,normal_cloud);

    Vector3f *colors_host = (Vector3f*)malloc(view->depth->noDims.x*view->depth->noDims.y*sizeof(Vector3f));

    int count=0;
    for(int i=0; i<view->depth->noDims.x*view->depth->noDims.y; i++){
      if(*(id_array_host+i)!=-1){
        (*(colors_host+i))[0] = colored_cloud->points[count].r;
        (*(colors_host+i))[1] = colored_cloud->points[count].g;
        (*(colors_host+i))[2] = colored_cloud->points[count].b;
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

    visualisationEngine->NewCreateICPMaps(scene, view, trackingState, colors_device);

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
    visualisationEngine->RealTimeSegment(scene, view, trackingState, points_device, normals_device);
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

    PointCloudPtr_RGB segmented_cloud(new PointCloud_RGB);
    segmentObject(cloud, segmented_cloud);
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

    visualisationEngine->NewCreateICPMaps(scene, view, trackingState, colors_device);

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
void ITMMainEngine::saveViewPoints(){
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
    visualisationEngine->RealTimeSegment(scene, view, trackingState, points_device, normals_device);
    ITMSafeCall(cudaMemcpy(points_host, points_device, view->depth->noDims.x*view->depth->noDims.y*sizeof(Vector3f), cudaMemcpyDeviceToHost));
    ITMSafeCall(cudaMemcpy(normals_host, normals_device, view->depth->noDims.x*view->depth->noDims.y*sizeof(Vector3f), cudaMemcpyDeviceToHost));

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
      }
    }

    pcl::io::savePLYFileASCII("Data/currentView.ply", *cloud);

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
void ITMMainEngine::savePoints(vector<Vector3f> &points)
{
  //visualisationEngine->GetAllPoints(scene);
  PointCloudPtr cloud(new PointCloud);

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
        if(value<mu/16.0&&value>-mu/16.0){
          //cout<<"value:"<<value<<endl;

          Vector3f p;
          float voxelSize = scene->sceneParams->voxelSize;
          p.x=(hashEntry.pos.x+j%SDF_BLOCK_SIZE)*voxelSize+0.5f*voxelSize;
          p.y=(hashEntry.pos.y+(j%(SDF_BLOCK_SIZE*SDF_BLOCK_SIZE))/SDF_BLOCK_SIZE)*voxelSize+0.5f*voxelSize;
          p.z=(hashEntry.pos.z+(j/(SDF_BLOCK_SIZE*SDF_BLOCK_SIZE)))*voxelSize+0.5f*voxelSize;
          points.push_back(p);

          Point pt;
          pt.x=p.x;
          pt.y=p.y;
          pt.z=p.z;
          /* pt.r=res.clr[0];
          pt.g=res.clr[1];
          pt.b=res.clr[2];*/
          cloud->points.push_back(pt);
        }
      }
    }
  }
  printf("points.size:%d\n", points.size());


  //for(int i=0; i<points.size(); i++){
  //  cloud->points.push_back(Point(points[i].x, points[i].y, points[i].z));
  //}

  //showPointCloud2(cloud, "cloud");

  pcl::io::savePLYFileBinary("Data/scan.ply", *cloud);

  free(voxels);
  voxels=NULL;



  //printf("aaaaaaaaaa\n");

  //int count = 0;

  //const ITMHashEntry *hashTable = scene->index.getIndexData()->entries_all;

  ////printf("sizeof(scene->index.getIndexData()->entries_all)/(SDF_BUCKET_NUM * SDF_ENTRY_NUM_PER_BUCKET + SDF_EXCESS_LIST_SIZE):%d\n", sizeof(scene->index.getIndexData()->entries_all)/(SDF_BUCKET_NUM * SDF_ENTRY_NUM_PER_BUCKET + SDF_EXCESS_LIST_SIZE));

  //float mu = scene->sceneParams->mu;

  // cout<<"mu:"<<mu<<endl;

  //printf("bbbbbbbbbbb\n");

  //for(int i=0; i<SDF_BUCKET_NUM * SDF_ENTRY_NUM_PER_BUCKET + SDF_EXCESS_LIST_SIZE; i++){
  //  const ITMHashEntry &hashEntry = hashTable[i];

  //  printf("ccccccccccc\n");
  //  /* printf("sizeof(hashTable[i]):%d\n", sizeof(hashTable[i]));
  //  printf("sizeof(hashEntry):%d\n", sizeof(hashEntry));
  //  printf("sizeof(ITMHashEntry):%d\n", sizeof(ITMHashEntry));
  //  printf("sizeof(int):%d\n", sizeof(int));
  //  printf("sizeof(short):%d\n", sizeof(short));
  //  printf("sizeof(Vector3s):%d\n", sizeof(Vector3s));
  //  printf("sizeof(hashEntry.offset):%d\n", sizeof(hashEntry.offset));
  //  printf("sizeof(hashEntry.ptr):%d\n", sizeof(hashEntry.ptr));
  //  printf("sizeof(hashEntry.pos):%d\n", sizeof(hashEntry.pos));*/

  //  cout<<"hashEntry.ptr:"<<hashEntry.ptr<<endl;

  //  if(hashEntry.ptr >= 0){
  //    printf("ggggggggggg\n");

  //    for(int j=0; j<SDF_BLOCK_SIZE*SDF_BLOCK_SIZE*SDF_BLOCK_SIZE; j++){
  //      printf("dddddddddddd\n");

  //      ITMVoxel *voxels = scene->globalCache->GetStoredVoxelBlocks();
  //      printf("eeeeeeeeeeee\n");
  //      ITMVoxel res = voxels[(hashEntry.ptr * SDF_BLOCK_SIZE3) + j];
  //      printf("ffffffffffff\n");
  //      float value = ITMVoxel::SDF_valueToFloat(res.sdf);

  //      if(value<mu&&value<-mu){
  //        count++;
  //        printf("cout:%d\n", cout);

  //        Vector3f p;
  //        p.x=0;
  //        p.y=0;
  //        p.z=0;
  //        points.push_back(p);
  //      }
  //    }
  //  }

  //  printf("hhhhhhhhhhhh\n");
  //}

  //printf("points.size:%d\n", points.size());
}
