#include "object_segmentation.h"

//segment object
void segmentObject(PointCloudPtr_RGB_NORMAL source_cloud, PointCloudPtr_RGB result_cloud){
  CPointCloudAnalysis cPointCloudAnalysis;

  /******************detect table************************/
  PointCloudPtr_RGB_NORMAL tabletopCloud(new PointCloud_RGB_NORMAL());
  PointCloudPtr_RGB_NORMAL planeCloud(new PointCloud_RGB_NORMAL());
  PointCloudPtr rect_cloud(new PointCloud());
  PointCloudPtr_RGB_NORMAL remainingCloud(new PointCloud_RGB_NORMAL());
  pcl::ModelCoefficients coefficients;

  detect_table(source_cloud, coefficients, planeCloud, rect_cloud, remainingCloud);

  MyPointCloud_RGB_NORMAL tablePoint;
  for(int i=0;i<planeCloud->size();i++){
    MyPt_RGB_NORMAL point;
    point.x = planeCloud->at(i).x;
    point.y = planeCloud->at(i).y;
    point.z = planeCloud->at(i).z;
    point.r = 255;
    point.g = 255;
    point.b = 255;
    tablePoint.mypoints.push_back(point);

    Point_RGB pt_rgb;
    pt_rgb.x = planeCloud->at(i).x;
    pt_rgb.y = planeCloud->at(i).y;
    pt_rgb.z = planeCloud->at(i).z;
    pt_rgb.r = 255;
    pt_rgb.g = 255;
    pt_rgb.b = 255;
    result_cloud->points.push_back(pt_rgb);
  }

  cPointCloudAnalysis.tablePoint = tablePoint;

  Eigen::Matrix4f matrix_transform;
  Eigen::Matrix4f matrix_transform_r;

  getTemTransformMatrix(coefficients, matrix_transform, matrix_transform_r);

  getCloudOnTable(remainingCloud, rect_cloud, matrix_transform, matrix_transform_r, tabletopCloud);

  float voxel_resolution = 0.004f;
  float seed_resolution = 0.04f;
  float color_importance = 0.2f;
  float spatial_importance = 0.4f;
  float normal_importance = 1.0f;

  /******************Euclidean Cluster Extraction************************/
  std::vector<MyPointCloud_RGB_NORMAL> cluster_points;

  object_seg_ECE(tabletopCloud, cluster_points);

  for(int i=0;i<cluster_points.size();i++){
    if(cluster_points.at(i).mypoints.size()<200){
      continue;
    }

    PointCloudT::Ptr colored_cloud(new PointCloudT);
    vector<MyPointCloud_RGB_NORMAL> patch_clouds;
    PointNCloudT::Ptr normal_cloud(new PointNCloudT);

    PointCloudPtr_RGB_NORMAL ct(new PointCloud_RGB_NORMAL);
    MyPointCloud_RGB_NORMAL2PointCloud_RGB_NORMAL(cluster_points.at(i), ct);

    VCCS_over_segmentation(ct,voxel_resolution,seed_resolution,color_importance,spatial_importance,normal_importance,patch_clouds,colored_cloud,normal_cloud);

    //add normal, point cloud, cluster patch num
    for(int i=0;i<patch_clouds.size();i++)
    {
      Normalt nor;
      pcl::PointNormal pn=normal_cloud->at(i);
      nor.normal_x = pn.normal_x;
      nor.normal_y = pn.normal_y;
      nor.normal_z = pn.normal_z;
      double normalizeValue = pow(nor.normal_x,2) + pow(nor.normal_y,2) + pow(nor.normal_z,2);
      nor.normal_x /= normalizeValue;
      nor.normal_y /= normalizeValue;
      nor.normal_z /= normalizeValue;
      cPointCloudAnalysis.vecPatcNormal.push_back(nor);
    }

    cPointCloudAnalysis.vecPatchPoint.insert(cPointCloudAnalysis.vecPatchPoint.end(),patch_clouds.begin(),patch_clouds.end());
    cPointCloudAnalysis.clusterPatchNum.push_back(patch_clouds.size());
  }

  std::cout<<"test graph cut" <<std::endl;	
  ofstream outFileg("Output\\RunGraphCut.txt");
  outFileg << "let's begin :) " << endl;

  /////////////////////////////////////run graph cut
  cPointCloudAnalysis.MainStep(true,0);
  outFileg << "MainStep finished :)   " << endl;

  srand((unsigned)time(0));
  for(int i = 0; i < cPointCloudAnalysis.vecAreaInterest.size();i++)
  {
    if(!cPointCloudAnalysis.vecAreaInterest[i].validFlag)	continue;
    for(int j = 0; j < cPointCloudAnalysis.vecAreaInterest[i].vecObjectHypo.size();j++)
    {
      double r,g,b;
      r = double(rand()%255);
      g = double(rand()%255);
      b = double(rand()%255);

      for(int k = 0; k < cPointCloudAnalysis.vecAreaInterest[i].vecObjectHypo[j].patchIndex.size();k++)
      {
        int patchIndex = cPointCloudAnalysis.vecAreaInterest[i].vecObjectHypo[j].patchIndex[k];
        for(int m = 0; m < cPointCloudAnalysis.vecAreaInterest[i].vecPatchPoint[patchIndex].mypoints.size();m++)
        {
          MyPoint_RGB_NORMAL point = cPointCloudAnalysis.vecAreaInterest[i].vecPatchPoint[patchIndex].mypoints[m];

          Point_RGB po;
          po.x = point.x;
          po.y = point.y;
          po.z = point.z;
          po.r = r;
          po.g = g;
          po.b = b;

          result_cloud->push_back(po);
        }
      }
    }
  }
}

//over segment object
void overSegmentObject(PointCloudPtr_RGB_NORMAL source_cloud, PointCloudPtr_RGB_NORMAL result_cloud){
  CPointCloudAnalysis cPointCloudAnalysis;

  /******************detect table************************/
  PointCloudPtr_RGB_NORMAL tabletopCloud(new PointCloud_RGB_NORMAL());
  PointCloudPtr_RGB_NORMAL planeCloud(new PointCloud_RGB_NORMAL());
  PointCloudPtr rect_cloud(new PointCloud());
  PointCloudPtr_RGB_NORMAL remainingCloud(new PointCloud_RGB_NORMAL());
  pcl::ModelCoefficients coefficients;

  detect_table(source_cloud, coefficients, planeCloud, rect_cloud, remainingCloud);

  appendCloud_RGB_NORMAL(planeCloud, result_cloud);

  Eigen::Matrix4f matrix_transform;
  Eigen::Matrix4f matrix_transform_r;

  getTemTransformMatrix(coefficients, matrix_transform, matrix_transform_r);

  getCloudOnTable(remainingCloud, rect_cloud, matrix_transform, matrix_transform_r, tabletopCloud);

  float voxel_resolution = 0.004f;
  float seed_resolution = 0.04f;
  float color_importance = 0.2f;
  float spatial_importance = 0.4f;
  float normal_importance = 1.0f;

  /******************Euclidean Cluster Extraction************************/
  std::vector<MyPointCloud_RGB_NORMAL> cluster_points;

  object_seg_ECE(tabletopCloud, cluster_points);

  for(int i=0;i<cluster_points.size();i++){
    if(cluster_points.at(i).mypoints.size()<200){
      continue;
    }

    PointCloudT::Ptr colored_cloud(new PointCloudT);
    vector<MyPointCloud_RGB_NORMAL> patch_clouds;
    PointNCloudT::Ptr normal_cloud(new PointNCloudT);

    PointCloudPtr_RGB_NORMAL ct(new PointCloud_RGB_NORMAL);
    MyPointCloud_RGB_NORMAL2PointCloud_RGB_NORMAL(cluster_points.at(i), ct);

    VCCS_over_segmentation(ct,voxel_resolution,seed_resolution,color_importance,spatial_importance,normal_importance,patch_clouds,colored_cloud,normal_cloud);

    for(int i=0; i<colored_cloud->size(); i++){
      Point_RGB_NORMAL pt;
      pt.x = colored_cloud->points[i].x;
      pt.y = colored_cloud->points[i].y;
      pt.z = colored_cloud->points[i].z;
      pt.r = colored_cloud->points[i].r;
      pt.g = colored_cloud->points[i].g;
      pt.b = colored_cloud->points[i].b;
      pt.normal_x = ct->points[i].normal_x;
      pt.normal_y = ct->points[i].normal_x;
      pt.normal_z = ct->points[i].normal_z;
      result_cloud->points.push_back(pt);
    }
  }
}