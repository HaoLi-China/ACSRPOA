#include "object_segmentation.h"

//get color
void GetColour(double v,double vmin,double vmax,double &r,double &g,double &b)
{
  r = g = b =1.0;
  double dv;

  if (v < vmin)
    v = vmin;
  if (v > vmax)
    v = vmax;
  dv = vmax - vmin;

  if (v < (vmin + 0.25 * dv)) {
    r = 0;
    g = 4 * (v - vmin) / dv;
  } else if (v < (vmin + 0.5 * dv)) {
    r = 0;
    b = 1 + 4 * (vmin + 0.25 * dv - v) / dv;
  } else if (v < (vmin + 0.75 * dv)) {
    r = 4 * (v - vmin - 0.5 * dv) / dv;
    b = 0;
  } else {
    g = 1 + 4 * (vmin + 0.75 * dv - v) / dv;
    b = 0;
  }
}

//shrink cloud range
void shrinkCloudRange(PointCloudPtr_RGB_NORMAL source_cloud, const float range_x, const float range_y, const float range_z, PointCloudPtr_RGB_NORMAL result_cloud){
  for(int i=0; i<source_cloud->size(); i++){
    if(!(source_cloud->points[i].z>range_z||source_cloud->points[i].x<-range_x||source_cloud->points[i].x>range_x||source_cloud->points[i].y<-range_y||source_cloud->points[i].y>range_y)){
      result_cloud->push_back(source_cloud->points[i]);
    }
  }
}

//segment special object
void segmentSepcialObjects(PointCloudPtr_RGB_NORMAL source_cloud, Eigen::Vector3f range, vector<ObjectAttri> &obas, PointCloudPtr_RGB object_cloud, PointCloudPtr_RGB confidence_cloud, vector<ushort> &objectIndexs, int &objectNum){
  objectNum = 0;

  CPointCloudAnalysis cPointCloudAnalysis;

  PointCloudPtr_RGB_NORMAL shrinked_cloud(new PointCloud_RGB_NORMAL);
  shrinkCloudRange(source_cloud, range[0], range[1], range[2], shrinked_cloud);

  /******************detect table************************/
  PointCloudPtr_RGB_NORMAL tabletopCloud(new PointCloud_RGB_NORMAL());
  PointCloudPtr_RGB_NORMAL planeCloud(new PointCloud_RGB_NORMAL());
  PointCloudPtr rect_cloud(new PointCloud());
  PointCloudPtr_RGB_NORMAL remainingCloud(new PointCloud_RGB_NORMAL());
  pcl::ModelCoefficients coefficients;

  detect_table(shrinked_cloud, coefficients, planeCloud, rect_cloud, remainingCloud);

  MyPointCloud_RGB_NORMAL tablePoint;
  for(int i=0;i<planeCloud->size();i++){
    MyPt_RGB_NORMAL point;
    point.x = planeCloud->at(i).x;
    point.y = planeCloud->at(i).y;
    point.z = planeCloud->at(i).z;
    point.r = 255;
    point.g = 255;
    point.b = 0;
    tablePoint.mypoints.push_back(point);
  }

  cPointCloudAnalysis.tablePoint = tablePoint;
  cPointCloudAnalysis.sup_plane_normal << coefficients.values[0], coefficients.values[1], coefficients.values[2];
  cPointCloudAnalysis.sup_plane_normal.normalize();

  Eigen::Matrix4f matrix_transform;
  Eigen::Matrix4f matrix_transform_r;

  getTemTransformMatrix(coefficients, matrix_transform, matrix_transform_r);

  getCloudOnTable(remainingCloud, rect_cloud, matrix_transform, matrix_transform_r, tabletopCloud);

  float voxel_resolution = 0.006f;
  float seed_resolution = 0.06f;
  float color_importance = 0;
  float spatial_importance = 0.4f;
  float normal_importance = 1.0f;

  /******************Euclidean Cluster Extraction************************/
  std::vector<MyPointCloud_RGB_NORMAL> cluster_points;

  object_seg_ECE(tabletopCloud, cluster_points);

  for(int i=0;i<cluster_points.size();i++){
    if(cluster_points.at(i).mypoints.size()<200){
      continue;
    }

    double sumcolorR = 0;
    double sumcolorG = 0;
    double sumcolorB = 0;
    for(int j=0; j<cluster_points.at(i).mypoints.size(); j++){
      sumcolorR+=(cluster_points.at(i).mypoints[j].r/255.0);
      sumcolorG+=(cluster_points.at(i).mypoints[j].g/255.0);
      sumcolorB+=(cluster_points.at(i).mypoints[j].b/255.0);
    }

    sumcolorR/=cluster_points.at(i).mypoints.size();
    sumcolorG/=cluster_points.at(i).mypoints.size();
    sumcolorB/=cluster_points.at(i).mypoints.size();

    if(!(sumcolorR<=1.0&&sumcolorR>=0.98&&sumcolorG<=1.0&&sumcolorG>=0.98&&sumcolorB<=1.0&&sumcolorB>=0.98)){
     continue;
    }

    PointCloudT::Ptr colored_cloud(new PointCloudT);
    vector<MyPointCloud_RGB_NORMAL> patch_clouds;
    PointNCloudT::Ptr normal_cloud(new PointNCloudT);

    PointCloudPtr_RGB_NORMAL ct(new PointCloud_RGB_NORMAL);
    MyPointCloud_RGB_NORMAL2PointCloud_RGB_NORMAL(cluster_points.at(i), ct);

    VCCS_over_segmentation(ct,voxel_resolution,seed_resolution,color_importance,spatial_importance,normal_importance,patch_clouds,colored_cloud,normal_cloud);

    //PointCloudPtr_RGB pc_tem(new PointCloud_RGB);
    //for(int i=0; i<colored_cloud->size(); i++){
    //  Point_RGB pt;
    //  pt.x = colored_cloud->points[i].x;
    //  pt.y = colored_cloud->points[i].y;
    //  pt.z = colored_cloud->points[i].z;
    //  pt.r = colored_cloud->points[i].r;
    //  pt.g = colored_cloud->points[i].g;
    //  pt.b = colored_cloud->points[i].b;
    //  pc_tem->push_back(pt);
    //}
    //showPointCloud(pc_tem, "colored_cloud");

    //add normal, point cloud, cluster patch num
    for(int j=0;j<patch_clouds.size();j++)
    {
      Normalt nor;
      pcl::PointNormal pn=normal_cloud->at(j);
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
      objectNum++;

      double r,g,b;
      r = double(rand()%255);
      g = double(rand()%255);
      b = double(rand()%255);

      ObjectAttri oba = {r, g, b, 0, 0, 0, 0};
      obas.push_back(oba);

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

          object_cloud->push_back(po);
          objectIndexs.push_back(objectNum);
        }
      }
    }
  }

  int count_tem = 0;
  for(int i = 0; i < cPointCloudAnalysis.vecAreaInterest.size();i++)
  {
    cPointCloudAnalysis.vecAreaInterest[i].ComputeScore();
    if(!cPointCloudAnalysis.vecAreaInterest[i].validFlag)	continue;
    for(int j = 0; j < cPointCloudAnalysis.vecAreaInterest[i].vecObjectHypo.size();j++)
    {
      double rObj,gObj,bObj;
      GetColour(double(650) - cPointCloudAnalysis.vecAreaInterest[i].vecObjectHypo[j].objectness,300,350,rObj,gObj,bObj);
      rObj *= 255;
      gObj *= 255;
      bObj *= 255;

      if(rObj>255){
        rObj = 255;
      }

      if(gObj>255){
        gObj = 255;
      }

      if(bObj>255){
        bObj = 255;
      }

      obas[count_tem].cR = rObj;
      obas[count_tem].cG = gObj;
      obas[count_tem].cB = bObj;
      obas[count_tem].confidence = cPointCloudAnalysis.vecAreaInterest[i].vecObjectHypo[j].objectness;
      count_tem++;

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
          po.r = rObj;
          po.g = gObj;
          po.b = bObj;

          confidence_cloud->push_back(po);
        }
      }
    }
  }
  //showPointCloud(confidence_cloud, "confidence_cloud");
}

//segment object
void segmentObject(PointCloudPtr_RGB_NORMAL source_cloud, Eigen::Vector3f range, vector<ObjectAttri> &obas, PointCloudPtr_RGB object_cloud, PointCloudPtr_RGB confidence_cloud, vector<ushort> &objectIndexs, int &objectNum){
  objectNum = 0;
  
  CPointCloudAnalysis cPointCloudAnalysis;

  PointCloudPtr_RGB_NORMAL shrinked_cloud(new PointCloud_RGB_NORMAL);
  shrinkCloudRange(source_cloud, range[0], range[1], range[2], shrinked_cloud);

  /******************detect table************************/
  PointCloudPtr_RGB_NORMAL tabletopCloud(new PointCloud_RGB_NORMAL());
  PointCloudPtr_RGB_NORMAL planeCloud(new PointCloud_RGB_NORMAL());
  PointCloudPtr rect_cloud(new PointCloud());
  PointCloudPtr_RGB_NORMAL remainingCloud(new PointCloud_RGB_NORMAL());
  pcl::ModelCoefficients coefficients;

  detect_table(shrinked_cloud, coefficients, planeCloud, rect_cloud, remainingCloud);

  MyPointCloud_RGB_NORMAL tablePoint;
  for(int i=0;i<planeCloud->size();i++){
    MyPt_RGB_NORMAL point;
    point.x = planeCloud->at(i).x;
    point.y = planeCloud->at(i).y;
    point.z = planeCloud->at(i).z;
    point.r = 255;
    point.g = 255;
    point.b = 0;
    tablePoint.mypoints.push_back(point);

    Point_RGB pt_rgb;
    pt_rgb.x = planeCloud->at(i).x;
    pt_rgb.y = planeCloud->at(i).y;
    pt_rgb.z = planeCloud->at(i).z;
    pt_rgb.r = 255;
    pt_rgb.g = 255;
    pt_rgb.b = 0;
    object_cloud->points.push_back(pt_rgb);
    confidence_cloud->points.push_back(pt_rgb);
    objectIndexs.push_back(0);
  }

  cPointCloudAnalysis.tablePoint = tablePoint;
  cPointCloudAnalysis.sup_plane_normal << coefficients.values[0], coefficients.values[1], coefficients.values[2];
  cPointCloudAnalysis.sup_plane_normal.normalize();

  Eigen::Matrix4f matrix_transform;
  Eigen::Matrix4f matrix_transform_r;

  getTemTransformMatrix(coefficients, matrix_transform, matrix_transform_r);

  getCloudOnTable(remainingCloud, rect_cloud, matrix_transform, matrix_transform_r, tabletopCloud);

  float voxel_resolution = 0.006f;
  float seed_resolution = 0.06f;
  float color_importance = 0;
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

    //PointCloudPtr_RGB pc_tem(new PointCloud_RGB);
    //for(int i=0; i<colored_cloud->size(); i++){
    //  Point_RGB pt;
    //  pt.x = colored_cloud->points[i].x;
    //  pt.y = colored_cloud->points[i].y;
    //  pt.z = colored_cloud->points[i].z;
    //  pt.r = colored_cloud->points[i].r;
    //  pt.g = colored_cloud->points[i].g;
    //  pt.b = colored_cloud->points[i].b;
    //  pc_tem->push_back(pt);
    //}
    //showPointCloud(pc_tem, "colored_cloud");

    //add normal, point cloud, cluster patch num
    for(int j=0;j<patch_clouds.size();j++)
    {
      Normalt nor;
      pcl::PointNormal pn=normal_cloud->at(j);
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
      objectNum++;

      double r,g,b;
      r = double(rand()%255);
      g = double(rand()%255);
      b = double(rand()%255);

      ObjectAttri oba = {r, g, b, 0, 0, 0, 0};
      obas.push_back(oba);

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

          object_cloud->push_back(po);
          objectIndexs.push_back(objectNum);
        }
      }
    }
  }

  int count_tem = 0;
  for(int i = 0; i < cPointCloudAnalysis.vecAreaInterest.size();i++)
  {
    cPointCloudAnalysis.vecAreaInterest[i].ComputeScore();
    if(!cPointCloudAnalysis.vecAreaInterest[i].validFlag)	continue;
    for(int j = 0; j < cPointCloudAnalysis.vecAreaInterest[i].vecObjectHypo.size();j++)
    {
      double rObj,gObj,bObj;
      GetColour(double(650) - cPointCloudAnalysis.vecAreaInterest[i].vecObjectHypo[j].objectness,300,350,rObj,gObj,bObj);
      rObj *= 255;
      gObj *= 255;
      bObj *= 255;

      if(rObj>255){
         rObj = 255;
      }

      if(gObj>255){
        gObj = 255;
      }

      if(bObj>255){
        bObj = 255;
      }

      obas[count_tem].cR = rObj;
      obas[count_tem].cG = gObj;
      obas[count_tem].cB = bObj;
      obas[count_tem].confidence = cPointCloudAnalysis.vecAreaInterest[i].vecObjectHypo[j].objectness;
      count_tem++;

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
          po.r = rObj;
          po.g = gObj;
          po.b = bObj;

          confidence_cloud->push_back(po);
        }
      }
    }
  }

  //showPointCloud(confidence_cloud, "confidence_cloud");
}

//over segment object
void overSegmentObject(PointCloudPtr_RGB_NORMAL source_cloud, PointCloudPtr_RGB result_cloud){
  PointCloudPtr_RGB_NORMAL shrinked_cloud(new PointCloud_RGB_NORMAL);
  shrinkCloudRange(source_cloud, 0.5, 0.5, 1.0, shrinked_cloud);

  /******************detect table************************/
  PointCloudPtr_RGB_NORMAL tabletopCloud(new PointCloud_RGB_NORMAL());
  PointCloudPtr_RGB_NORMAL planeCloud(new PointCloud_RGB_NORMAL());
  PointCloudPtr rect_cloud(new PointCloud());
  PointCloudPtr_RGB_NORMAL remainingCloud(new PointCloud_RGB_NORMAL());
  pcl::ModelCoefficients coefficients;

  detect_table(shrinked_cloud, coefficients, planeCloud, rect_cloud, remainingCloud);

  for(int i=0;i<planeCloud->size();i++){
    Point_RGB pt_rgb;
    pt_rgb.x = planeCloud->at(i).x;
    pt_rgb.y = planeCloud->at(i).y;
    pt_rgb.z = planeCloud->at(i).z;
    pt_rgb.r = 255;
    pt_rgb.g = 255;
    pt_rgb.b = 0;
    result_cloud->points.push_back(pt_rgb);
  }

  Eigen::Matrix4f matrix_transform;
  Eigen::Matrix4f matrix_transform_r;

  getTemTransformMatrix(coefficients, matrix_transform, matrix_transform_r);

  getCloudOnTable(remainingCloud, rect_cloud, matrix_transform, matrix_transform_r, tabletopCloud);

  float voxel_resolution = 0.004f;
  float seed_resolution = 0.04f;
  float color_importance = 0;
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

    for(int j=0; j<colored_cloud->size(); j++){
      Point_RGB pt;
      pt.x = colored_cloud->points[j].x;
      pt.y = colored_cloud->points[j].y;
      pt.z = colored_cloud->points[j].z;
      pt.r = colored_cloud->points[j].r;
      pt.g = colored_cloud->points[j].g;
      pt.b = colored_cloud->points[j].b;
      /*pt.normal_x = ct->points[j].normal_x;
      pt.normal_y = ct->points[j].normal_x;
      pt.normal_z = ct->points[j].normal_z;*/
      result_cloud->points.push_back(pt);
    }
  }
}

//update segment object
void updateSegmentObject(PointCloudPtr_RGB_NORMAL source_cloud, Eigen::Vector3f range, PointCloudPtr_RGB_NORMAL change_cloudA, PointCloudPtr_RGB_NORMAL change_cloudB, vector<ObjectAttri> &obas, PointCloudPtr_RGB object_cloud, PointCloudPtr_RGB confidence_cloud, vector<ushort> &objectIndexs, int &objectNum){
  objectNum = 0;
  PointCloudPtr_RGB_NORMAL shrinked_cloud(new PointCloud_RGB_NORMAL);
  shrinkCloudRange(source_cloud, range[0], range[1], range[2], shrinked_cloud);

  std::vector<MyPointCloud_RGB_NORMAL> change_cluster;
  object_seg_ECE(change_cloudB, change_cluster);

  //cout<<"change_cluster.size:"<<change_cluster.size()<<endl;
  /*cout<<"change_cloudB->points[0].r:"<<change_cloudB->points[0].r<<endl;
  cout<<"change_cloudB->points[0].g:"<<change_cloudB->points[0].g<<endl;
  cout<<"change_cloudB->points[0].b:"<<change_cloudB->points[0].b<<endl;*/

  KDtree tree;
  CUDA_KDTree GPU_tree;
  int max_tree_levels = 13; // play around with this value to get the best result

  vector<KDPoint> data(shrinked_cloud->size());

  for(int i=0; i<shrinked_cloud->size(); i++){
    data[i].coords[0] = shrinked_cloud->points[i].x;
    data[i].coords[1] = shrinked_cloud->points[i].y;
    data[i].coords[2] = shrinked_cloud->points[i].z;
  }

  tree.Create(data, max_tree_levels);
  GPU_tree.CreateKDTree(tree.GetRoot(), tree.GetNumNodes(), data);

  for(int k=0; k<change_cluster.size(); k++){
    vector<KDPoint> queries(change_cluster[k].mypoints.size());

    for(int i=0; i<change_cluster[k].mypoints.size(); i++){
      queries[i].coords[0] = change_cluster[k].mypoints[i].x;
      queries[i].coords[1] = change_cluster[k].mypoints[i].y;
      queries[i].coords[2] = change_cluster[k].mypoints[i].z;
    }

    vector <int> gpu_indexes;
    vector <float> gpu_dists;

    GPU_tree.Search(queries, gpu_indexes, gpu_dists);

    for(int i=0; i<change_cluster[k].mypoints.size(); i++){
      shrinked_cloud->points[gpu_indexes[i]].r = 10*(k+1);
      shrinked_cloud->points[gpu_indexes[i]].g = 0;
      shrinked_cloud->points[gpu_indexes[i]].b = 0;
    }
  }

  CPointCloudAnalysis cPointCloudAnalysis;

  /******************detect table************************/
  PointCloudPtr_RGB_NORMAL tabletopCloud(new PointCloud_RGB_NORMAL());
  PointCloudPtr_RGB_NORMAL planeCloud(new PointCloud_RGB_NORMAL());
  PointCloudPtr rect_cloud(new PointCloud());
  PointCloudPtr_RGB_NORMAL remainingCloud(new PointCloud_RGB_NORMAL());
  pcl::ModelCoefficients coefficients;

  detect_table(shrinked_cloud, coefficients, planeCloud, rect_cloud, remainingCloud);

  MyPointCloud_RGB_NORMAL tablePoint;
  for(int i=0;i<planeCloud->size();i++){
    MyPt_RGB_NORMAL point;
    point.x = planeCloud->at(i).x;
    point.y = planeCloud->at(i).y;
    point.z = planeCloud->at(i).z;
    point.r = 255;
    point.g = 255;
    point.b = 0;
    tablePoint.mypoints.push_back(point);
  }

  cPointCloudAnalysis.tablePoint = tablePoint;
  cPointCloudAnalysis.sup_plane_normal << coefficients.values[0], coefficients.values[1], coefficients.values[2];
  cPointCloudAnalysis.sup_plane_normal.normalize();

  Eigen::Matrix4f matrix_transform;
  Eigen::Matrix4f matrix_transform_r;

  getTemTransformMatrix(coefficients, matrix_transform, matrix_transform_r);

  getCloudOnTable(remainingCloud, rect_cloud, matrix_transform, matrix_transform_r, tabletopCloud);

  float voxel_resolution = 0.004f;
  float seed_resolution = 0.04f;
  float color_importance = 0;
  float spatial_importance = 0.4f;
  float normal_importance = 1.0f;

  /******************Euclidean Cluster Extraction************************/
  std::vector<MyPointCloud_RGB_NORMAL> cluster_points;

  object_seg_ECE(tabletopCloud, cluster_points);

  for(int i=0;i<cluster_points.size();i++){
    if(cluster_points.at(i).mypoints.size()<200){
      continue;
    }

    double sumcolorR = 0;
    double sumcolorG = 0;
    double sumcolorB = 0;
    for(int j=0; j<cluster_points.at(i).mypoints.size(); j++){
      sumcolorR+=(cluster_points.at(i).mypoints[j].r/255.0);
      sumcolorG+=(cluster_points.at(i).mypoints[j].g/255.0);
      sumcolorB+=(cluster_points.at(i).mypoints[j].b/255.0);
    }

    sumcolorR/=cluster_points.at(i).mypoints.size();
    sumcolorG/=cluster_points.at(i).mypoints.size();
    sumcolorB/=cluster_points.at(i).mypoints.size();

    if(!(sumcolorR<=1.0&&sumcolorR>=0.98&&sumcolorG<=1.0&&sumcolorG>=0.98&&sumcolorB<=1.0&&sumcolorB>=0.98)){
      continue;
    }

    PointCloudT::Ptr colored_cloud(new PointCloudT);
    vector<MyPointCloud_RGB_NORMAL> patch_clouds;
    PointNCloudT::Ptr normal_cloud(new PointNCloudT);

    PointCloudPtr_RGB_NORMAL ct(new PointCloud_RGB_NORMAL);
    MyPointCloud_RGB_NORMAL2PointCloud_RGB_NORMAL(cluster_points.at(i), ct);

    VCCS_over_segmentation(ct,voxel_resolution,seed_resolution,color_importance,spatial_importance,normal_importance,patch_clouds,colored_cloud,normal_cloud);

    //PointCloudT::Ptr new_colored_cloud(new PointCloudT);
    vector<MyPointCloud_RGB_NORMAL> new_patch_clouds;
    PointNCloudT::Ptr new_normal_cloud(new PointNCloudT);

    vector<MyPointCloud_RGB_NORMAL> merged_patch_clouds(change_cluster.size());
    vector<pcl::PointNormal> merged_normals(change_cluster.size());
    vector<int> patch_num_vec(change_cluster.size());

    for(int j=0; j<change_cluster.size(); j++){
      merged_normals[j].x = 0;
      merged_normals[j].y = 0;
      merged_normals[j].z = 0;
      merged_normals[j].normal_x = 0;
      merged_normals[j].normal_y = 0;
      merged_normals[j].normal_z = 0;

      patch_num_vec[j] = 0;
    }

    for(int j=0; j<patch_clouds.size(); j++)
    {
      MyPointCloud_RGB_NORMAL pt_tem = patch_clouds[j];
      int count = 0;
      int id = -1;

      for(int k=0; k<pt_tem.mypoints.size(); k++){
        for(int m=0; m<change_cluster.size(); m++){
          if(pt_tem.mypoints[k].r==(m+1)*10){
            pt_tem.mypoints[k].r=255;
            pt_tem.mypoints[k].g=255;
            pt_tem.mypoints[k].b=255;
            id=m;
            count++;
          }
        }
      }

      if(count*1.0/pt_tem.mypoints.size()>0.5){
        for(int k=0; k<pt_tem.mypoints.size(); k++){
          merged_patch_clouds[id].mypoints.push_back(pt_tem.mypoints[k]);
        }

        patch_num_vec[id]++;

        merged_normals[id].x += normal_cloud->at(j).x;
        merged_normals[id].y += normal_cloud->at(j).y;
        merged_normals[id].z += normal_cloud->at(j).z;
        merged_normals[id].normal_x += normal_cloud->at(j).normal_x;
        merged_normals[id].normal_y += normal_cloud->at(j).normal_y;
        merged_normals[id].normal_z += normal_cloud->at(j).normal_z;
      }
      else{
        new_patch_clouds.push_back(pt_tem);
        new_normal_cloud->push_back(normal_cloud->at(j));
      }
    }

    for(int j=0; j<merged_patch_clouds.size(); j++){
      if(merged_patch_clouds[j].mypoints.size()>0){
        new_patch_clouds.push_back(merged_patch_clouds[j]);
        merged_normals[j].x/=patch_num_vec[j];
        merged_normals[j].y/=patch_num_vec[j];
        merged_normals[j].z/=patch_num_vec[j];

        Eigen::Vector3f nor_tem(merged_normals[j].normal_x, merged_normals[j].normal_y, merged_normals[j].normal_z);
        nor_tem.normalize();

        merged_normals[j].normal_x = nor_tem[0];
        merged_normals[j].normal_y = nor_tem[1];
        merged_normals[j].normal_z = nor_tem[2];

        new_normal_cloud->push_back(merged_normals[j]);

        //PointCloudPtr_RGB_NORMAL tem(new PointCloud_RGB_NORMAL);
        //MyPointCloud_RGB_NORMAL2PointCloud_RGB_NORMAL(merged_patch_clouds[j], tem);
        //showPointCloud3(tem, "tem");
      }
    }

    //add normal, point cloud, cluster patch num
    for(int j=0;j<new_patch_clouds.size();j++)
    {
      Normalt nor;
      pcl::PointNormal pn=new_normal_cloud->at(j);
      nor.normal_x = pn.normal_x;
      nor.normal_y = pn.normal_y;
      nor.normal_z = pn.normal_z;
      double normalizeValue = pow(nor.normal_x,2) + pow(nor.normal_y,2) + pow(nor.normal_z,2);
      nor.normal_x /= normalizeValue;
      nor.normal_y /= normalizeValue;
      nor.normal_z /= normalizeValue;
      cPointCloudAnalysis.vecPatcNormal.push_back(nor);
    }

    cPointCloudAnalysis.vecPatchPoint.insert(cPointCloudAnalysis.vecPatchPoint.end(),new_patch_clouds.begin(),new_patch_clouds.end());
    cPointCloudAnalysis.clusterPatchNum.push_back(new_patch_clouds.size());
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
      objectNum++;
      double r,g,b;
      r = double(rand()%255);
      g = double(rand()%255);
      b = double(rand()%255);

      ObjectAttri oba = {r, g, b, 0, 0, 0, 0};
      obas.push_back(oba);

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

          object_cloud->push_back(po);
          objectIndexs.push_back(objectNum);
        }
      }
    }
  }

  int count_tem=0;
  for(int i = 0; i < cPointCloudAnalysis.vecAreaInterest.size();i++)
  {
    cPointCloudAnalysis.vecAreaInterest[i].ComputeScore();
    if(!cPointCloudAnalysis.vecAreaInterest[i].validFlag)	continue;
    for(int j = 0; j < cPointCloudAnalysis.vecAreaInterest[i].vecObjectHypo.size();j++)
    {
      double rObj,gObj,bObj;
      GetColour(double(650) - cPointCloudAnalysis.vecAreaInterest[i].vecObjectHypo[j].objectness,300,350,rObj,gObj,bObj);
      rObj *= 255;
      gObj *= 255;
      bObj *= 255;

      if(rObj>255){
        rObj = 255;
      }

      if(gObj>255){
        gObj = 255;
      }

      if(bObj>255){
        bObj = 255;
      }

      obas[count_tem].cR = rObj;
      obas[count_tem].cG = gObj;
      obas[count_tem].cB = bObj;
      obas[count_tem].confidence = cPointCloudAnalysis.vecAreaInterest[i].vecObjectHypo[j].objectness;
      count_tem++;

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
          po.r = rObj;
          po.g = gObj;
          po.b = bObj;

          confidence_cloud->push_back(po);
        }
      }
    }
  }

}
