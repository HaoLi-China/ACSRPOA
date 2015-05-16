//#include "StdAfx.h"
#include "InteractionCompute.h"

//get transform matrix between plane and x_y plane 
void GetTempTransformMatrix(Eigen::Vector3d normal, Eigen::Matrix4f& matrix_transform, Eigen::Matrix4f& matrix_transform_r){
  normal.normalize();

  if(normal.dot(Eigen::Vector3d(0,0,1))>0){
    normal = -normal;
  }

  double angle=acos(normal.dot(Eigen::Vector3d(0,0,1)));
  Eigen::Vector3d axis=normal.cross(Eigen::Vector3d(0,0,1));
  axis.normalize();

  Eigen::Matrix4d matrix;
  getRotationMatrix(axis, angle, matrix);
  matrix_transform = matrix.cast<float>();

  getRotationMatrix(axis, -angle, matrix);
  matrix_transform_r = matrix.cast<float>();
}

CInteractionCompute::CInteractionCompute(CPointCloudAnalysis &cPointCloudAnalysis)
{
  paraNearObject = 0.1;
  fMax = SMALL_NUM;
  fMin = LARGE_NUM;

  sup_plane_normal = cPointCloudAnalysis.sup_plane_normal;
  
  for(int i = 0; i < cPointCloudAnalysis.vecAreaInterest.size();i++)
  {
    for(int j = 0; j < cPointCloudAnalysis.vecAreaInterest[i].vecObjectHypo.size();j++)
    { 
      NewObjectHypo tem;
      ObjectHypo oh = cPointCloudAnalysis.vecAreaInterest[i].vecObjectHypo[j];
      tem.patchIndex = oh.patchIndex;
      tem.areaIndex = oh.areaIndex;
      tem.objectness = oh.objectness;
      tem.mergeFlag = oh.mergeFlag;
      tem.centerPoint = oh.cenPoint;
      tem.minHeight =0;
      tem.maxHeight =0;
      vecObjectHypo.push_back(tem);

      MyPointCloud_RGB_NORMAL cloud_tem;

      for(int k = 0; k < cPointCloudAnalysis.vecAreaInterest[i].vecObjectHypo[j].patchIndex.size();k++)
      {
        int patchIndex = cPointCloudAnalysis.vecAreaInterest[i].vecObjectHypo[j].patchIndex[k];
        for(int m = 0; m < cPointCloudAnalysis.vecAreaInterest[i].vecPatchPoint[patchIndex].mypoints.size();m++)
        {
          MyPoint_RGB_NORMAL point = cPointCloudAnalysis.vecAreaInterest[i].vecPatchPoint[patchIndex].mypoints[m];
          cloud_tem.mypoints.push_back(point);
        }
      }
      vecObjectPoints.push_back(cloud_tem);
    }
  }

  for(int i=0; i<vecObjectHypo.size(); i++){
    NewObjectHypo tem0 = vecObjectHypo[i];
    MyPointCloud_RGB_NORMAL tem1 = vecObjectPoints[i];
    int id = i;
    for(int j=i+1; j<vecObjectHypo.size(); j++){
      if(tem0.objectness < vecObjectHypo[j].objectness){
        tem0 = vecObjectHypo[j];
        tem1 = vecObjectPoints[j];
        id=j;
      }
    }
    vecObjectHypo[id] = vecObjectHypo[i];
    vecObjectHypo[i] = tem0;

    vecObjectPoints[id] = vecObjectPoints[i];
    vecObjectPoints[i] = tem1;
  }

  Eigen::Matrix4f matrix_transform;
  Eigen::Matrix4f matrix_transform_r;
  GetTempTransformMatrix(sup_plane_normal, matrix_transform, matrix_transform_r);

  for(int i=0; i<vecObjectPoints.size(); i++){
  PointCloudPtr_RGB_NORMAL pc(new PointCloud_RGB_NORMAL);
  PointCloudPtr_RGB_NORMAL tem(new PointCloud_RGB_NORMAL);
  MyPointCloud_RGB_NORMAL2PointCloud_RGB_NORMAL(vecObjectPoints[i], pc);
  pcl::transformPointCloud (*pc, *tem, matrix_transform);

  for (int j=0; j<tem->size(); ++j) {
    float x, y, z;
    x=tem->points[i].x;
    y=tem->points[i].y;
    z=tem->points[i].z;

    if(z<(vecObjectHypo[i].minHeight)){
      vecObjectHypo[i].minHeight=z;
    }
    else if(z>(vecObjectHypo[i].maxHeight)){
      vecObjectHypo[i].maxHeight=z;
    }
  }
  }
}

CInteractionCompute::~CInteractionCompute(void)
{
}

void CInteractionCompute::getTouchPointAndDir(const int objectId, Eigen::Vector3f &position, Eigen::Vector3f &direction){
  Preprocessing(objectId);
  Filter(objectId);
  int max_score_index;
  ComputeScore(objectId, max_score_index);

  position[0] = vecObjectPoints[objectId].mypoints[max_score_index].x;
  position[1] = vecObjectPoints[objectId].mypoints[max_score_index].y;
  position[2] = vecObjectPoints[objectId].mypoints[max_score_index].z;

  direction[0] = -vecObjectPoints[objectId].mypoints[max_score_index].normal_x;
  direction[1] = -vecObjectPoints[objectId].mypoints[max_score_index].normal_y; 
  direction[2] = 0; 
}

void CInteractionCompute::Preprocessing(const int objectId){
  ptScore.vecScore.resize(vecObjectPoints[objectId].mypoints.size(),0);
  ptScore.patchIndex = objectId;
  ptScore.vecInvalidFlag.resize(vecObjectPoints[objectId].mypoints.size(),true);
}

void CInteractionCompute::Filter(const int objectId){

  Eigen::Matrix4f matrix_transform;
  Eigen::Matrix4f matrix_transform_r;
  GetTempTransformMatrix(sup_plane_normal, matrix_transform, matrix_transform_r);

  PointCloudPtr_RGB_NORMAL pc(new PointCloud_RGB_NORMAL);
  PointCloudPtr_RGB_NORMAL tem(new PointCloud_RGB_NORMAL);
  MyPointCloud_RGB_NORMAL2PointCloud_RGB_NORMAL(vecObjectPoints[objectId], pc);
  pcl::transformPointCloud (*pc, *tem, matrix_transform);

  for(int k = 0;k < vecObjectPoints[objectId].mypoints.size();k++)
  {
    MyPt_RGB_NORMAL point = vecObjectPoints[objectId].mypoints[k];

    //Lower push
    if(pc->points[k].z - vecObjectHypo[objectId].minHeight > (vecObjectHypo[objectId].maxHeight - vecObjectHypo[objectId].minHeight) * 2.0 / 3 || pc->points[k].z  - vecObjectHypo[objectId].minHeight < (vecObjectHypo[objectId].maxHeight - vecObjectHypo[objectId].minHeight) * 1.0 / 5)
    {
      ptScore.vecInvalidFlag[k] = false;
      continue;
    }

    ////Support boundary
    //MyPt_RGB_NORMAL pointGoNormal;
    //pointGoNormal.x = point.x + (-point.normal_x) * vecObjectHypo[objectId].width;
    //pointGoNormal.y = point.y + (-point.normal_y) * vecObjectHypo[objectId].width;
    //if(pointGoNormal.x > 10 || pointGoNormal.x < -10|| pointGoNormal.y > 16|| pointGoNormal.y < -16)   //para
    //{
    //  ptScore.vecInvalidFlag[k] = false;
    //  continue;
    //}

    //Horizontal push
    double horiValue;
    horiValue = point.normal_x * sup_plane_normal[0] + point.normal_y * sup_plane_normal[1] + point.normal_z * sup_plane_normal[2];
    if(horiValue > 0.2 || horiValue < -0.2)
    {
      ptScore.vecInvalidFlag[k] = false;
      continue;
    }

    //if exist block
    if(IsExistBlock(point, objectId))
    {
      ptScore.vecInvalidFlag[k] = false;
      continue;
    }

    //Accessibility
    if(!IsAccessible(point, objectId))
    {
      ptScore.vecInvalidFlag[k] = false;
      continue;
    }
  }
}

bool CInteractionCompute::IsExistBlock(const MyPt_RGB_NORMAL point, const int objectId){
  vector<int> neighborObject;
  GetNeighborObject(objectId, 0.1, neighborObject);

  for(int i = 0;i < neighborObject.size();i++)
  {
    int currentId = neighborObject[i];
    if(objectId == currentId) continue;
    for(int k = 0; k < vecObjectPoints[currentId].mypoints.size(); k++)
    {
      MyPt_RGB_NORMAL pointi = vecObjectPoints[currentId].mypoints[k];
      double dis_real = sqrt((pointi.x - point.x) * (pointi.x - point.x) + (pointi.y - point.y) * (pointi.y - point.y) + (pointi.z - point.z) * (pointi.z - point.z));
      double dis_hor = (pointi.x - point.x) * point.normal_x + (pointi.y - point.y) * point.normal_y + (pointi.z - point.z) * point.normal_z;
      
      if(dis_hor < 0 && dis_hor > -0.1){
        double dis_rad = sqrt(dis_real*dis_real - dis_hor*dis_hor);
        if(dis_rad < 0.05)  //para
          return true;
      }
    }
  }
  return false;
}

bool CInteractionCompute::IsAccessible(const MyPt_RGB_NORMAL point, const int objectId)
{
  vector<int> neighborObject;
  GetNeighborObject(objectId, 0.35, neighborObject);

  for(int i = 0;i < neighborObject.size();i++)
  {
    int currentId = neighborObject[i];
    if(objectId == currentId) continue;
    for(int k = 0; k < vecObjectPoints[currentId].mypoints.size(); k++)
    {
      MyPt_RGB_NORMAL pointi = vecObjectPoints[currentId].mypoints[k];
      double dis_real = sqrt((pointi.x - point.x) * (pointi.x - point.x) + (pointi.y - point.y) * (pointi.y - point.y) + (pointi.z - point.z) * (pointi.z - point.z));
      double dis_hor = (pointi.x - point.x) * point.normal_x + (pointi.y - point.y) * point.normal_y + (pointi.z - point.z) * point.normal_z;

      if(dis_hor > 0 && dis_hor < 0.3){
        double dis_rad = sqrt(dis_real*dis_real - dis_hor*dis_hor);
        if(dis_rad < 0.05)  //para
          return false;
      }
    }
  }
  return true;
}

void CInteractionCompute::GetNeighborObject(const int objectId, const double range_dis, vector<int> &nearObject)
{
  MyPoint centerPointm = vecObjectHypo[objectId].centerPoint;
  for(int i = 0;i < vecObjectHypo.size();i++)
  {
    if(i == objectId)		
      continue;

    MyPoint centerPointi = vecObjectHypo[i].centerPoint;
    double dis = (centerPointi.x - centerPointm.x) * (centerPointi.x - centerPointm.x) +
      (centerPointi.y - centerPointm.y) * (centerPointi.y - centerPointm.y) +
      (centerPointi.z - centerPointm.z) * (centerPointi.z - centerPointm.z);
    dis = sqrt(dis);
    if(dis <= range_dis)
      nearObject.push_back(i);
  }
}

double CInteractionCompute::GetFs(const int objectId, vector<int> &neighborObject, MyPt_RGB_NORMAL point)
{
  if(neighborObject.size() == 0)
    return 1;

  double fsTemp;
  double fs = LARGE_NUM;
  MyPoint centerPointi = vecObjectHypo[objectId].centerPoint;
  for(int i = 0;i < neighborObject.size();i++)
  {
    MyPoint centerPointj = vecObjectHypo[neighborObject[i]].centerPoint;
    MyPoint vji;
    vji.x = centerPointj.x - centerPointi.x;
    vji.y = centerPointj.y - centerPointi.y;
    vji.z = centerPointj.z - centerPointi.z;
    double nomalize = sqrt(vji.x * vji.x + vji.y * vji.y + vji.z * vji.z);
    vji.x /= nomalize;
    vji.y /= nomalize;
    vji.z /= nomalize;

    fsTemp = 1 - (-point.normal_x) * vji.x - (-point.normal_y) * vji.y - (-point.normal_z) * vji.z;
    if(fs > fsTemp)
      fs = fsTemp;
  }

  if (fs > 1)
    fs = 1;

  return fs;
}


void CInteractionCompute::ComputeScore(const int objectId, int &max_score_index)
{
  MyPoint centerPoint = vecObjectHypo[objectId].centerPoint;

  for(int k = 0;k < vecObjectPoints[objectId].mypoints.size();k++)
  {
    if(!ptScore.vecInvalidFlag[k])	continue;

    MyPt_RGB_NORMAL point = vecObjectPoints[objectId].mypoints[k];
    MyPt_RGB_NORMAL pointq;
    pointq.x = centerPoint.x - point.x;
    pointq.y = centerPoint.y - point.y;
    pointq.z = 0;
    double nomalize = sqrt(pointq.x*pointq.x + pointq.y*pointq.y);
    pointq.x /= nomalize;
    pointq.y /= nomalize;
    pointq.z /= nomalize;
    double ft = (-point.normal_x) * pointq.x + (-point.normal_y) * pointq.y + (-point.normal_z) * pointq.z;


    vector<int> neighborObject;
    GetNeighborObject(objectId, paraNearObject, neighborObject);
    double fs = GetFs(objectId,neighborObject,point);
    double f = pow(ft,0.3) * pow(fs,0.7);
    ptScore.vecScore[k] = f;		

    if(f > fMax){
      fMax = f;
      max_score_index = k;
    }
  }
}

//void CInteractionCompute::SavePlyFile(const string& filename,int m)
//{
//	ofstream outFile(filename.c_str());
//	outFile << "ply" << endl;
//	outFile << "format ascii 1.0" << endl;
//	outFile << "comment VCGLIB generated" << endl;
//	outFile << "element vertex " << vecPatchPoint[m].mypoints.size() <<endl;
//	outFile << "property float x" << endl;
//	outFile << "property float y" << endl;
//	outFile << "property float z" << endl;
//// 	outFile << "property float nx" << endl;
//// 	outFile << "property float ny" << endl;
//// 	outFile << "property float nz" << endl;
//	outFile << "property uchar red" << endl;
//	outFile << "property uchar green" << endl;
//	outFile << "property uchar blue" << endl;
//	outFile << "property uchar alpha" << endl;
//	outFile << "element face " << vecPatchFace[m].vecFace.size() <<endl;
//	outFile << "property list uchar int vertex_indices" << endl;
//	outFile << "end_header" << endl;
//
//
//	for(int i=0;i<vecPatchPoint[m].mypoints.size();i++)
//	{
//		MyPt_RGB_NORMAL point = vecPatchPoint[m].mypoints[i];
//		double r,g,b;
//		r = g = b = 1;
//		if(point.r <=1 && point.r>=0)	r = point.r;
//		if(point.g <=1 && point.g>=0)	g = point.g;
//		if(point.b <=1 && point.b>=0)	b = point.b;
//
//		r *= 255;
//		g *= 255;
//		b *= 255;
//
//		r = int(r);
//		g = int(g);
//		b = int(b);
//
//		outFile << point.x << " "<< point.y << " "<<point.z << " "<<
//			r << " "<< g << " "<<b << " "<< "255"<< endl;
//	}	
//
//	for(int i=0;i<vecPatchFace[m].vecFace.size();i++)
//	{
//		int p0,p1,p2;
//		p0 = vecPatchFace[m].vecFace[i].f1;
//		p1 = vecPatchFace[m].vecFace[i].f2;
//		p2 = vecPatchFace[m].vecFace[i].f3;
//
//		outFile <<  "3 "<< p0 << " "<< p1 << " "<< p2 << endl;
//	}	
//	outFile.close();
//}