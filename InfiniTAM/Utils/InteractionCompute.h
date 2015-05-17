#pragma once

#include "../Common/common_type.h"
#include "../Common/common_func.h"
#include "../Common/color_op.h"
#include "../GraphCut/graph.h"
#include "../GraphCut/kdtree.h"
#include "../GraphCut/PointCloudAnalysis.h"

#define LARGE_NUM 9999999
#define SMALL_NUM -9999999

struct PatchPointScore
{
	vector<double> vecScore;
	int patchIndex;
	vector<bool> vecInvalidFlag;
};

typedef struct COLOUR{
	double r;
  double g;
  double b;
};

struct NewObjectHypo
{
  vector<int> patchIndex;
  int areaIndex;
  double objectness;
  bool mergeFlag;
  MyPoint centerPoint;
  double minHeight;
  double maxHeight;
};

class CInteractionCompute
{
public:
  vector<NewObjectHypo> vecObjectHypo;
  vector<MyPointCloud_RGB_NORMAL> vecObjectPoints;
  PatchPointScore ptScore;
  Eigen::Vector3d sup_plane_normal;
	
	double paraNearObject;
	double fMax,fMin;

public:
	CInteractionCompute(CPointCloudAnalysis &cPointCloudAnalysis);
	~CInteractionCompute(void);
  void getTouchPointAndDir(const int objectId, Eigen::Vector3f &position, Eigen::Vector3f &direction, bool debug);
  void Preprocessing(const int objectId);
  void Filter(const int objectId);
  bool IsAccessible(const MyPt_RGB_NORMAL point, const int objectId);
  bool IsExistBlock(const MyPt_RGB_NORMAL point, const int objectId);
  void GetNeighborObject(const int objectId, const double range_dis, vector<int> &nearObject);
  double GetFs(const int objectId, vector<int> &neighborObject, MyPt_RGB_NORMAL point);
  void ComputeScore(const int objectId, int &max_score_index);
};

