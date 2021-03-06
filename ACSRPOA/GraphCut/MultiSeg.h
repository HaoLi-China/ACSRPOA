#pragma once

#include "graph.h"
#include "GCoptimization.h"
#include "BinarySeg.h"
#include "../Common/common_type.h"
#include "GraphCutBasicStruct.h"


class CMultiSeg
{
public:
	CMultiSeg(void);
	~CMultiSeg(void);

public:
	vector<MyPointCloud_RGB_NORMAL> vecPatchPoint;
	vector<pair<int,int>> vecpairPatchConnection;
	vector<double> vecSmoothValue;
	vector<double> vecDataValue;

	vector<MyPoint> vecPatchCenPoint;
	vector<COLORMODEL> vecObjectColorModel;
	vector<ColorType> vecPatchColor;
	double boundingBoxSize;

	vector<double> vecCenterDistance;
	vector<double> vecColorSimilarity;
	vector<double> vecObjectCount;

	vector<int> clusterPatchNumLocal;
	vector<int> clusterPatchInitIndexLocal;

	vector<int> clusterPatchNum;
	vector<int> clusterPatchInitIndex;
	vector<int> clusterPatchInterval;
	vector<int> vecObjectClusteringIndex;  

	vector<double> vecObjectness;
	vector<double> vecSeparateness;
	vector<pair<int,int>> vecpairSeperatenessEdge;
	vector<vector<pair<int,int>>> vecvecpairSeperatenessSmallEdge;

	vector<ObjectHypo> vecObjectHypo;
	vector<EdgeHypo> vecEdgeHypo;

	vector<vector<bool>> vecvecPatchConnectFlag;

	GRAPHSHOW graphContract;
	vector<vector<int>> vecvecMultiResult;
	
	vector<vector<int>> vecvecObjectPoolClustering;
	vector<int> vecObjectPoolClusteringCount;

	double xMin,xMax,yMin,yMax,zMin,zMax;

public:
	void Clear();
	void MainStep();
	void GetColorModel();
	void AddObjectPool();
	double GetMultiDataValue(int SiteID,int LableID);
	void GraphCutSolve();
	void ComputeHypo();
	void ComputeObjectness(int m);
	void ComputeSeparateness(int m,int n);
	void ConstructGraph();
	int GetAreaIndex(int patchIndex);
};

