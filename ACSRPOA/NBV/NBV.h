#ifndef NBV_H
#define NBV_H

#include <windows.h>
#include <iostream>
#include <fstream>
#include <wrap/io_trimesh/import.h>
#include <wrap/io_trimesh/export.h>
#include <eigenlib/Eigen/Dense>

#include <tbb/parallel_for.h>
#include <tbb/concurrent_vector.h>

#include "../Common/CMesh.h"
#include "TriMesh.h"
#include "ANN/ANN.h"
#include "vcg/complex/trimesh/point_sampling.h"
#include "grid.h"

using namespace std;
using namespace vcg;

#define LINKED_WITH_TBB

#ifndef PI
#define PI float(3.1415926535897932384626433832795)
#endif
/*
TODO:
1. kinect扫描范围 写成参数配置文件
2. 过滤掉机器人达到不了的NBV
*/

/*global variables*/
extern CMesh original;
extern CMesh sample;
extern CMesh field_points;
extern CMesh tentative_mesh;
extern CMesh iso_points;

//NBV
typedef pair<Point3f, Point3f> ScanCandidate;

extern CMesh view_grid_points;
extern CMesh nbv_candidates;
extern vector<ScanCandidate>  scan_candidates;
extern Point3f                whole_space_box_max;
extern Point3f                whole_space_box_min;
extern Box3f                  whole_space_box;

const int grid_resolution = 100;
const double camera_far_dist = 350.0f;     //cm 120cm ~ 350cm
const double camera_near_dist = 120.0f;
const double predicted_model_size = 20.0f; //cm
const double camera_far_dist_virtual = camera_far_dist / predicted_model_size;
const double camera_near_dist_virtual = camera_near_dist / predicted_model_size;
const double max_ray_steps = 1.5f;
const int view_bin_each_axis = 10;

extern double grid_step_size;
extern int x_max;
extern int y_max;
extern int z_max;

/*采样函数*/
class BaseSampler
{
public:
  BaseSampler(CMesh* _m){m=_m; uvSpaceFlag = false; qualitySampling=false; /*tex=0*/;};
  CMesh *m;
  //QImage* tex;
  int texSamplingWidth;
  int texSamplingHeight;
  bool uvSpaceFlag;
  bool qualitySampling;

  void AddVert(const CMesh::VertexType &p);

  void AddFace(const CMesh::FaceType &f, CMesh::CoordType p);

  //void AddTextureSample(const CMesh::FaceType &f, const CMesh::CoordType &p, const Point2i &tp, float edgeDist)
  //{
  //  if (edgeDist != .0) return;

  //  tri::Allocator<CMesh>::AddVertices(*m,1);

  //  if(uvSpaceFlag) m->vert.back().P() = Point3f(float(tp[0]),float(tp[1]),0); 
  //  else m->vert.back().P() = f.P(0)*p[0] + f.P(1)*p[1] +f.P(2)*p[2];

  //  m->vert.back().N() = f.V(0)->N()*p[0] + f.V(1)->N()*p[1] +f.V(2)->N()*p[2];
  //  if(tex)
  //  {
  //    QRgb val;
  //    // Computing normalized texels position
  //    int xpos = (int)(tex->width()  * (float(tp[0])/texSamplingWidth)) % tex->width();
  //    int ypos = (int)(tex->height() * (1.0- float(tp[1])/texSamplingHeight)) % tex->height();

  //    if (xpos < 0) xpos += tex->width();
  //    if (ypos < 0) ypos += tex->height();

  //    val = tex->pixel(xpos,ypos);
  //    m->vert.back().C().SetRGB(qRed(val),qGreen(val),qBlue(val));
  //  }
  //}

}; // end class BaseSampler

void loadMesh(char *fileName, CMesh &mesh);
void saveMesh(char *fileName, CMesh &target);
void normalizeROSA_Mesh(CMesh& mesh);
void runPoissonExe();
void extractPoissonConfidence();
void extractIsoPoints();
void computeHoleConfidence();
void buildGrid();
void propagate();
void viewExtractionIntoBins(int view_bin_each_axis = 10);
void viewPrune();
void clearCMesh(CMesh &mesh);

#endif