#include "file_io.h"

bool loadPointCloud_pcd(char* fileName, PointCloudPtr_RGB_NORMAL cloud){

  if(pcl::io::loadPCDFile<Point_RGB_NORMAL>(fileName, *cloud) == -1){
    PCL_ERROR("Could't read file! \n");
    return false;
  }

  return true;
}

bool loadPointCloud_ply(char* fileName, PointCloudPtr_RGB cloud){

  std::ifstream input(fileName) ;
  if(input.fail()) {
    std::cout<<"could not open file!" << std::endl;
    return false;
  }

  int num_header = 14;
  int num_points = 0;
  // this line contains point number
  int line_num = 3;

  for (int i=0; i<num_header; ++i) {
    std::string line;
    getline(input, line);

    if (i==line_num) {
      std::istringstream line_input(line);
      std::string dummy1;
      std::string dummy2;
      line_input >> dummy1 >> dummy2 >>num_points;

      printf("num_points:%d\n",num_points);
    }
  }

  std::cout<< "===========================" <<std::endl;

  for (int i=0; i<num_points; ++i) {
    Point_RGB point_tem;
    int alpha=0;
    int r,g,b;

    input >> point_tem.x >> point_tem.y >> point_tem.z >> r >> g >> b >> alpha;

    point_tem.r=r;
    point_tem.g=g;
    point_tem.b=b;
    point_tem.a=i;//save point index

    cloud->push_back(point_tem);
  }

  return true;
}

//bool loadPointCloud_normal_ply(char* fileName, PointCloudPtr_RGB_NORMAL cloud,MeshFace &meshFace,MeshVertex &meshVertex){
//
//  std::ifstream input(fileName) ;
//  if(input.fail()) {
//    std::cout<<"could not open file!" << std::endl;
//    return false;
//  }
//
//  int num_header = 17;
//  int num_points = 0;
//  int num_faces = 0;
//  // this line contains point number
//  int line_num = 3;
//  int face_num = 14;
//
//  for (int i=0; i<num_header; ++i) {
//    std::string line;
//    getline(input, line);
//
//    if (i==line_num) {
//      std::istringstream line_input(line);
//      std::string dummy1;
//      std::string dummy2;
//      line_input >> dummy1 >> dummy2 >>num_points;
//
//      printf("num_points:%d\n",num_points);
//    }
//	if (i==face_num) {
//		std::istringstream line_input(line);
//		std::string dummy1;
//		std::string dummy2;
//		line_input >> dummy1 >> dummy2 >>num_faces;
//
//		printf("num_faces:%d\n",num_faces);
//	}
//  }
//
//  std::cout<< "===========================" <<std::endl;
//
//  for (int i=0; i<num_points; ++i) {
//    Point_RGB_NORMAL point_tem;
//    int alpha=0;
//    int r,g,b;
//
//    input >> point_tem.x >> point_tem.y >> point_tem.z >>point_tem.normal_x>>point_tem.normal_y>>point_tem.normal_z>>r >> g >> b >> alpha;
//
//    point_tem.r=r;
//    point_tem.g=g;
//    point_tem.b=b;
//	point_tem.a=i;//save point index
//
//    cloud->push_back(point_tem);
//
//	meshVertex.vecVertex.push_back(point_tem);
//  }
//  meshVertex.objectColor.resize(num_points);
//  meshVertex.objectnessColor.resize(num_points);
//  meshVertex.patchColor.resize(num_points);
//
//
//  for (int i=0; i<num_faces; ++i) {
//	  Face3 face_tem;
//	  int temp;
//	  int r,g,b;
//
//	  input >> temp >> face_tem.p0 >> face_tem.p1 >>face_tem.p2;
//
//	 meshFace.vecFace.push_back(face_tem);
//  }
//
//  return true;
//}

bool loadPointCloud_normal_ply(char* fileName, PointCloudPtr_RGB_NORMAL cloud){

  std::ifstream input(fileName) ;
  if(input.fail()) {
    std::cout<<"could not open file!" << std::endl;
    return false;
  }

  int num_header = 17;
  int num_points = 0;
  // this line contains point number
  int line_num = 3;

  for (int i=0; i<num_header; ++i) {
    std::string line;
    getline(input, line);

    if (i==line_num) {
      std::istringstream line_input(line);
      std::string dummy1;
      std::string dummy2;
      line_input >> dummy1 >> dummy2 >>num_points;

      printf("num_points:%d\n",num_points);
    }
  }

  std::cout<< "===========================" <<std::endl;

  for (int i=0; i<num_points; ++i) {
    Point_RGB_NORMAL point_tem;
    int alpha=0;
    int r,g,b;

    input >> point_tem.x >> point_tem.y >> point_tem.z >>point_tem.normal_x>>point_tem.normal_y>>point_tem.normal_z>>r >> g >> b >> alpha;

    point_tem.r=r;
    point_tem.g=g;
    point_tem.b=b;

    cloud->push_back(point_tem);
  }

  return true;
}
